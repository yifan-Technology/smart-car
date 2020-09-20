import serial
import threading
import struct
import time
from queue import Queue


def init():
    # 全局化节点名称
    global nodeName
    global mapSize
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml", "r") as f:
        config = yaml.load(f)

    mapSize = config["costMap"]["mapSize"]
    # 读取节点名称参数
    nodeName = config["RosTopic"]


class SerialThread:
    """
    串口通信线程，包含读线程和写线程
    """

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, parity=None, bytesize=8, stopbits=1, timeout=1):
        self.my_serial_port = serial.Serial()
        self.my_serial_port.port = port  # 端口号
        self.my_serial_port.baudrate = baudrate  # 波特率
        self.my_serial_port.bytesize = bytesize  # 数据位
        self.my_serial_port.stopbits = stopbits  # 停止位
        self.my_serial_port.timeout = timeout

        self.read_lock = threading.RLock()
        self.write_lock = threading.RLock()

        self.alive = False  # 当 alive 为 True，读写线程会进行

        # self.control_data = [-300.0, -300.0, -300.0, -300.0]
        self.control_data = [220.0, 220.0, 220.0, 220.0]
        # self.control_data = [0.0, 0.0, 0.0, 0.0]
        # self.control_data = [800.0, 800.0, 800.0, 800.0]
        self.read_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.my_serial_port.open()

    def stop(self):
        self.alive = False
        if self.my_serial_port.isOpen():
           self.my_serial_port.close()

    def tryRead(self, length):
        bytes_remaining = length
        result = bytearray()

        while bytes_remaining != 0:
            with self.read_lock:
                received = self.my_serial_port.read(bytes_remaining)
            if len(received) != 0:
                result.extend(received)
                bytes_remaining -= len(received)

        return bytes(result)

    def read(self):
        t = time.time()
        while self.alive:
            try:
                with self.read_lock:
                    n = self.my_serial_port.inWaiting()
                    if n < 1:
                        time.sleep(0.001)
                        continue

                    #print("waited {} bytes".format(n))

                if n == 40:
                    with self.read_lock:
                        myByte = self.my_serial_port.read(40)
                    if myByte[0] == 97 and myByte[1] == 97 and myByte[2] == 97 and myByte[3] == 97:
                        if myByte[-1] == 98 and myByte[-2] == 98 and myByte[-3] == 98 and myByte[-4] == 98:
                            try:
                                data = myByte[4:-4]
                                new_values = struct.unpack('<ffffffff', data)
                                self.read_data = new_values
                                print("read bytes length:", n, " with real data: ", self.read_data)

                                fps = 1 / (time.time() - t)
                                print("fps: ", fps)
                                t = time.time()

                            except Exception as ex:
                                print("struct:")
                                print(ex)

                elif n == 39:
                    with self.read_lock:
                        myByte = self.my_serial_port.read(39)
                    if myByte[0] == 97 and myByte[1] == 97 and myByte[2] == 97 :
                        if myByte[-1] == 98 and myByte[-2] == 98 and myByte[-3] == 98 and myByte[-4] == 98:
                            try:
                                data = myByte[3:-4]
                                new_values = struct.unpack('<ffffffff', data)
                                self.read_data = new_values
                                print("read bytes length:", n, "with real data: ", self.read_data)

                            except Exception as ex:
                                print("struct:")
                                print(ex)

                else:
                    with self.read_lock:
                        myByte = self.my_serial_port.read(n)
                    print("read bytes length:", n, "with my_Byte: ", myByte)


            except Exception as ex:
                print("all:")
                print(ex)

    def write(self):
        while self.alive:
            time.sleep(0.025)

            try:
                start = 99
                end = 100
                data = struct.pack("<B4fB", start, self.control_data[0], self.control_data[1], self.control_data[2],
                                   self.control_data[3], end)
                with self.write_lock:
                    self.my_serial_port.write(data)
                #print("control_data: ", self.control_data)

            except Exception as ex:
                print(ex)


if __name__ == "__main__":
    try:
        Motor_serial = SerialThread("/dev/ttyUSB0")

        t_read = threading.Thread(target=Motor_serial.read, daemon=False)
        t_write = threading.Thread(target=Motor_serial.write, daemon=False)

        Motor_serial.alive = True
        t_read.start()
        t_write.start()

        while True:
            pass

    finally:
        Motor_serial.control_data = [0.0, 0.0, 0.0, 0.0]
        time.sleep(.1)
        Motor_serial.stop()
        time.sleep(.1)
        t_read.join()
        t_write.join()
        time.sleep(.1)
