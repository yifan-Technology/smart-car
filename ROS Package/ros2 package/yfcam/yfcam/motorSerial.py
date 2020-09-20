import serial
import threading
import struct
import time
import yf_node
import rclpy
import yaml
import cv2
import threading
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

        # self.control_data = [800.0, 800.0, 800.0, 800.0]
        # self.control_data = [-300.0, -300.0, -300.0, -300.0]
        # self.control_data = [220.0, 220.0, 220.0, 220.0]
        self.control_data = [0.0, 0.0, 0.0, 0.0]
        
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
        last_time = time.time()

        while self.alive:
            try:
                with self.read_lock:
                    n = self.my_serial_port.inWaiting()
                    if n < 1:
                        time.sleep(0.001)
                        continue

                    #print("waited {} bytes".format(n))

                myByte = self.tryRead(40)
                if myByte[0] == 97 and myByte[1] == 97 and myByte[2] == 97 and myByte[3] == 97 \
                    and myByte[-1] == 98 and myByte[-2] == 98 and myByte[-3] == 98 and myByte[-4] == 98:
                        fps = 1 / (time.time() - last_time)

                        # comment print can increase fps
                        print("fps: ", fps)

                        data = myByte[4:-4]
                        self.read_data = struct.unpack('<ffffffff', data)

                        # comment print can increase fps
                        print("Real data: ", self.read_data)

                        last_time = time.time()

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
                
                # comment print can increase fps
                print("\ncontrol_data: ", self.control_data)

            except Exception as ex:
                print(ex)


def pubSpin(q_node):
    node = q_node.get_nowait()
    print("i will suck")
    while True:
        rclpy.spin_once(node.node,timeout_sec=0.01)
        time.sleep(0.02)
    print("i am back")


def subSpin(q_node):
    node = q_node.get_nowait()
    print("i will suck")
    rclpy.spin(node.node)
    print("i am back")


def main():
    init()
    rclpy.init()
    real = yf_node.YF_RealSpeed(nodeName["RealSpeed"],"RealSpeed","pub")
    soll = yf_node.YF_SollSpeed(nodeName["SollSpeed"],"SollSpeed","sub")
    rclpy.spin_once(soll.node,timeout_sec=0.1)
    
    q_subNode = Queue(1)
    q_pubNode = Queue(1)
    q_subNode.put_nowait(soll)
    q_pubNode.put_nowait(real)

    t_sub = threading.Thread(target=subSpin,args=(q_subNode,))
    t_pub = threading.Thread(target=pubSpin,args=(q_pubNode,))
    t_sub.start()
    t_pub.start()
    Motor_serial = SerialThread("/dev/ttyUSB0")

    t_read = threading.Thread(target=Motor_serial.read, daemon=False)
    t_write = threading.Thread(target=Motor_serial.write, daemon=False)

    Motor_serial.alive = True
    t_read.start()
    t_write.start()

    try:
        while True:#:
            Motor_serial.control_data = soll.subMsg
            real.publishMsg(Motor_serial.read_data)
            time.sleep(0.25)
            

    finally:
        Motor_serial.control_data = [0.0, 0.0, 0.0, 0.0]
        time.sleep(.7)
        Motor_serial.stop()
        time.sleep(.5)
        t_read.join()
        t_write.join()
        time.sleep(.1)


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
        time.sleep(.7)
        Motor_serial.stop()
        time.sleep(.5)
        t_read.join()
        t_write.join()
        time.sleep(.1)