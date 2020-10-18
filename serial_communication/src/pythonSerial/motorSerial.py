import serial
import threading
import struct
import time

class SerialThread:
    """
    串口通信线程，包含读线程和写线程
    """
    def __init__(self, port = 'COM18', baudrate=115200, parity=None, bytesize=8, stopbits=1, timeout=1):
        self.my_serial = serial.Serial()
        self.my_serial.port = port              # 端口号
        self.my_serial.baudrate = baudrate      # 波特率
        self.my_serial.bytesize = bytesize      # 数据位
        self.my_serial.stopbits = stopbits      # 停止位
        self.my_serial.timeout = timeout

        self.alive = False                      # 当 alive 为 True，读写线程会进行
        self.wait_end = None                    # 用来控制主线程
        self.thread_read = None                 # 读线程
        self.thread_write = None                # 写线程
        self.control_data = [800, -800, 200, -200]
        self.read_data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    def start(self):
        self.my_serial.open()
        if self.my_serial.isOpen():
            self.alive = True
            self.wait_end = threading.Event()

            self.thread_read = threading.Thread(target=self.read)
            self.thread_read.setDaemon(True)                        # 当主线程结束，读线程和主线程一并退出

            self.thread_write = threading.Thread(target=self.write)
            self.thread_write.setDaemon(True)                       # 当主线程结束，写线程和主线程一并退出

            self.thread_read.start()
            self.thread_write.start()

            return True

        else:
            return False

    def wait(self):
        if not self.wait_end is None:
            self.wait_end.wait()            # 阻塞主线程

    def stop(self):
        self.alive = False
        if self.my_serial.isOpen():
            self.my_serial.close()


    def read(self):
        while self.alive:
            try:
                n = self.my_serial.inWaiting()                       # 返回接收缓存中的字节数
                if n == 40:
                    myByte = self.my_serial.read(40)
                    if myByte[0] == 97 and myByte[1] == 97 and myByte[2] == 97 and myByte[2] == 97:
                        if myByte[-1] == 98 and myByte[-2] == 98 and myByte[-3] == 98 and myByte[-4] == 98:
                            try:
                                data = myByte[4:-4]
                                new_values = struct.unpack('<ffffffff', data)
                                self.read_data = new_values
                                print(self.read_data)
                                break # 线程结束

                            except Exception as ex:
                                print(ex)
            except Exception as ex:
                print(ex)

        self.wait_end.set()
        self.alive = False

    def write(self):
        while self.alive:

            try:
                start = 99
                end = 100
                data = struct.pack("<B4fB", start, self.control_data[0], self.control_data[1], self.control_data[2],
                                   self.control_data[3], end)
                self.my_serial.write(data)     # 
                #self.my_serial.flush()
                print(self.control_data)
                
            except Exception as ex:
                print(ex)

            time.sleep(0.03)

        self.wait_end.set()
        self.alive = False

if __name__ == "__main__":
    Motor_serial = SerialThread("/dev/ttyUSB0")
    while True:

        if Motor_serial.start():
            Motor_serial.wait()
            Motor_serial.stop()

        if Motor_serial.alive == True:
            Motor_serial.stop()

        time.sleep(5)




    print("END")

    del my_serial