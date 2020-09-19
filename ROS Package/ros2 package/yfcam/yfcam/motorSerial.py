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
    # ???????
    global nodeName  
    global mapSize
    # ??yaml??
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)  
        
    mapSize = config["costMap"]["mapSize"]
    # ????????
    nodeName = config["RosTopic"]


class SerialThread:
    """
    ??????,?????????
    """
    def __init__(self, port, baudrate=115200, parity=None, bytesize=8, stopbits=1, timeout=1):
        self.my_serial = serial.Serial()
        self.my_serial.port = port              # ???
        self.my_serial.baudrate = baudrate      # ???
        self.my_serial.bytesize = bytesize      # ???
        self.my_serial.stopbits = stopbits      # ???
        self.my_serial.timeout = timeout

        self.alive = False                      # ? alive ? True,???????
        self.wait_end = None                    # ???????
        self.thread_read = None                 # ???
        self.thread_write = None                # ???
        self.control_data = [220.0, 220.0, 220.0, 220.0]
        self.read_data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.my_serial.open()

    def start(self):
        #self.my_serial.open()
        if self.my_serial.isOpen():
            self.alive = True
            self.wait_end = threading.Event()

            self.thread_read = threading.Thread(target=self.read)
            self.thread_read.setDaemon(True)                        # ??????,???????????

            self.thread_write = threading.Thread(target=self.write)
            self.thread_write.setDaemon(True)                       # ??????,???????????

            self.thread_read.start()
            self.thread_write.start()

            return True

        else:
            return False

    def wait(self):
        if not self.wait_end is None:
            self.wait_end.wait()            # ?????

    def stop(self):
        self.alive = False
        #if self.my_serial.isOpen():
        #    self.my_serial.close()


    def read(self):
        while self.alive:
            try:
                n = self.my_serial.inWaiting()  # ???????????
                if n > 0:
                    print("waited {} bytes".format(n))

                    if n == 40:
                        myByte = self.my_serial.read(40)
                        if myByte[0] == 97 and myByte[1] == 97 and myByte[2] == 97 and myByte[3] == 97:
                            if myByte[-1] == 98 and myByte[-2] == 98 and myByte[-3] == 98 and myByte[-4] == 98:
                                try:
                                    data = myByte[4:-4]
                                    new_values = struct.unpack('<ffffffff', data)
                                    self.read_data = new_values
                                    print("read  data: ", self.read_data)
                                    break  # ????

                                except Exception as ex:
                                    print("struct:")
                                    print(ex)

                    elif n == 39:
                        myByte = self.my_serial.read(39)
                        if myByte[0] == 97 and myByte[1] == 97 and myByte[2] == 97 :
                            if myByte[-1] == 98 and myByte[-2] == 98 and myByte[-3] == 98 and myByte[-4] == 98:
                                try:
                                    data = myByte[3:-4]
                                    new_values = struct.unpack('<ffffffff', data)
                                    self.read_data = new_values
                                   # print("n: ", n, "real data: ", self.read_data)
                                    break  # ????

                                except Exception as ex:
                                    print("struct:")
                                    print(ex)

                    else:
                        myByte = self.my_serial.read(n)
                        print("n: ", n, "my_Byte: ", myByte)
                        break  # ????

            except Exception as ex:
                print("all:")
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
                self.my_serial.write(data)     # ??? gbk ?(????????)
                print("write: ", self.control_data)
            except Exception as ex:
                print(ex)

            time.sleep(1.0/20.0)

        self.wait_end.set()
        self.alive = False

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
    Motor_serial = SerialThread("/dev/ttyUSB0")
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
    try:
        while 1:#:
            t = time.time()
            # set data
            # rclpy.spin_once(soll.node,timeout_sec=0.02)
            Motor_serial.control_data = soll.subMsg
            #print("soll value: ",  Motor_serial.control_data)
            # pub data
            real.publishMsg(Motor_serial.read_data)
            # rclpy.spin_once(real.node,timeout_sec=0.02)
            #print("real value: ", Motor_serial.read_data[0], Motor_serial.read_data[2],  Motor_serial.read_data[4],  Motor_serial.read_data[6])
            # give it to A
            if Motor_serial.start():
                Motor_serial.wait()
                Motor_serial.stop()

            # if Motor_serial.alive == True:
            #     Motor_serial.stop()

            time.sleep(0.03)

            fps = 1 / (time.time() - t)
            print("fps: ", fps)

            
    finally:
        Motor_serial.my_serial.close()
        Motor_serial = SerialThread("/dev/ttyUSB0")
        Motor_serial.control_data = [0.0,0.0,0.0,0.0]
        if Motor_serial.start():
                Motor_serial.wait()
                Motor_serial.stop()

        if Motor_serial.alive == True:
                Motor_serial.stop()
        time.sleep(1)
        Motor_serial.my_serial.close()


if __name__ == "__main__":
    
    try:
        Motor_serial = SerialThread("/dev/ttyUSB0")

        while True:        
           
           # give it to A
            if Motor_serial.start():
                Motor_serial.wait()
                Motor_serial.stop()

            if Motor_serial.alive == True:
                Motor_serial.stop()

            time.sleep(0.03)
    finally:

        Motor_serial.my_serial.close()
        Motor_serial = SerialThread("/dev/ttyUSB0")
        Motor_serial.control_data = [0.0,0.0,0.0,0.0]
        if Motor_serial.start():
                Motor_serial.wait()
                Motor_serial.stop()

        if Motor_serial.alive == True:
                Motor_serial.stop()
        time.sleep(1)
        Motor_serial.my_serial.close()
