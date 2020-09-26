import serial
import threading
import struct
import time
import yf_node
import rclpy
import yaml
import threading
from queue import Queue


class SerialThread:
    """
    串口通信线程，包含读线程和写线程
    """

    def __init__(self, soll_node, real_node, port="/dev/ttyUSB0", baudrate=115200, parity=None, bytesize=8, stopbits=1, timeout=1):
        self.my_serial_port = serial.Serial()
        self.my_serial_port.port = port  # 端口号
        self.my_serial_port.baudrate = baudrate  # 波特率
        self.my_serial_port.bytesize = bytesize  # 数据位
        self.my_serial_port.stopbits = stopbits  # 停止位
        self.my_serial_port.timeout = timeout

        
        self._soll = soll_node
        self._real = real_node

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
                      
                        self._real.publishMsg(self.read_data)

                        last_time = time.time()

            except Exception as ex:
                print("all:")
                print(ex)
                with self.read_lock:
                    self.my_serial_port.reset_input_buffer()

    def write(self):
        with self._soll.write_queue.mutex:
            self._soll.write_queue.clear()

        while self.alive:
            if self._soll.write_queue.empty():
                time.sleep(0.01)
            else:
                control_data = self._soll.write_queue.get()

                try:
                    start = 99
                    end = 100
                    data = struct.pack("<B4fB", start, control_data[0], control_data[1], control_data[2], control_data[3], end)
                    
                    with self.write_lock:
                        self.my_serial_port.write(data)
                    
                    # comment print can increase fps
                    print("\ncontrol_data: ", control_data)

                except Exception as ex:
                    print(ex)
                    with self.write_lock:
                        self.my_serial_port.reset_output_buffer()


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

    Motor_serial = SerialThread(soll, real, "/dev/ttyUSB0")

    t_read = threading.Thread(target=Motor_serial.read, daemon=False)
    t_write = threading.Thread(target=Motor_serial.write, daemon=False)

    Motor_serial.alive = True
    t_read.start()
    t_write.start()

    try:
        while True:
            pass
            
    finally:
        Motor_serial.control_data = [0.0, 0.0, 0.0, 0.0]
        time.sleep(.7)
        with Motor_serial.read_lock:
            Motor_serial.my_serial_port.reset_input_buffer()
        with Motor_serial.write_lock:
            Motor_serial.my_serial_port.reset_output_buffer()
        Motor_serial.stop()
        time.sleep(.5)
        t_read.join()
        t_write.join()
        time.sleep(.1)
