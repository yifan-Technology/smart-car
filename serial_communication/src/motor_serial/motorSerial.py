import os
import serial
import threading
import struct
import time
import sys
import yf_node
import rclpy
import yaml
import threading
import queue
from sys import argv, exit
from mainwindow import MainWindow
from PyQt5.QtWidgets import QApplication
from multiprocessing import freeze_support, Event, Queue, Process

PLOT = True

class SerialThread:
    """
    串口通信线程，包含读线程和写线程
    """

    def __init__(self, soll_node, real_node, parser_process=None, port="/dev/ttyUSB0", baudrate=115200, bytesize=8,
                 stopbits=1, timeout=1):
        self.my_serial_port = serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize, stopbits=stopbits,
                                            timeout=timeout)
        self._parser = parser_process

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

        if self.my_serial_port.isOpen():
            self.my_serial_port.close()
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

                    # print("waited {} bytes".format(n))

                myByte = self.tryRead(40)

                if myByte[0] == 97 and myByte[1] == 97 and myByte[2] == 97 and myByte[3] == 97 \
                        and myByte[-1] == 98 and myByte[-2] == 98 and myByte[-3] == 98 and myByte[-4] == 98:
                    fps = 1 / (time.time() - last_time)

                    # comment print can increase fps
                    #print("read fps: ", fps)

                    data = myByte[4:-4]

                    self.read_data = struct.unpack('<ffffffff', data)
                    self._real.publishMsg(self.read_data)
                    if PLOT:
                        self._parser.add_data(["real_data", self.read_data])

                    # comment print can increase fps
                    #print("Real data: ", self.read_data)

                    last_time = time.time()

            except Exception as ex:
                print("all:")
                print(ex)
                with self.read_lock:
                    self.my_serial_port.reset_input_buffer()

    def write(self):
        with self._soll.write_queue.mutex:
           self._soll.write_queue.queue.clear()

        last_time = time.time()

        while self.alive:
            if self._soll.write_queue.empty():
                time.sleep(0.005)
                #print("wait soll speed from ros topic......")
            else:
                self.control_data = self._soll.write_queue.get()
                # self.control_data = self._soll.subMsg
                # time.sleep(.02)
                
                try:
                    start = 99
                    end = 100
                    data = struct.pack("<B4fB", start, self.control_data[0], self.control_data[1], self.control_data[2],
                                       self.control_data[3], end)

                    with self.write_lock:
                        self.my_serial_port.write(data)

                    if PLOT:
                        self._parser.add_data(["soll_data", self.control_data])

                    # comment print can increase fps
                    #print("\ncontrol_data: ", control_data)
                    fps = 1 / (time.time() - last_time)

                    # comment print can increase fps
                    print("write fps: ", fps)
                    last_time = time.time()

                except Exception as ex:
                    print(ex)
                    with self.write_lock:
                        self.my_serial_port.reset_output_buffer()


class Parser(Process):
    def __init__(self, data_queue):
        Process.__init__(self)
        self._exit = Event()
        self._in_queue = Queue()
        self._out_queue = data_queue

    def add_data(self, data):
        self._in_queue.put(data)

    def _store_data(self, data):
        self._out_queue.put(data)

    def get_data(self):
        return self._out_queue.get()

    def run(self):
        while not self._exit.is_set():
            self._consume_queue()

    def _consume_queue(self):
        while not self._in_queue.empty():
            data = self._in_queue.get()
            # print('in Parser: {}'.format(data))
            self._store_data(data)

    def stop(self):
        """
        Signals the process to stop parsing data.
        :return:
        """
        self._exit.set()


def init():
    # 全局化节点名称
    global nodeName
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml", "r") as f:
        config = yaml.load(f)

    # 读取节点名称参数
    nodeName = config["RosTopic"]


def pubSpin(q_node):
    node = q_node.get_nowait()
    while True:
        rclpy.spin_once(node.node, timeout_sec=0.01)
        time.sleep(0.02)


def subSpin(q_node):
    node = q_node.get_nowait()
    rclpy.spin(node.node)


def main():
    os.system("sudo chmod 666 /dev/ttyUSB0")

    init()
    rclpy.init()
    real = yf_node.YF_RealSpeed(nodeName["RealSpeed"], "RealSpeed", "pub")
    soll = yf_node.YF_SollSpeed(nodeName["SollSpeed"], "SollSpeed", "sub")
    rclpy.spin_once(soll.node, timeout_sec=0.1)

    q_subNode = queue.Queue(1)
    q_pubNode = queue.Queue(1)
    q_subNode.put_nowait(soll)
    q_pubNode.put_nowait(real)

    t_sub = threading.Thread(target=subSpin, args=(q_subNode,), daemon=True)
    t_pub = threading.Thread(target=pubSpin, args=(q_pubNode,), daemon=True)
    t_sub.start()
    t_pub.start()

    if PLOT:
        data_queue = Queue()
        parser_process = Parser(data_queue)
        Motor_serial = SerialThread(soll, real, parser_process, baudrate=460800)
        parser_process.start()
    else:
        Motor_serial = SerialThread(soll, real, baudrate=460800)

    t_read = threading.Thread(target=Motor_serial.read, daemon=False)
    t_write = threading.Thread(target=Motor_serial.write, daemon=False)

    Motor_serial.alive = True
    t_read.start()
    t_write.start()

    if PLOT:
        freeze_support()
        app = QApplication(argv)

        main_window = MainWindow(parser_process)
        main_window.show()

    try:
        if PLOT:
            app.exec_()
        else:
            while True:
                pass

    except Exception as error:
        print(error)

        if PLOT:
            for process in [parser_process]:
                if process is not None and process.is_alive():
                    process.stop()
                    process.terminate()

            exit()

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

        if PLOT:
            for process in [parser_process]:
                if process is not None and process.is_alive():
                    process.stop()
                    process.terminate()
            exit()

if __name__ == "__main__":
    main()
