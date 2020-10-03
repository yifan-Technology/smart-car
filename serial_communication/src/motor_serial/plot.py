import pyqtgraph as pg
from time import time, sleep
from PyQt5.QtCore import Qt, QTimer
from ringBuffer import RingBuffer
from PyQt5.QtWidgets import QWidget, QDesktopWidget


class Plot(QWidget):
    def __init__(self, all_data, update_plot_timeout=500):
        super(Plot, self).__init__()
        self._all_data = all_data
        self.update_plot_timeout = update_plot_timeout

        self.configure_plot()
        self.configure_timer()
        self.configure_ringbuffer()

        self.close = False
        self.added_plot_line = []

        # self.setWindowFlags(Qt.WindowStaysOnTopHint)  # 窗体总在最前端

    def center(self):
        # Position the graphics window to the middle of the screen

        # Get the window frame
        qr = self.frameGeometry()
        # Get the screen center point of the display
        cp = QDesktopWidget().availableGeometry().center()
        # Move the center point of the virtual frame to the center of the screen
        qr.moveCenter(cp)
        # Move the real window to the center of the screen
        self.move(qr.topLeft())

    def configure_plot(self):
        self.win = pg.GraphicsLayoutWidget(self)
        self.win.resize(1980, 1080)
        self.win.setWindowTitle('pyqtgraph: Plotting')
        self.win.setAntialiasing(True)

        self.plotArea = {"leftfront": None, "rightfront": None, "leftback": None, "rightback": None}
        self.plot_lines = {"real_leftfront_rs": None, "real_leftfront_ra": None, "soll_leftfront_rs": None,
                         "real_rightfront_rs": None, "real_rightfront_ra": None, "soll_rightfront_rs": None,
                         "real_leftback_rs": None, "real_leftback_ra": None, "back_leftback_rs": None,
                         "real_rightback_rs": None, "real_rightback_ra": None, "soll_rightback_rs": None}
        self.plot_lineNum = 0
        self.plot_timestamp = {"leftfront": None, "rightfront": None, "leftback": None, "rightback": None}
        self.plot_timestamp_begin = {"leftfront": None, "rightfront": None, "leftback": None, "rightback": None}

        self.plot_colors = ['#0072bd', '#d95319', '#edb120', '#7e2f8e', '#77ac30', '#4dbeee',
                            '#483D8B', '#4682B4', '#00FFFF', '#00FF7F', '#7FFF00', '#a2142f']

    def configure_timer(self):
        self.timer_plot_leftfrontArea = QTimer()
        self.timer_plot_leftfrontArea.setInterval(self.update_plot_timeout)
        self.timer_plot_leftfrontArea.timeout.connect(lambda: self.update_plot("leftfront"))

        self.timer_plot_rightfrontArea = QTimer()
        self.timer_plot_rightfrontArea.setInterval(self.update_plot_timeout)
        self.timer_plot_rightfrontArea.timeout.connect(lambda: self.update_plot("rightfront"))

        self.timer_plot_leftbackArea = QTimer()
        self.timer_plot_leftbackArea.setInterval(self.update_plot_timeout)
        self.timer_plot_leftbackArea.timeout.connect(lambda: self.update_plot("leftback"))

        self.timer_plot_rightbackArea = QTimer()
        self.timer_plot_rightbackArea.setInterval(self.update_plot_timeout)
        self.timer_plot_rightbackArea.timeout.connect(lambda: self.update_plot("rightback"))

    def configure_ringbuffer(self):
        self.time_buffer = {"leftfront": None, "rightfront": None, "leftback": None, "rightback": None}
        self.data_buffer = {"leftfront": {}, "rightfront": {}, "leftback": {}, "rightback": {}}

    def add_plot_area(self, data_type):
        self.plotArea[data_type] = self.win.addPlot(title=data_type+" wheel")
        self.plotArea[data_type].setLabel('bottom', 'Time', 's')
        self.plotArea[data_type].setLabel('left', 'Speed', 'rpm')
        self.plotArea[data_type].addLegend()
        self.plot_timestamp_begin[data_type] = time()
        if data_type == "leftfront":
            self.timer_plot_leftfrontArea.start()
        elif data_type == "rightfront":
            self.timer_plot_rightfrontArea.start()
        elif data_type == "leftback":
            self.timer_plot_leftbackArea.start()
        elif data_type == "rightback":
            self.timer_plot_rightbackArea.start()

    def plot_line_added(self, lcdNum_id):
        if lcdNum_id in self.added_plot_line:
            return True
        self.added_plot_line.append(lcdNum_id)

    def add_plot_line(self, update_plot_samples, lcdNum_id, data_type):
        if self.time_buffer[data_type] == None:
            self.time_buffer[data_type] = RingBuffer(update_plot_samples)
        self.data_buffer[data_type][lcdNum_id] = RingBuffer(update_plot_samples)

        line_name = "Data of {}".format(lcdNum_id)
        self.plot_lines[lcdNum_id] = self.plotArea[data_type].plot(pen=self.plot_colors[self.plot_lineNum], name=line_name)
        self.plot_lineNum += 1

    def update_plot_helper(self, data_type):
        self.plot_timestamp[data_type] = time() - self.plot_timestamp_begin[data_type]
        self.time_buffer[data_type].append(self.plot_timestamp[data_type])

        for lcdNum_id in self.data_buffer[data_type]:
            self.data_buffer[data_type][lcdNum_id].append(self._all_data[lcdNum_id])
            X = self.time_buffer[data_type].get_all()
            Y = self.data_buffer[data_type][lcdNum_id].get_all()
            # print("X: {}".format(X))
            # print("Y: {}".format(Y))
            self.plot_lines[lcdNum_id].setData(x=X, y=Y)

    def update_plot(self, data_type):
        self.update_plot_helper(data_type)

    def closeEvent(self, event):
        # the function that is triggered when When the user presses the red cross button
        for timer in [self.timer_plot_leftfrontArea, self.timer_plot_rightfrontArea, self.timer_plot_leftbackArea,
                      self.timer_plot_leftbackArea]:
            if timer.isActive():
                timer.stop()

        sleep(.01)

        for data_type in self.plotArea:
            if self.plotArea[data_type] != None:
                self.plotArea[data_type].clear()

        # self.configure_plot()
        self.win = None
        # self.configure_ringbuffer()

        self.close = True

        event.accept()



