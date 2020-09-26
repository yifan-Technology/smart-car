from plot import Plot
from sys import argv, exit
from threading import Thread, RLock
from datashow import Ui_Form
from auto_resize import AutoResize
from PyQt5.QtGui import QResizeEvent
from PyQt5.QtCore import pyqtSignal, QObject, QEvent, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QLCDNumber, QLabel, QGroupBox


class MainWindow(QMainWindow, Ui_Form):
    # directly inherit the Ui_MainWindow class

    def __init__(self, parser_process):
        # Initialize the class Ui_MainWindow,
        # call the object method setupUi to generate a graphical interface for displaying sensor data.
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self._parser = parser_process

        widgets_list = [self.findChildren(QLCDNumber), self.findChildren(QLabel),
                        self.findChildren(QGroupBox)]

        self.auto_resize_handler = AutoResize(self, widgets_list)

        # configures
        self.initLcdNumber()
        self.configure_timers()

        self.all_data = {"real_leftfront_rs": 0.0, "real_leftfront_ra": 0.0, "soll_leftfront_rs": 0.0,
                         "real_rightfront_rs": 0.0, "real_rightfront_ra": 0.0, "soll_rightfront_rs": 0.0,
                         "real_leftback_rs": 0.0, "real_leftback_ra": 0.0, "soll_leftback_rs": 0.0,
                         "real_rightback_rs": 0.0, "real_rightback_ra": 0.0, "soll_rightback_rs": 0.0}

        self.update_plot_timeout = 20
        self.update_plot_samples = 500  # 0.02s = 1 samples, 10s = 500 samples
        self.plt = None

        self.write_lock = RLock()
        update_shown_data_thread = Thread(target=self.update_shown_data, args=(), daemon=True)
        update_shown_data_thread.start()

        # update LCD_NUmber start
        for i in range(len(self.timer)):
            self.timer[i].timer.start()

    def closeEvent(self, *args, **kwargs):
        self.close()

    def resizeEvent(self, event: QResizeEvent):
        self.auto_resize_handler.do_auto_resize()

    def update_shown_data(self):
        while True:
            data = self._parser.get_data()
            with self.write_lock:
                if data[0] == "soll_data":
                    self.all_data["soll_leftfront_rs"] = data[1][0]
                    self.all_data["soll_rightfront_rs"] = data[1][1]
                    self.all_data["soll_leftback_rs"] = data[1][2]
                    self.all_data["soll_rightback_rs"] = data[1][3]

                elif data[0] == "real_data":
                    self.all_data["real_leftfront_rs"] = data[1][0]
                    self.all_data["real_leftfront_ra"] = data[1][1]
                    self.all_data["real_rightfront_rs"] = data[1][2]
                    self.all_data["real_rightfront_ra"] = data[1][3]
                    self.all_data["real_leftback_rs"] = data[1][4]
                    self.all_data["real_leftback_ra"] = data[1][5]
                    self.all_data["real_rightback_rs"] = data[1][6]
                    self.all_data["real_rightback_ra"] = data[1][7]

    def get_data(self, lcdNum_id):
        return self.all_data[lcdNum_id]

    def configure_timers(self):
        self.timer = []
        for lcdNum_id in ["real_leftfront_rs", "real_leftfront_ra", "soll_leftfront_rs",
                     "real_rightfront_rs", "real_rightfront_ra", "soll_rightfront_rs",
                     "real_leftback_rs", "real_leftback_ra", "soll_leftback_rs",
                     "real_rightback_rs", "real_rightback_ra", "soll_rightback_rs"]:
            self.timer.append(TimerUpdateLCDNum(lcdNum_id, self))

    def initLcdNumber(self):
        for lcdNum_id in ["real_leftfront_rs", "real_leftfront_ra", "soll_leftfront_rs",
                     "real_rightfront_rs", "real_rightfront_ra", "soll_rightfront_rs",
                     "real_leftback_rs", "real_leftback_ra", "soll_leftback_rs",
                     "real_rightback_rs", "real_rightback_ra", "soll_rightback_rs"]:     # A total of 13 * 8 = 104 LcdNumber
            lcdNumber_name = eval("self." + str(lcdNum_id))   # Dynamically named variable
            lcdNumber_name.setDigitCount(10)  # Set the maximum number of characters displayed by the lcdNumber
            lcdNumber_name.setMode(QLCDNumber.Dec)  # Set the display mode of the lcdNumber to decimal
            lcdNumber_name.setStyleSheet("background: gray;")  # The initial background color of all lcdNumber is gray

            # Pass lcdNumber and i as arguments to the method clickable to generate a custom signal corresponding to it.
            # Each signal is bound to a slot function.
            clickable(lcdNumber_name, lcdNum_id).connect(lambda: showData(self))

def clickable(widget, i):
    # Implement a customized signal that will be emitted when the mouse is pressed and released,
    # used to bind the corresponding slot function to implement a custom mouse-down callback function.
    class Filter(QObject):
        # Variable id marks which lcdNumber was pressed.
        lcdNum_id = i
        # Initialize a pyqt signal for customization.
        clicked = pyqtSignal()

        def eventFilter(self, obj, event):
            # Overwrite the method "eventFilter" of the inherited official class QObject of pyqt.
            if obj == widget:
                if event.type() == QEvent.MouseButtonRelease:
                    # Determine whether the mouse is pressed in the range of the pyqt component
                    # corresponding to the parameter passed by the function, i.e. lcdNumber.
                    if obj.rect().contains(event.pos()):
                        # The custom pyqt signal is emitted, and the slot function bound to it is automatically triggered.
                        self.clicked.emit()
                        return True
            return False

    filter = Filter(widget)
    widget.installEventFilter(filter)
    # Return a custom pyqt signal to bind a slot function。
    return filter.clicked


def showData(main_window):
    update_plot_samples = main_window.update_plot_samples
    lcdNum_id = main_window.sender().lcdNum_id

    if main_window.plt != None and main_window.plt.close == True:
        main_window.plt = None

    if main_window.plt == None:
        main_window.plt = Plot(main_window.all_data, main_window.update_plot_timeout)
        main_window.plt.show()
        main_window.plt.center()

    data_type = lcdNum_id[5:-3]

    if main_window.plt.plotArea[data_type] == None:
        main_window.plt.add_plot_area(data_type)

    if not main_window.plt.plot_line_added(lcdNum_id):
        main_window.plt.add_plot_line(update_plot_samples, lcdNum_id, data_type)


def showText(main_window):
    lcdNum_id = main_window.sender().lcdNum_id
    print(lcdNum_id)

class TimerUpdateLCDNum:
    def __init__(self, ID, main_window):
        self.ID = ID
        self.main_window = main_window

        self.timer = QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(lambda: self.update_LCDNum(self.main_window))

    def update_LCDNum(self, main_window):
        lcdNumber_name = eval("main_window." + str(self.ID))
        data = main_window.get_data(self.ID)

        lcdNumber_name.setStyleSheet("background: green;")  # Change the color of a single lcdNumber to green.
        lcdNumber_name.display(data)


if __name__ == "__main__":
    app = QApplication(argv)
    win = MainWindow()
    win.show()
    exit(app.exec_())
