import math
from PyQt5.QtCore import QRegExp, QRect


class AutoResize:

    def __init__(self, main_window, widgets_list):
        """
        Description:
            This is a module for realizing the automatic resize of gui. This means that the window does not remain fixed
            and can be changed. The size of gui also adapts to the screen resolution. The font size changes with the
            size of the gui.
        Usage:
            Only four lines of code are needed to achieve the above functions. The 2, 3, 4 lines should be unchanged.
            1. Initialize a list that contains all Type whose widget object should resize automatically. In the usage
               example there are six types in the list. This means all widgets in GUI belong to one of these six types.
            2. Initialize the class AutoResize
            3, 4. Rewrite the resizeEvent function

        Usage example:
            class MainWindow(QWidget):
                def __init__(self):
                    ...
            1.      widgets_list = [self.findChildren(QPushButton), self.findChildren(QLabel),
                        self.findChildren(QComboBox), self.findChildren(QLineEdit),
                        self.findChildren(QGroupBox), self.findChildren(QRadioButton)]
            2.      self.auto_resize_handler = auto_resize.AutoResize(self, widgets_list)
                    ...

            3.  def resizeEvent(self, event: QtGui.QResizeEvent):
            4.      self.auto_resize_handler.do_auto_resize()

        :param main_window: GUI main window
        :param widgets_list: a list of all type of widgets that should be automatically resized
        """

        self._main_window = main_window
        self._auto_resize = True
        self._main_window_base_width = self._main_window.geometry().width()
        self._main_window_base_height = self._main_window.geometry().height()
        self._hor_ratio = 1.0
        self._ver_ratio = 1.0
        self._font_ratio = 1.0
        self._resize_map = {}
        self._widgets_list = widgets_list
        self._push_all_resize_widgets()
        self.auto_resize_for_resolution()

    def auto_resize_for_resolution(self):
        # Auto resize for resolution
        self._main_window.setMinimumSize(self._main_window.geometry().width(), self._main_window.geometry().height())
        self._main_window.setGeometry(QRect(self._main_window.x(), self._main_window.y(),
                                            self._main_window.geometry().width() * 2,
                                            self._main_window.geometry().height() * 2))

    def _push_all_resize_widgets(self):
        for widget_list in self._widgets_list:
            for widget in widget_list:
                widget_geometry = widget.geometry()
                resize_data = {"geometry": widget_geometry, "font": widget.font()}
                self._resize_map[widget] = resize_data

    def push_resize_widgets(self, widget_list):
        for widget in widget_list:
            widget_geometry = widget.geometry()
            resize_data = {"geometry": widget_geometry, "font": widget.font()}
            self._resize_map[widget] = resize_data

    def push_back_resize_widgets(self, widget_list):
        for widget in widget_list:
            self._resize_map.pop(widget)

    def update_resize_map(self):
        for widget in self._resize_map:
            widget_geometry = widget.geometry()
            widget_geometry = QRect(widget_geometry.x() / self._hor_ratio, widget_geometry.y() / self._ver_ratio,
                                    widget_geometry.width() / self._hor_ratio,
                                    widget_geometry.height() / self._ver_ratio)
            widget_font = widget.font()
            widget_font.setPointSize(math.ceil(widget_font.pointSize() / self._font_ratio))
            resize_data = {"geometry": widget_geometry, "font": widget_font}
            self._resize_map[widget] = resize_data

    def calculate_resize_ratio(self):
        self._hor_ratio = self._main_window.width() / self._main_window_base_width
        self._ver_ratio = self._main_window.height() / self._main_window_base_height
        self._font_ratio = self._hor_ratio if self._hor_ratio < self._ver_ratio else self._ver_ratio

    def _font_auto_resize(self, widget, font_size):
        if font_size <= 0:
            return
        else:
            font_size *= self._font_ratio

        has_text_style = False
        font_text_reg = "font:\\s+[0-9]+pt"
        font_size_reg = "[0-9]+"
        reg = QRegExp(font_text_reg)
        size = QRegExp(font_size_reg)
        if reg.indexIn(widget.styleSheet()) != -1:
            font_text = reg.capturedTexts().at(0)
            if size.indexIn(font_text) != -1:
                has_text_style = True

        if has_text_style:
            font_format = "font: {}pt".format(font_size)
            style_text = widget.styleSheet()
            style_text.replace(reg, font_format)
            widget.setStyleSheet(style_text)
        else:
            changed_font = widget.font()
            changed_font.setPointSize(font_size)
            widget.setFont(changed_font)

    def do_auto_resize(self):
        self.calculate_resize_ratio()
        if self._auto_resize:
            for widget in self._resize_map:
                # change font size
                origin_font_size = self._resize_map[widget]["font"].pointSize()
                self._font_auto_resize(widget, origin_font_size)
                # change geometry of widget
                origin_geometry = self._resize_map[widget]["geometry"]
                widget.setGeometry(QRect(origin_geometry.x()*self._hor_ratio, origin_geometry.y()*self._ver_ratio,
                                         origin_geometry.width()*self._hor_ratio,
                                         origin_geometry.height()*self._ver_ratio))