# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'datashow.ui'
#
# Created by: PyQt5 UI code generator 5.15.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(714, 219)
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(20, 15, 669, 181))
        self.groupBox.setObjectName("groupBox")
        self.label = QtWidgets.QLabel(self.groupBox)
        self.label.setGeometry(QtCore.QRect(12, 50, 89, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.label_3 = QtWidgets.QLabel(self.groupBox)
        self.label_3.setGeometry(QtCore.QRect(12, 95, 91, 31))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.label_10 = QtWidgets.QLabel(self.groupBox)
        self.label_10.setGeometry(QtCore.QRect(112, 20, 72, 29))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.groupBox)
        self.label_11.setGeometry(QtCore.QRect(252, 20, 72, 29))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_11.setFont(font)
        self.label_11.setObjectName("label_11")
        self.label_7 = QtWidgets.QLabel(self.groupBox)
        self.label_7.setGeometry(QtCore.QRect(12, 145, 91, 26))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.groupBox)
        self.label_8.setGeometry(QtCore.QRect(195, 140, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.label_12 = QtWidgets.QLabel(self.groupBox)
        self.label_12.setGeometry(QtCore.QRect(195, 95, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_12.setFont(font)
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(self.groupBox)
        self.label_13.setGeometry(QtCore.QRect(195, 50, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_13.setFont(font)
        self.label_13.setObjectName("label_13")
        self.label_18 = QtWidgets.QLabel(self.groupBox)
        self.label_18.setGeometry(QtCore.QRect(534, 20, 72, 29))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_18.setFont(font)
        self.label_18.setObjectName("label_18")
        self.label_25 = QtWidgets.QLabel(self.groupBox)
        self.label_25.setGeometry(QtCore.QRect(394, 20, 72, 29))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_25.setFont(font)
        self.label_25.setObjectName("label_25")
        self.real_leftfront_rs = QtWidgets.QLCDNumber(self.groupBox)
        self.real_leftfront_rs.setGeometry(QtCore.QRect(110, 50, 71, 26))
        self.real_leftfront_rs.setObjectName("real_leftfront_rs")
        self.real_leftfront_ra = QtWidgets.QLCDNumber(self.groupBox)
        self.real_leftfront_ra.setGeometry(QtCore.QRect(110, 95, 71, 26))
        self.real_leftfront_ra.setObjectName("real_leftfront_ra")
        self.soll_leftfront_rs = QtWidgets.QLCDNumber(self.groupBox)
        self.soll_leftfront_rs.setGeometry(QtCore.QRect(110, 140, 71, 26))
        self.soll_leftfront_rs.setObjectName("soll_leftfront_rs")
        self.label_14 = QtWidgets.QLabel(self.groupBox)
        self.label_14.setGeometry(QtCore.QRect(335, 50, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_14.setFont(font)
        self.label_14.setObjectName("label_14")
        self.label_15 = QtWidgets.QLabel(self.groupBox)
        self.label_15.setGeometry(QtCore.QRect(335, 95, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_15.setFont(font)
        self.label_15.setObjectName("label_15")
        self.soll_rightfront_rs = QtWidgets.QLCDNumber(self.groupBox)
        self.soll_rightfront_rs.setGeometry(QtCore.QRect(250, 140, 71, 26))
        self.soll_rightfront_rs.setObjectName("soll_rightfront_rs")
        self.real_rightfront_ra = QtWidgets.QLCDNumber(self.groupBox)
        self.real_rightfront_ra.setGeometry(QtCore.QRect(250, 95, 71, 26))
        self.real_rightfront_ra.setObjectName("real_rightfront_ra")
        self.label_9 = QtWidgets.QLabel(self.groupBox)
        self.label_9.setGeometry(QtCore.QRect(335, 140, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.real_rightfront_rs = QtWidgets.QLCDNumber(self.groupBox)
        self.real_rightfront_rs.setGeometry(QtCore.QRect(250, 50, 71, 26))
        self.real_rightfront_rs.setObjectName("real_rightfront_rs")
        self.label_16 = QtWidgets.QLabel(self.groupBox)
        self.label_16.setGeometry(QtCore.QRect(475, 50, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_16.setFont(font)
        self.label_16.setObjectName("label_16")
        self.label_17 = QtWidgets.QLabel(self.groupBox)
        self.label_17.setGeometry(QtCore.QRect(475, 95, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_17.setFont(font)
        self.label_17.setObjectName("label_17")
        self.soll_leftback_rs = QtWidgets.QLCDNumber(self.groupBox)
        self.soll_leftback_rs.setGeometry(QtCore.QRect(390, 140, 71, 26))
        self.soll_leftback_rs.setObjectName("soll_leftback_rs")
        self.real_leftback_ra = QtWidgets.QLCDNumber(self.groupBox)
        self.real_leftback_ra.setGeometry(QtCore.QRect(390, 95, 71, 26))
        self.real_leftback_ra.setObjectName("real_leftback_ra")
        self.label_19 = QtWidgets.QLabel(self.groupBox)
        self.label_19.setGeometry(QtCore.QRect(475, 140, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_19.setFont(font)
        self.label_19.setObjectName("label_19")
        self.real_leftback_rs = QtWidgets.QLCDNumber(self.groupBox)
        self.real_leftback_rs.setGeometry(QtCore.QRect(390, 50, 71, 26))
        self.real_leftback_rs.setObjectName("real_leftback_rs")
        self.label_20 = QtWidgets.QLabel(self.groupBox)
        self.label_20.setGeometry(QtCore.QRect(615, 50, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_20.setFont(font)
        self.label_20.setObjectName("label_20")
        self.label_21 = QtWidgets.QLabel(self.groupBox)
        self.label_21.setGeometry(QtCore.QRect(615, 95, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_21.setFont(font)
        self.label_21.setObjectName("label_21")
        self.soll_rightback_rs = QtWidgets.QLCDNumber(self.groupBox)
        self.soll_rightback_rs.setGeometry(QtCore.QRect(530, 140, 71, 26))
        self.soll_rightback_rs.setObjectName("soll_rightback_rs")
        self.real_rightback_ra = QtWidgets.QLCDNumber(self.groupBox)
        self.real_rightback_ra.setGeometry(QtCore.QRect(530, 95, 71, 26))
        self.real_rightback_ra.setObjectName("real_rightback_ra")
        self.label_22 = QtWidgets.QLabel(self.groupBox)
        self.label_22.setGeometry(QtCore.QRect(615, 140, 39, 27))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_22.setFont(font)
        self.label_22.setObjectName("label_22")
        self.real_rightback_rs = QtWidgets.QLCDNumber(self.groupBox)
        self.real_rightback_rs.setGeometry(QtCore.QRect(530, 50, 71, 26))
        self.real_rightback_rs.setObjectName("real_rightback_rs")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.groupBox.setTitle(_translate("Form", "CarControl"))
        self.label.setText(_translate("Form", "实际转速："))
        self.label_3.setText(_translate("Form", "实际转角："))
        self.label_10.setText(_translate("Form", "左前"))
        self.label_11.setText(_translate("Form", "右前"))
        self.label_7.setText(_translate("Form", "理想转速："))
        self.label_8.setText(_translate("Form", "rpm"))
        self.label_12.setText(_translate("Form", "rad"))
        self.label_13.setText(_translate("Form", "rpm"))
        self.label_18.setText(_translate("Form", "右后"))
        self.label_25.setText(_translate("Form", "左后"))
        self.label_14.setText(_translate("Form", "rpm"))
        self.label_15.setText(_translate("Form", "rad"))
        self.label_9.setText(_translate("Form", "rpm"))
        self.label_16.setText(_translate("Form", "rpm"))
        self.label_17.setText(_translate("Form", "rad"))
        self.label_19.setText(_translate("Form", "rpm"))
        self.label_20.setText(_translate("Form", "rpm"))
        self.label_21.setText(_translate("Form", "rad"))
        self.label_22.setText(_translate("Form", "rpm"))