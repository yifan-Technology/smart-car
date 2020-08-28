/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionAbout;
    QAction *actionAboutQt;
    QAction *actionConnect;
    QAction *actionDisconnect;
    QAction *actionConfigure;
    QAction *actionClear;
    QAction *actionQuit;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QLabel *label;
    QLabel *realLeftFrontRS;
    QLabel *label_3;
    QLabel *realLeftFrontRA;
    QLabel *label_5;
    QLineEdit *sollLeftFrontRS_Test;
    QLabel *label_6;
    QPushButton *startTest;
    QPushButton *stopTest;
    QLabel *realRightFrontRS;
    QLineEdit *sollRightFrontRS_Test;
    QLabel *realRightFrontRA;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_7;
    QLabel *sollLeftFrontRS;
    QLabel *sollRightFrontRS;
    QLabel *label_8;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *realRightBackRS;
    QLabel *realLeftBackRS;
    QLabel *label_17;
    QLabel *label_18;
    QLabel *label_19;
    QLabel *label_20;
    QLabel *realRightBackRA;
    QLabel *label_21;
    QLabel *label_22;
    QLineEdit *sollLeftBackRS_Test;
    QLineEdit *sollRightBackRS_Test;
    QLabel *label_23;
    QLabel *sollLeftBackRS;
    QLabel *label_24;
    QLabel *label_25;
    QLabel *sollRightBackRS;
    QLabel *realLeftBackRA;
    QLabel *label_26;
    QMenuBar *menuBar;
    QMenu *menuCalls;
    QMenu *menuTools;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(687, 325);
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionAboutQt = new QAction(MainWindow);
        actionAboutQt->setObjectName(QStringLiteral("actionAboutQt"));
        actionConnect = new QAction(MainWindow);
        actionConnect->setObjectName(QStringLiteral("actionConnect"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/connect.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionConnect->setIcon(icon);
        actionDisconnect = new QAction(MainWindow);
        actionDisconnect->setObjectName(QStringLiteral("actionDisconnect"));
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/images/disconnect.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionDisconnect->setIcon(icon1);
        actionConfigure = new QAction(MainWindow);
        actionConfigure->setObjectName(QStringLiteral("actionConfigure"));
        QIcon icon2;
        icon2.addFile(QStringLiteral(":/images/settings.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionConfigure->setIcon(icon2);
        actionClear = new QAction(MainWindow);
        actionClear->setObjectName(QStringLiteral("actionClear"));
        QIcon icon3;
        icon3.addFile(QStringLiteral(":/images/clear.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionClear->setIcon(icon3);
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        QIcon icon4;
        icon4.addFile(QStringLiteral(":/images/application-exit.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionQuit->setIcon(icon4);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(12, 48, 89, 31));
        QFont font;
        font.setPointSize(12);
        label->setFont(font);
        realLeftFrontRS = new QLabel(groupBox);
        realLeftFrontRS->setObjectName(QStringLiteral("realLeftFrontRS"));
        realLeftFrontRS->setGeometry(QRect(108, 48, 89, 31));
        realLeftFrontRS->setFont(font);
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(12, 78, 91, 31));
        label_3->setFont(font);
        realLeftFrontRA = new QLabel(groupBox);
        realLeftFrontRA->setObjectName(QStringLiteral("realLeftFrontRA"));
        realLeftFrontRA->setGeometry(QRect(108, 78, 87, 31));
        realLeftFrontRA->setFont(font);
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(12, 138, 91, 31));
        label_5->setFont(font);
        sollLeftFrontRS_Test = new QLineEdit(groupBox);
        sollLeftFrontRS_Test->setObjectName(QStringLiteral("sollLeftFrontRS_Test"));
        sollLeftFrontRS_Test->setGeometry(QRect(110, 142, 83, 21));
        sollLeftFrontRS_Test->setFont(font);
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(198, 138, 39, 27));
        label_6->setFont(font);
        startTest = new QPushButton(groupBox);
        startTest->setObjectName(QStringLiteral("startTest"));
        startTest->setGeometry(QRect(480, 190, 75, 27));
        startTest->setFont(font);
        stopTest = new QPushButton(groupBox);
        stopTest->setObjectName(QStringLiteral("stopTest"));
        stopTest->setGeometry(QRect(570, 190, 75, 27));
        stopTest->setFont(font);
        realRightFrontRS = new QLabel(groupBox);
        realRightFrontRS->setObjectName(QStringLiteral("realRightFrontRS"));
        realRightFrontRS->setGeometry(QRect(250, 48, 79, 31));
        realRightFrontRS->setFont(font);
        sollRightFrontRS_Test = new QLineEdit(groupBox);
        sollRightFrontRS_Test->setObjectName(QStringLiteral("sollRightFrontRS_Test"));
        sollRightFrontRS_Test->setGeometry(QRect(250, 140, 83, 21));
        sollRightFrontRS_Test->setFont(font);
        realRightFrontRA = new QLabel(groupBox);
        realRightFrontRA->setObjectName(QStringLiteral("realRightFrontRA"));
        realRightFrontRA->setGeometry(QRect(250, 78, 81, 31));
        realRightFrontRA->setFont(font);
        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(339, 136, 37, 27));
        label_9->setFont(font);
        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(112, 20, 72, 29));
        label_10->setFont(font);
        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(252, 20, 72, 29));
        label_11->setFont(font);
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(12, 106, 91, 31));
        label_7->setFont(font);
        sollLeftFrontRS = new QLabel(groupBox);
        sollLeftFrontRS->setObjectName(QStringLiteral("sollLeftFrontRS"));
        sollLeftFrontRS->setGeometry(QRect(108, 106, 85, 31));
        sollLeftFrontRS->setFont(font);
        sollRightFrontRS = new QLabel(groupBox);
        sollRightFrontRS->setObjectName(QStringLiteral("sollRightFrontRS"));
        sollRightFrontRS->setGeometry(QRect(250, 106, 83, 31));
        sollRightFrontRS->setFont(font);
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(198, 106, 39, 27));
        label_8->setFont(font);
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(198, 76, 39, 27));
        label_12->setFont(font);
        label_13 = new QLabel(groupBox);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(198, 48, 39, 27));
        label_13->setFont(font);
        label_14 = new QLabel(groupBox);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(338, 46, 39, 27));
        label_14->setFont(font);
        label_15 = new QLabel(groupBox);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(338, 104, 39, 27));
        label_15->setFont(font);
        label_16 = new QLabel(groupBox);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(338, 74, 39, 27));
        label_16->setFont(font);
        realRightBackRS = new QLabel(groupBox);
        realRightBackRS->setObjectName(QStringLiteral("realRightBackRS"));
        realRightBackRS->setGeometry(QRect(532, 44, 79, 31));
        realRightBackRS->setFont(font);
        realLeftBackRS = new QLabel(groupBox);
        realLeftBackRS->setObjectName(QStringLiteral("realLeftBackRS"));
        realLeftBackRS->setGeometry(QRect(390, 44, 89, 31));
        realLeftBackRS->setFont(font);
        label_17 = new QLabel(groupBox);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(620, 70, 39, 27));
        label_17->setFont(font);
        label_18 = new QLabel(groupBox);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(534, 20, 72, 29));
        label_18->setFont(font);
        label_19 = new QLabel(groupBox);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(480, 102, 39, 27));
        label_19->setFont(font);
        label_20 = new QLabel(groupBox);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(620, 42, 39, 27));
        label_20->setFont(font);
        realRightBackRA = new QLabel(groupBox);
        realRightBackRA->setObjectName(QStringLiteral("realRightBackRA"));
        realRightBackRA->setGeometry(QRect(532, 74, 81, 31));
        realRightBackRA->setFont(font);
        label_21 = new QLabel(groupBox);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setGeometry(QRect(480, 134, 39, 27));
        label_21->setFont(font);
        label_22 = new QLabel(groupBox);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setGeometry(QRect(621, 132, 37, 27));
        label_22->setFont(font);
        sollLeftBackRS_Test = new QLineEdit(groupBox);
        sollLeftBackRS_Test->setObjectName(QStringLiteral("sollLeftBackRS_Test"));
        sollLeftBackRS_Test->setGeometry(QRect(392, 138, 83, 21));
        sollLeftBackRS_Test->setFont(font);
        sollRightBackRS_Test = new QLineEdit(groupBox);
        sollRightBackRS_Test->setObjectName(QStringLiteral("sollRightBackRS_Test"));
        sollRightBackRS_Test->setGeometry(QRect(532, 136, 83, 21));
        sollRightBackRS_Test->setFont(font);
        label_23 = new QLabel(groupBox);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setGeometry(QRect(480, 72, 39, 27));
        label_23->setFont(font);
        sollLeftBackRS = new QLabel(groupBox);
        sollLeftBackRS->setObjectName(QStringLiteral("sollLeftBackRS"));
        sollLeftBackRS->setGeometry(QRect(390, 102, 85, 31));
        sollLeftBackRS->setFont(font);
        label_24 = new QLabel(groupBox);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setGeometry(QRect(480, 44, 39, 27));
        label_24->setFont(font);
        label_25 = new QLabel(groupBox);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setGeometry(QRect(394, 20, 72, 29));
        label_25->setFont(font);
        sollRightBackRS = new QLabel(groupBox);
        sollRightBackRS->setObjectName(QStringLiteral("sollRightBackRS"));
        sollRightBackRS->setGeometry(QRect(532, 102, 83, 31));
        sollRightBackRS->setFont(font);
        realLeftBackRA = new QLabel(groupBox);
        realLeftBackRA->setObjectName(QStringLiteral("realLeftBackRA"));
        realLeftBackRA->setGeometry(QRect(390, 74, 87, 31));
        realLeftBackRA->setFont(font);
        label_26 = new QLabel(groupBox);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setGeometry(QRect(620, 100, 39, 27));
        label_26->setFont(font);

        verticalLayout->addWidget(groupBox);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 687, 22));
        menuCalls = new QMenu(menuBar);
        menuCalls->setObjectName(QStringLiteral("menuCalls"));
        menuTools = new QMenu(menuBar);
        menuTools->setObjectName(QStringLiteral("menuTools"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuCalls->menuAction());
        menuBar->addAction(menuTools->menuAction());
        menuCalls->addAction(actionConnect);
        menuCalls->addAction(actionDisconnect);
        menuCalls->addSeparator();
        menuCalls->addAction(actionQuit);
        menuTools->addAction(actionConfigure);
        mainToolBar->addAction(actionConnect);
        mainToolBar->addAction(actionDisconnect);
        mainToolBar->addAction(actionConfigure);
        mainToolBar->addSeparator();
        mainToolBar->addAction(actionClear);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Simple Terminal", Q_NULLPTR));
        actionAbout->setText(QApplication::translate("MainWindow", "&About", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionAbout->setToolTip(QApplication::translate("MainWindow", "About program", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_SHORTCUT
        actionAbout->setShortcut(QApplication::translate("MainWindow", "Alt+A", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        actionAboutQt->setText(QApplication::translate("MainWindow", "About Qt", Q_NULLPTR));
        actionConnect->setText(QApplication::translate("MainWindow", "C&onnect", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionConnect->setToolTip(QApplication::translate("MainWindow", "Connect to serial port", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_SHORTCUT
        actionConnect->setShortcut(QApplication::translate("MainWindow", "Ctrl+O", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        actionDisconnect->setText(QApplication::translate("MainWindow", "&Disconnect", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionDisconnect->setToolTip(QApplication::translate("MainWindow", "Disconnect from serial port", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_SHORTCUT
        actionDisconnect->setShortcut(QApplication::translate("MainWindow", "Ctrl+D", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        actionConfigure->setText(QApplication::translate("MainWindow", "&Configure", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionConfigure->setToolTip(QApplication::translate("MainWindow", "Configure serial port", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_SHORTCUT
        actionConfigure->setShortcut(QApplication::translate("MainWindow", "Alt+C", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        actionClear->setText(QApplication::translate("MainWindow", "C&lear", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionClear->setToolTip(QApplication::translate("MainWindow", "Clear data", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_SHORTCUT
        actionClear->setShortcut(QApplication::translate("MainWindow", "Alt+L", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        actionQuit->setText(QApplication::translate("MainWindow", "&Quit", Q_NULLPTR));
#ifndef QT_NO_SHORTCUT
        actionQuit->setShortcut(QApplication::translate("MainWindow", "Ctrl+Q", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        groupBox->setTitle(QApplication::translate("MainWindow", "CarControl", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "\345\256\236\351\231\205\350\275\254\351\200\237\357\274\232", Q_NULLPTR));
        realLeftFrontRS->setText(QApplication::translate("MainWindow", "0.0     ", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "\345\256\236\351\231\205\350\275\254\350\247\222\357\274\232", Q_NULLPTR));
        realLeftFrontRA->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "\347\220\206\346\203\263\350\275\254\351\200\237\357\274\232", Q_NULLPTR));
        sollLeftFrontRS_Test->setText(QApplication::translate("MainWindow", "25", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        startTest->setText(QApplication::translate("MainWindow", "Start", Q_NULLPTR));
        stopTest->setText(QApplication::translate("MainWindow", "Stop", Q_NULLPTR));
        realRightFrontRS->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        sollRightFrontRS_Test->setText(QApplication::translate("MainWindow", "25", Q_NULLPTR));
        realRightFrontRA->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindow", "\345\267\246\345\211\215", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindow", "\345\217\263\345\211\215", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "\347\220\206\346\203\263\350\275\254\351\200\237\357\274\232", Q_NULLPTR));
        sollLeftFrontRS->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        sollRightFrontRS->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        label_12->setText(QApplication::translate("MainWindow", "rad", Q_NULLPTR));
        label_13->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        label_14->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        label_15->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        label_16->setText(QApplication::translate("MainWindow", "rad", Q_NULLPTR));
        realRightBackRS->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        realLeftBackRS->setText(QApplication::translate("MainWindow", "0.0     ", Q_NULLPTR));
        label_17->setText(QApplication::translate("MainWindow", "rad", Q_NULLPTR));
        label_18->setText(QApplication::translate("MainWindow", "\345\217\263\345\220\216", Q_NULLPTR));
        label_19->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        label_20->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        realRightBackRA->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        label_21->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        label_22->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        sollLeftBackRS_Test->setText(QApplication::translate("MainWindow", "25", Q_NULLPTR));
        sollRightBackRS_Test->setText(QApplication::translate("MainWindow", "25", Q_NULLPTR));
        label_23->setText(QApplication::translate("MainWindow", "rad", Q_NULLPTR));
        sollLeftBackRS->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        label_24->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        label_25->setText(QApplication::translate("MainWindow", "\345\267\246\345\220\216", Q_NULLPTR));
        sollRightBackRS->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        realLeftBackRA->setText(QApplication::translate("MainWindow", "0.0      ", Q_NULLPTR));
        label_26->setText(QApplication::translate("MainWindow", "rpm", Q_NULLPTR));
        menuCalls->setTitle(QApplication::translate("MainWindow", "Calls", Q_NULLPTR));
        menuTools->setTitle(QApplication::translate("MainWindow", "Tools", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
