#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "settingsdialog.h"
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QTimer>
#include <string.h>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_status(new QLabel),
    //m_console(new Console),
    m_settings(new SettingsDialog),

    m_serial(new QSerialPort(this))
{
    m_ui->setupUi(this);
    //m_console->setEnabled(false);
    //setCentralWidget(m_console);

    m_ui->actionConnect->setEnabled(true);
    m_ui->actionDisconnect->setEnabled(false);
    m_ui->actionQuit->setEnabled(true);
    m_ui->actionConfigure->setEnabled(true);

    m_ui->statusBar->addWidget(m_status);

    initActionsConnections();
    initConnections();
    connect(m_serial, &QSerialPort::errorOccurred, this, &MainWindow::handleError);
    connect(m_serial, &QSerialPort::readyRead, this, &MainWindow::readData);

    //connect(m_console, &Console::getData, this, &MainWindow::writeData);
    //set Timer
    QTimer *timer = new QTimer;
    connect(timer, SIGNAL(timeout()), SLOT(send()));
    timer->start(1000);

    //set data frame SOF and EOF
    _controlFrame.SOF = JetsonCommSOF;
    _controlFrame._EOF = JetsonCommEOF;
    _feedBackFrame.SOF = STM32CommSOF;
    _feedBackFrame._EOF = STM32CommEOF;

    //set up two buttons for test
    m_ui->startTest->setEnabled(true);
    m_ui->stopTest->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete m_settings;
    delete m_ui;
}

void MainWindow::openSerialPort()
{
    const SettingsDialog::Settings p = m_settings->settings();
    m_serial->setPortName(p.name);
    m_serial->setBaudRate(p.baudRate);
    m_serial->setDataBits(p.dataBits);
    m_serial->setParity(p.parity);
    m_serial->setStopBits(p.stopBits);
    m_serial->setFlowControl(p.flowControl);
    if (m_serial->open(QIODevice::ReadWrite)) {
        //m_console->setEnabled(false);
        //m_console->setLocalEchoEnabled(p.localEchoEnabled);
        m_ui->actionConnect->setEnabled(false);
        m_ui->actionDisconnect->setEnabled(true);
        m_ui->actionConfigure->setEnabled(false);
        showStatusMessage(tr("Connected to %1 : %2, %3, %4, %5, %6")
                          .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                          .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
    } else {
        QMessageBox::critical(this, tr("Error"), m_serial->errorString());

        showStatusMessage(tr("Open error"));
    }
}

void MainWindow::closeSerialPort()
{
    if (m_serial->isOpen())
        m_serial->close();
    //m_console->setEnabled(false);
    m_ui->actionConnect->setEnabled(true);
    m_ui->actionDisconnect->setEnabled(false);
    m_ui->actionConfigure->setEnabled(true);
    showStatusMessage(tr("Disconnected"));
}

//void MainWindow::about()
//{
//    QMessageBox::about(this, tr("About Simple Terminal"),
//                       tr("The <b>Simple Terminal</b> example demonstrates how to "
//                          "use the Qt Serial Port module in modern GUI applications "
//                          "using Qt, with a menu bar, toolbars, and a status bar."));
//}

MainWindow::ControlFrame MainWindow::pack(ControlData& ctrl)
{
    return ControlFrame
    {
        JetsonCommSOF,
        ctrl.soll_left_rs,
        ctrl.soll_right_rs,
        JetsonCommEOF
    };
}

ControlData MainWindow::calculateRS()
{
    float soll_left_rs;
    float soll_right_rs;
    float rand_error = rand()/(RAND_MAX+1.0);
    rand_error = (int)(1000*rand_error) / 1000.0;
    if (!Test) {
        soll_left_rs = 22.323 + rand_error;
        soll_right_rs = 23.457 + rand_error;
    } else {
        soll_left_rs = m_ui->sollLeftRS_Test->text().toFloat();
        soll_right_rs = m_ui->sollRightRS_Test->text().toFloat();
    }
    return ControlData{
        soll_left_rs,
        soll_right_rs
    };
}

void MainWindow::send()
{
//    const char send_info[] = {0x55, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x66};

//    float * data_ptr_1 = (float *)(send_info + 1);
//    data_ptr_1[0] = 234.4;
//    float * data_ptr_2 = (float *)(send_info + 5);
//    data_ptr_2[0] = 234.5;

    //生成数据帧
    ControlData _control_data = calculateRS();
    _controlFrame = pack(_control_data);

//    //转换为char数组
//    int struct_real_len = sizeof(_controlFrame);
//    int struct_soll_len = 10;
//    int struct_diff_len = struct_real_len - struct_soll_len;
//    int incre_index = struct_diff_len / 2;
//    qDebug()<<"struct_real_len: "<<struct_real_len;
//    char send_info[struct_real_len];
//    memcpy( send_info, &_controlFrame, sizeof(_controlFrame));

//    float * b = (float *)(send_info+1+incre_index);
//    float * c = (float *)(send_info+5+incre_index);

//    qDebug()<<b[0];
//    qDebug()<<c[0];

    //转换为char数组
    char send_info[10] = {(char) _controlFrame.SOF, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, (char) _controlFrame._EOF};
    float * soll_left_rs = (float *)(send_info + 1);
    soll_left_rs[0] = _controlFrame.soll_left_rs;
    float * soll_right_rs = (float *)(send_info + 5);
    soll_right_rs[0] = _controlFrame.soll_right_rs;

//    float * data_ptr_3 = (float *)(send_info + 1);
//    qDebug()<<data_ptr_3[0]<<"   ";
//    float * data_ptr_4 = (float *)(send_info + 5);
//    qDebug()<<data_ptr_4[0];

    //转换为QByteArray
    QByteArray data(QByteArray::fromRawData(send_info, sizeof(send_info)));
    if (m_serial->isOpen())
        writeData(data);

//    qDebug()<<sizeof(data);
//    char data_par[4];
//    for (int i = 1; i < 5; ++i) {
//         data_par[i-1] = data.at(i);
//    }

//    float * dd = (float *)(data_par);
//    qDebug()<<dd[0];

//    char data_parr[4];
//    for (int i = 1; i < 5; ++i) {
//         data_parr[i-1] = data.at(i+4);
//    }

//    float * ddd = (float *)(data_parr);
//    qDebug()<<ddd[0];
//    qDebug()<<sizeof(data);
    QString str_soll_left_rs = QString::number(_controlFrame.soll_left_rs, 'f', 3);
    QString str_soll_right_rs = QString::number(_controlFrame.soll_right_rs, 'f', 3);
    m_ui->sollLeftRS->setText(str_soll_left_rs);
    m_ui->sollRightRS->setText(str_soll_right_rs);
}

void MainWindow::writeData(const QByteArray &data)
{
    //qDebug()<<"writeData: "<<data<<" with "<<data.count()<<" bytes";
    m_serial->write(data);
}


FeedBackData MainWindow::unpack(MainWindow::FeedBackFrame& fb)
{
    return FeedBackData
    {
        fb.real_left_rs,
        fb.real_right_rs,
        fb.real_left_ra,
        fb.real_right_rs,
    };
}

void MainWindow::readData()
{
    const QByteArray data = m_serial->readAll();

    uint8_t data_len = data.count();
    //qDebug()<<"readData: "<<data<<" with size: "<<data_len;

    if (data.at(0) == 0x05) {
        *Rx_Count = 0;
        //qDebug()<<"Begining "<<*Rx_Count<<"\n";
    }
    for (int i = 0; i < data_len; ++i) {

         Rx_buf[*Rx_Count] = data.at(i);
         *Rx_Count = *Rx_Count + 1;
        //qDebug()<<"i: "<<i<<" rxcount++ : "<<*Rx_Count<<"\n";
    }

    if (data.at(data_len-1) == 0x06) {
        *Rx_Count = 0;
        //qDebug()<<"Last "<<*Rx_Count<<"\n";

        char a[4];
         for (int i = 0; i < 4; i++) {
             a[i] = Rx_buf[i+4];
         }
         float * aa = (float *)a;
         //qDebug()<<QString::number(aa[0], 'f', 3);
         m_ui->realLeftRS->setText(QString::number(aa[0], 'f', 3));

         char b[4];
          for (int i = 0; i < 4; i++) {
              b[i] = Rx_buf[i+8];
          }
          float * bb = (float *)b;
          //qDebug()<<QString::number(bb[0], 'f', 3);
          m_ui->realLeftRA->setText(QString::number(bb[0], 'f', 3));

          char c[4];
           for (int i = 0; i < 4; i++) {
               c[i] = Rx_buf[i+12];
           }
           float * cc = (float *)c;
           //qDebug()<<QString::number(cc[0], 'f', 3);
           m_ui->realRightRS->setText(QString::number(cc[0], 'f', 3));

           char d[4];
            for (int i = 0; i < 4; i++) {
                d[i] = Rx_buf[i+16];
            }
            float * dd = (float *)d;
            //qDebug()<<QString::number(dd[0], 'f', 3);
            m_ui->realRightRA->setText(QString::number(dd[0], 'f', 3));
        }
}

void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), m_serial->errorString());
        closeSerialPort();
    }
}
void MainWindow::resetTestValue()
{
    m_ui->sollLeftRS_Test->setText(QString("0.0"));
    m_ui->sollRightRS_Test->setText(QString("0.0"));
}
void MainWindow::initActionsConnections()
{
    connect(m_ui->actionConnect, &QAction::triggered, this, &MainWindow::openSerialPort);
    connect(m_ui->actionDisconnect, &QAction::triggered, this, &MainWindow::closeSerialPort);
    connect(m_ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(m_ui->actionConfigure, &QAction::triggered, m_settings, &SettingsDialog::show);
    connect(m_ui->actionClear, &QAction::triggered, this, &MainWindow::resetTestValue);
    //connect(m_ui->actionAbout, &QAction::triggered, this, &MainWindow::about);
    //connect(m_ui->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::initConnections()
{
    connect(m_ui->startTest, SIGNAL(clicked()), this, SLOT(start_test()));
    connect(m_ui->stopTest, SIGNAL(clicked()), this, SLOT(stop_test()));
}

void MainWindow::start_test()
{
    m_ui->startTest->setEnabled(false);
    m_ui->stopTest->setEnabled(true);
    Test = true;
}

void MainWindow::stop_test()
{
    m_ui->startTest->setEnabled(true);
    m_ui->stopTest->setEnabled(false);
    Test = false;
}

void MainWindow::showStatusMessage(const QString &message)
{
    m_status->setText(message);
}
