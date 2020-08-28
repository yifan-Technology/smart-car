#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "settingsdialog.h"
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QTimer>
#include <string.h>
#include <QtSerialPort/QSerialPort>
#include <QThread>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_status(new QLabel)
{
    m_ui->setupUi(this);

    m_serial = new QSerialPort(this);
    m_settings = new SettingsDialog;

    m_ui->actionConnect->setEnabled(true);
    m_ui->actionDisconnect->setEnabled(false);
    m_ui->actionQuit->setEnabled(true);
    m_ui->actionConfigure->setEnabled(true);

    m_ui->statusBar->addWidget(m_status);

    initActionsConnections();
    initConnections();
    connect(m_serial, &QSerialPort::errorOccurred, this, &MainWindow::handleError);
    connect(m_serial, &QSerialPort::readyRead, this, &MainWindow::readData);

    QTimer *timer = new QTimer;
    connect(timer, SIGNAL(timeout()), SLOT(send()));
    timer->start(25);

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
    emit m_ui->stopTest->click();

    if (m_serial->isOpen())
        m_serial->close();

    m_ui->actionConnect->setEnabled(true);
    m_ui->actionDisconnect->setEnabled(false);
    m_ui->actionConfigure->setEnabled(true);
    showStatusMessage(tr("Disconnected"));
//    m_ui->realLeftFrontRS->setText("0.0");
//    m_ui->realLeftFrontRA->setText("0.0");
//    m_ui->realRightFrontRS->setText("0.0");
//    m_ui->realRightFrontRA->setText("0.0");
//    m_ui->realLeftBackRS->setText("0.0");
//    m_ui->realLeftBackRA->setText("0.0");
//    m_ui->realRightBackRS->setText("0.0");
//    m_ui->realRightBackRA->setText("0.0");

}

MainWindow::ControlFrame MainWindow::pack(ControlData& ctrl)
{
    return ControlFrame
    {
        JetsonCommSOF,
        ctrl.soll_left_front_rs,
        ctrl.soll_right_front_rs,
        ctrl.soll_left_back_rs,
        ctrl.soll_right_back_rs,
        JetsonCommEOF
    };
}

ControlData MainWindow::calculateRS()
{
    float soll_left_front_rs;
    float soll_right_front_rs;
    float soll_left_back_rs;
    float soll_right_back_rs;

    float rand_error = rand()/(RAND_MAX+1.0);
    rand_error = (int)(1000*rand_error) / 1000.0;


    if (!Test) {
        soll_left_front_rs = 0.0;
        soll_right_front_rs = 0.0;
        soll_left_back_rs = 0.0;
        soll_right_back_rs = 0.0;
    } else {
        soll_left_front_rs = m_ui->sollLeftFrontRS_Test->text().toFloat();
        soll_left_front_rs *= 19.0;
        soll_right_front_rs = m_ui->sollRightFrontRS_Test->text().toFloat();
        soll_right_front_rs *= 19.0;
        soll_left_back_rs = m_ui->sollLeftBackRS_Test->text().toFloat();
        soll_left_back_rs *= 19.0;
        soll_right_back_rs = m_ui->sollRightBackRS_Test->text().toFloat();
        soll_right_back_rs *= 19.0;
    }

    return ControlData{
        soll_left_front_rs,
        soll_right_front_rs,
        soll_left_back_rs,
        soll_right_back_rs
    };
}

void MainWindow::writeData()
{
    //生成数据帧
    ControlData _control_data = calculateRS();
    _controlFrame = pack(_control_data);

    //转换为char数组
    char send_info[18] = {(char) _controlFrame.SOF, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, (char) _controlFrame._EOF};
    float * soll_left_front_rs = (float *)(send_info + 1);
    soll_left_front_rs[0] = _controlFrame.soll_left_front_rs;
    float * soll_right_front_rs = (float *)(send_info + 5);
    soll_right_front_rs[0] = _controlFrame.soll_right_front_rs;
    float * soll_left_back_rs = (float *)(send_info + 9);
    soll_left_back_rs[0] = _controlFrame.soll_left_back_rs;
    float * soll_right_back_rs = (float *)(send_info + 13);
    soll_right_back_rs[0] = _controlFrame.soll_right_back_rs;

    //转换为QByteArray
    QByteArray data(QByteArray::fromRawData(send_info, sizeof(send_info)));
    if (m_serial->isOpen())
        m_serial->write(data);
}

void MainWindow::showSollValue()
{
    QString str_soll_left_front_rs = QString::number(_controlFrame.soll_left_front_rs/19.0, 'f', 3);
    QString str_soll_right_front_rs = QString::number(_controlFrame.soll_right_front_rs/19.0, 'f', 3);
    QString str_soll_left_back_rs = QString::number(_controlFrame.soll_left_back_rs/19.0, 'f', 3);
    QString str_soll_right_back_rs = QString::number(_controlFrame.soll_right_back_rs/19.0, 'f', 3);
    m_ui->sollLeftFrontRS->setText(str_soll_left_front_rs);
    m_ui->sollRightFrontRS->setText(str_soll_right_front_rs);
    m_ui->sollLeftBackRS->setText(str_soll_left_back_rs);
    m_ui->sollRightBackRS->setText(str_soll_right_back_rs);
}

void MainWindow::send()
{
    if (m_serial->isOpen()) {
        writeData();
        showSollValue();
    }
}

void MainWindow::readData()
{
    const QByteArray data = m_serial->readAll();

    uint8_t data_len = data.count();
    //qDebug()<<"readData: "<<data<<" with size: "<<data_len;

    if (data.at(0) == STM32CommSOF) {
        *Rx_Count = 0;
        //qDebug()<<"Begining "<<*Rx_Count<<"\n";
    }
    for (int i = 0; i < data_len; ++i) {

         Rx_buf[*Rx_Count] = data.at(i);
         *Rx_Count = *Rx_Count + 1;
         //qDebug()<<"i: "<<i<<" rxcount++ : "<<*Rx_Count<<"\n";
    }

    if (data.at(data_len-1) == STM32CommEOF) {
        *Rx_Count = 0;
        //qDebug()<<"Last "<<*Rx_Count<<"\n";

        char a[4];
         for (int i = 0; i < 4; i++) {
             a[i] = Rx_buf[i+4];
         }
         float * aa = (float *)a;
         //qDebug()<<QString::number(aa[0], 'f', 3);
         m_ui->realLeftFrontRS->setText(QString::number(aa[0]/19.0, 'f', 3));

         char b[4];
          for (int i = 0; i < 4; i++) {
              b[i] = Rx_buf[i+8];
          }
          float * bb = (float *)b;
          //qDebug()<<QString::number(bb[0], 'f', 3);
          m_ui->realLeftFrontRA->setText(QString::number(bb[0]/19.0, 'f', 3));

          char c[4];
           for (int i = 0; i < 4; i++) {
               c[i] = Rx_buf[i+12];
           }
           float * cc = (float *)c;
           //qDebug()<<QString::number(cc[0], 'f', 3);
           m_ui->realRightFrontRS->setText(QString::number(cc[0]/19.0, 'f', 3));

           char d[4];
            for (int i = 0; i < 4; i++) {
                d[i] = Rx_buf[i+16];
            }
            float * dd = (float *)d;
            //qDebug()<<QString::number(dd[0], 'f', 3);
            m_ui->realRightFrontRA->setText(QString::number(dd[0]/19.0, 'f', 3));

            char e[4];
             for (int i = 0; i < 4; i++) {
                 e[i] = Rx_buf[i+20];
             }
             float * ee = (float *)e;
             //qDebug()<<QString::number(ee[0], 'f', 3);
             m_ui->realLeftBackRS->setText(QString::number(ee[0]/19.0, 'f', 3));

             char f[4];
              for (int i = 0; i < 4; i++) {
                  f[i] = Rx_buf[i+24];
              }
              float * ff = (float *)f;
              //qDebug()<<QString::number(ff[0], 'f', 3);
              m_ui->realLeftBackRA->setText(QString::number(ff[0]/19.0, 'f', 3));

              char g[4];
               for (int i = 0; i < 4; i++) {
                   g[i] = Rx_buf[i+28];
               }
               float * gg = (float *)g;
               //qDebug()<<QString::number(gg[0], 'f', 3);
               m_ui->realRightBackRS->setText(QString::number(gg[0]/19.0, 'f', 3));

               char h[4];
                for (int i = 0; i < 4; i++) {
                    h[i] = Rx_buf[i+32];
                }
                float * hh = (float *)h;
                //qDebug()<<QString::number(hh[0], 'f', 3);
                m_ui->realRightBackRA->setText(QString::number(hh[0]/19.0, 'f', 3));
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
    m_ui->sollLeftFrontRS_Test->setText(QString("0.0"));
    m_ui->sollRightFrontRS_Test->setText(QString("0.0"));
    m_ui->sollLeftBackRS_Test->setText(QString("0.0"));
    m_ui->sollRightBackRS_Test->setText(QString("0.0"));
}
void MainWindow::initActionsConnections()
{
    connect(m_ui->actionConnect, &QAction::triggered, this, &MainWindow::openSerialPort);
    connect(m_ui->actionDisconnect, &QAction::triggered, this, &MainWindow::closeSerialPort);
    connect(m_ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(m_ui->actionConfigure, &QAction::triggered, m_settings, &SettingsDialog::show);
    connect(m_ui->actionClear, &QAction::triggered, this, &MainWindow::resetTestValue);
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
