#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include "RobotThread.h"

QT_BEGIN_NAMESPACE

class QLabel;

namespace Ui {
class MainWindow;
}

QT_END_NAMESPACE

class SettingsDialog;

struct ControlData
{
    float   soll_left_front_rs;           //左前轮目标转速
    float   soll_right_front_rs;          //右前轮目标转速
    float   soll_left_back_rs;           //左后轮目标转速
    float   soll_right_back_rs;          //右后轮目标转速
};

struct FeedBackData
{
    float real_left_front_rs;
    float real_left_front_ra;
    float real_right_front_rs;
    float real_right_front_ra;
    float real_left_back_rs;
    float real_left_back_ra;
    float real_right_back_rs;
    float real_right_back_ra;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget * parent = 0);
    ~MainWindow();

    Q_SLOT void updateSollSpeed(float soll_left_front_rs_, float soll_right_front_rs_, float soll_left_back_rs_, float soll_right_back_rs_);

private slots:
    void openSerialPort();
    void closeSerialPort();
    //void about();
    void readData();
    void send();
    void start_test();
    void stop_test();

    void handleError(QSerialPort::SerialPortError error);



private:
    void initActionsConnections();
    void initConnections();

private:
    void showStatusMessage(const QString &message);
    void resetTestValue();
    void writeData();
    void showSollValue();

    Ui::MainWindow *m_ui = nullptr;
    QLabel *m_status = nullptr;
    //Console *m_console = nullptr;
    SettingsDialog *m_settings = nullptr;
    QSerialPort *m_serial = nullptr;
    RobotThread m_RobotThread;

    bool Test = false;

    char Rx_buf[40];
    uint8_t *Rx_Count;

private:
    enum
    {
        JetsonCommSOF = (uint8_t)0x03,
        JetsonCommEOF = (uint8_t)0x04,
        STM32CommSOF = (uint8_t)0x05,
        STM32CommEOF = (uint8_t)0x06,
    };

    struct ControlFrame
    {
        uint8_t  SOF;
        float    soll_left_front_rs;           //左前轮目标转速
        float    soll_right_front_rs;          //右前轮目标转速
        float   soll_left_back_rs;           //左后轮目标转速
        float   soll_right_back_rs;          //右后轮目标转速
        uint8_t  _EOF;
    }_controlFrame;

    struct FeedBackFrame
    {
        uint8_t  SOF;
        float real_left_front_rs;
        float real_left_front_ra;
        float real_right_front_rs;
        float real_right_front_ra;
        float real_left_back_rs;
        float real_left_back_ra;
        float real_right_back_rs;
        float real_right_back_ra;
        uint8_t  _EOF;
    }_feedBackFrame;
};

#endif // MAINWINDOW_H
