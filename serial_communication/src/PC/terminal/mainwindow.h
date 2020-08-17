#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>

QT_BEGIN_NAMESPACE

class QLabel;

namespace Ui {
class MainWindow;
}

QT_END_NAMESPACE

class Console;
class SettingsDialog;

struct ControlData
{
    float   soll_left_rs;           //左轮目标转速
    float   soll_right_rs;          //右轮目标转速
};

struct FeedBackData
{
    float   real_left_rs;           //左轮实际转速
    float   real_right_rs;          //右轮实际转速
    float   real_left_ra;           //左轮实际转角
    float   real_right_ra;          //右轮实际转角
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openSerialPort();
    void closeSerialPort();
    //void about();
    void writeData(const QByteArray &data);
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
    ControlData calculateRS();
    void resetTestValue();

    Ui::MainWindow *m_ui = nullptr;
    QLabel *m_status = nullptr;
    //Console *m_console = nullptr;
    SettingsDialog *m_settings = nullptr;
    QSerialPort *m_serial = nullptr;

    bool Test = false;
    char Rx_buf[24];
    uint8_t *Rx_Count;

private:
    enum
    {
        JetsonCommSOF = (uint8_t)0x33,
        JetsonCommEOF = (uint8_t)0x44,
        STM32CommSOF = (uint8_t)0x55,
        STM32CommEOF = (uint8_t)0x66,
    };

    struct ControlFrame
    {
        uint8_t  SOF;
        float    soll_left_rs;           //左轮目标转速
        float    soll_right_rs;          //右轮目标转速
        uint8_t  _EOF;
    }_controlFrame;

    struct FeedBackFrame
    {
        uint8_t  SOF;
        float    real_left_rs;           //左轮实际转速
        float    real_right_rs;          //右轮实际转速
        float    real_left_ra;           //左轮实际转角
        float    real_right_ra;          //右轮实际转角
        uint8_t  _EOF;
    }_feedBackFrame;

    ControlFrame pack(ControlData& controlData);
    FeedBackData unpack(FeedBackFrame& FeedBackData);
};

#endif // MAINWINDOW_H
