#ifndef ___ROBOTTHREAD_H___
#define ___ROBOTTHREAD_H___

#include <QtCore>
#include <QThread>
#include <QStringList>
#include <stdlib.h>
#include <QMutex>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <geometry_msgs/Twist.h>

class RobotThread : public QObject {
	Q_OBJECT
public:
    RobotThread(int argc, char **pArgv);
    virtual ~RobotThread();

//    double getSollLeftFrontSpeed();
//    double getSollRightFrontSpeed();
//    double getSollLeftBackSpeed();
//    double getSollRightBackSpeed();

    bool init();

    void sollSpeedCallback(const geometry_msgs::Twist & msg);

    void setRealSpeed(QList<float> to_set);

    Q_SLOT void run();

    Q_SIGNAL void newSollSpeed(float,float,float,float);
private:
    int m_Init_argc;
    char** m_pInit_argv;

    double  soll_left_front_rs;           //左前轮目标转速
    double  soll_right_front_rs;          //右前轮目标转速
    double  soll_left_back_rs;           //左后轮目标转速
    double  soll_right_back_rs;          //右后轮目标转速

    double real_left_front_rs;
    //double real_left_front_ra;
    double real_right_front_rs;
    //double real_right_front_ra;
    double real_left_back_rs;
    //double real_left_back_ra;
    double real_right_back_rs;
    //double real_right_back_ra;

    QThread * m_pThread;

    ros::Subscriber soll_speed_listener;
    ros::Publisher  real_speed_publisher;
};
#endif

