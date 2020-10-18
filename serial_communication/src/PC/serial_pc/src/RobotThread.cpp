#include "RobotThread.h"
#include "qdebug.h"

RobotThread::RobotThread(int argc, char** pArgv)
    :	m_Init_argc(argc),
        m_pInit_argv(pArgv)
{/** Constructor for the robot thread **/}

RobotThread::~RobotThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }//end if

    m_pThread->wait();
}//end destructor

bool RobotThread::init()
{
    m_pThread = new QThread();
    this->moveToThread(m_pThread);

    connect(m_pThread, &QThread::started, this, &RobotThread::run);
    ros::init(m_Init_argc, m_pInit_argv, "ros_gui");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    real_speed_publisher  = nh.advertise<geometry_msgs::Twist>("/real_speed", 100);
    soll_speed_listener = nh.subscribe("/soll_speed", 100, &RobotThread::sollSpeedCallback, this);

    m_pThread->start();
    return true;
}//set up the thread

void RobotThread::sollSpeedCallback(const geometry_msgs::Twist & msg)
{
    QMutex * pMutex = new QMutex();

    pMutex->lock();
    soll_left_front_rs = msg.linear.x;
    soll_right_front_rs = msg.linear.y;
    soll_left_back_rs = msg.linear.z;
    soll_left_back_rs = msg.angular.x;
    pMutex->unlock();

    delete pMutex;

    Q_EMIT newSollSpeed((float) soll_left_front_rs, (float) soll_right_front_rs, (float) soll_left_back_rs, (float) soll_left_back_rs);
}//callback method to update the robot's position.

void RobotThread::run()
{
    ros::Rate loop_rate(40);
    QMutex * pMutex;
    while (ros::ok())
    {
        pMutex = new QMutex();

        geometry_msgs::Twist cmd_msg;
        pMutex->lock();
        cmd_msg.linear.x = real_left_front_rs;
        cmd_msg.linear.y = real_right_front_rs;
        cmd_msg.linear.z = real_left_back_rs;
        cmd_msg.angular.x = real_right_back_rs;
        pMutex->unlock();

        real_speed_publisher.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
        delete pMutex;
    }//do ros things.
}

void RobotThread::setRealSpeed(QList<float> to_set)
{
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    real_left_front_rs = (double) to_set.at(0);
    real_right_front_rs = (double) to_set.at(1);
    real_left_back_rs = (double) to_set.at(2);
    real_right_back_rs = (double) to_set.at(3);
    pMutex->unlock();

    delete pMutex;
}//set the speed of the robot.



