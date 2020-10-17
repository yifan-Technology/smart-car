//******************************************************************
//yifan DWA
//******************************************************************

#include <iostream>
#include <ctime>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <cv_bridge/cv_bridge.h>

#include "DWA_Planner.h"

// ????h??double?????NAN
using std::placeholders::_1;
using namespace dwa_planner;
using namespace std;
using namespace Eigen;

// init global value
Control controlspeed, motor_ist, u_;
MatrixXd oblist;
State car_x;
Goal goal;
MatrixXd traj_(5, 1);
DWA_result plan;
DWA planner;
clock_t start = clock();

float figure_size = 600;
float scale_up = 90;
int count0 = 0;


cv::Point2i cv_offset(
    double x, double y, int image_width = 600, int image_height = 600)
{
    cv::Point2i output;
    output.x = int(x * 90) + image_width / 9;
    output.y = image_height - int(y * 90) - image_height / 9;
    return output;
}

class DWA_SollspeedPublisher : public rclcpp::Node
{
public:
    DWA_SollspeedPublisher()
        : Node("DWA_SollspeedPublisher"), count_(0)
    {
        publisherSollspeed_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("soll_speed", 10);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&DWA_SollspeedPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Float32MultiArray();
        //cout << "car_x: " << car_x.transpose() << endl;
        //cout << "motor ist " << motor_ist.transpose() << endl;
        cout << "goal " << goal.transpose() << endl;
        plan = planner.dwa_control(motor_ist, car_x, goal, oblist);
        controlspeed = plan.u;
        //cout<<"motor soll"<<controlspeed.transpose()<<endl;
        if (!planner.RESET_STATE)
        {
            u_ = planner.speed_change(plan.u, "MOTOR_TO_PC");
            car_x = planner.motion(car_x, u_, planner.dt);
            traj_.conservativeResize(traj_.rows(), traj_.cols() + 1);
            traj_.col(traj_.cols() - 1) = car_x;
        }

        if (planner.TEMPORARY_GOAL_ARRIVED)
        {
            std::cout << "goal arrived!" << endl;
            controlspeed << 0, 0;
            planner.TEMPORARY_GOAL_ARRIVED = false;
        }

        std::vector<float> sollspeed;
        sollspeed.push_back(controlspeed(0));
        sollspeed.push_back(controlspeed(1));
        sollspeed.push_back(controlspeed(0));
        sollspeed.push_back(controlspeed(1));

        message.data = sollspeed;
        publisherSollspeed_->publish(message);

        if (planner.MEASURE_TIME)
        {
            clock_t end = clock();
            double elapsed_time = (double(end) - double(start)) / (CLOCKS_PER_SEC);
            std::cout << "FPS is:" << 1 / elapsed_time << endl;
            start = clock();
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisherSollspeed_;
    size_t count_;
};

class DWA_Subscriber : public rclcpp::Node
{
public:
    DWA_Subscriber()
        : Node("DWA_Subscriber")
    {
        subscriptionSpeed_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "realspeed", 1, std::bind(&DWA_Subscriber::topicRealspeed_callback, this, _1));

        // subscriptionMap_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "/yf_camera/obmap", 1, std::bind(&DWA_Subscriber::topicObmap_callback, this, _1));

        subscriptionMaplist_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/yf_camera/obList", 1, std::bind(&DWA_Subscriber::topicOblist_callback, this, _1));

        // subscriptionGoal_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "/yf_camera/goal", 1, std::bind(&DWA_Subscriber::topicGoal_callback, this, _1));

        subscriptionGoal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/yf_camera/goal", 1, std::bind(&DWA_Subscriber::topicGoal_callback, this, _1));
    }

private:
    void topicRealspeed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr speedmsg) const
    {
        auto RealSpeed = speedmsg->data;
        float motor_left = (RealSpeed[0] + RealSpeed[2]) / 2;
        float motor_right = (RealSpeed[4] + RealSpeed[6]) / 2;
        motor_ist << motor_left, motor_right;
    }

    // void topicObmap_callback(const sensor_msgs::msg::Image::SharedPtr mapmsg) const
    // {
    //   cv_bridge::CvImagePtr cv_ptr;
    //   try
    //   {
    //     cv_ptr = cv_bridge::toCvCopy(mapmsg, sensor_msgs::image_encodings::BGRA8);
    //     cv::Mat img = cv_ptr->image;
    //     MatrixXd obmap(img.cols, img.rows);
    //     cv::cv2eigen(img, obmap);
    //     oblist = planner.obmap2coordinaten(obmap, 5 / 50);
    //   }
    //   catch (cv_bridge::Exception &e)
    //   {
    //     std::cout << "Map recieve fail! " << std::endl;
    //   }
    // }

    void topicOblist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr mapmsg) const
    {
        std::vector<float> v = mapmsg->data;
        float* v2 = v.data();
        Map<MatrixXf> ob(v2, 2, mapmsg->data.size() / 2);
        oblist = ob.cast<double>();
    }

    void topicGoal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message) const
    {
        Goal temp_goal;
        temp_goal << message->pose.position.x , message->pose.position.z;
        goal = temp_goal;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionSpeed_;
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionMap_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionMaplist_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriptionGoal_;
};

//class DWA_RealspeedSubscriber : public rclcpp::Node
//{
//public:
//    DWA_RealspeedSubscriber()
//        : Node("DWA_RealspeedSubscriber")
//    {
//        subscriptionSpeed_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//            "soll_speed", 1, std::bind(&DWA_RealspeedSubscriber::topic_callback, this, _1));
//    }
//
//private:
//    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
//    {
//        //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//        auto RealSpeed = msg->data;
//        motor_ist << RealSpeed[0], RealSpeed[1];
//        //cout<<"speed recieved.."<<endl;
//        //std::cout<<"broadcast speed :" << RealSpeed[0]<<","<<RealSpeed[1] <<","<<RealSpeed[2]<<","<<RealSpeed[3]<<std::endl;
//    }
//    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionSpeed_;
//};
//
//class DWA_GoalspeedSubscriber : public rclcpp::Node
//{
//public:
//    DWA_GoalspeedSubscriber()
//        : Node("DWA_GoalspeedSubscriber")
//    {
//        subscriptionGoal_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//            "dwa_goal", 10, std::bind(&DWA_GoalspeedSubscriber::topic_callback, this, _1));
//    }
//
//private:
//    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
//    {
//        //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//        auto RealGoal = msg->data;
//        goal << RealGoal[0], RealGoal[1];
//        cout << "goal:" << goal << endl;
//        //std::cout<<"broadcast speed :" << RealSpeed[0]<<","<<RealSpeed[1] <<","<<RealSpeed[2]<<","<<RealSpeed[3]<<std::endl;
//    }
//    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionGoal_;
//};

class DWA_Visualizer : public rclcpp::Node
{
public:
    DWA_Visualizer()
        : Node("DWA_Visualizer"), count_(0)
    {
        timer_ = this->create_wall_timer(
            10ms, std::bind(&DWA_Visualizer::timer_callback, this));
    }

private:
    void timer_callback() 
    {
        ////// visualization
        if (planner.SHOW_ANIMATION)
        {
            //cout << "bis visulizer check" << endl;
            cv::Mat bg(figure_size, figure_size, CV_8UC3, cv::Scalar(255, 255, 255));

            // draw goal
            cv::circle(bg, cv_offset(goal(0) + 2.56, goal(1), bg.cols, bg.rows), int(scale_up * 0.5), cv::Scalar(255, 0, 0), int(figure_size / 200));

            // draw calibration 1m
            //cv::circle(bg, 0,0, 10, cv::Scalar(155, 155, 115), int(figure_size / 200));
            //cv::circle(bg, cv_offset(2.56, 1, bg.cols, bg.rows), int(scale_up0.5), cv::Scalar(155, 155, 115), int(figure_size / 200));

            // draw maximum detect range
            cv::rectangle(bg, (cv_offset(0, 0, bg.cols, bg.rows)), (cv_offset(5, 5, bg.cols, bg.rows)), cv::Scalar(10, 205, 215), int(figure_size / 200));

            // draw all obstacle
            for (unsigned int j = 0; j < oblist.cols(); j++)
            {
                cv::circle(bg, cv_offset(oblist(0, j) + 2.56, oblist(1, j), bg.cols, bg.rows), int(figure_size / 200), cv::Scalar(0, 0, 0), -1);
            }

            /* if(!planner.RESET_STATE){
               for (unsigned int j = 0; j < traj_.cols(); j++)
                 {
                   cv::circle(bg, cv_offset(traj_(0, j)+2.5, traj_(1, j), bg.cols, bg.rows), int(figure_size / 500), cv::Scalar(10, 55, 100), -1);
                 }
             }*/
             // draw car all posible trajectory
            for (unsigned int i = 0; i < plan.all_traj.size(); i++)
            {
                for (unsigned int j = 0; j < plan.all_traj[i].cols(); j++)
                {
                    cv::circle(bg, cv_offset(plan.all_traj[i](0, j) + 2.56, plan.all_traj[i](1, j), bg.cols, bg.rows), int(figure_size / 300), cv::Scalar(0, 255, 0), -1);
                }
            }

            // draw car optimal trajectory
            for (unsigned int j = 0; j < plan.traj.cols(); j++)
            {
                cv::circle(bg, cv_offset(plan.traj(0, j) + 2.56, plan.traj(1, j), bg.cols, bg.rows), int(figure_size / 200), cv::Scalar(139, 134, 0), -1);
            }

            // draw car body
            MatrixXd diag_pos(4, 2);
            diag_pos << -planner.robot_width / 2, +planner.robot_length / 2,
                -planner.robot_width / 2, -planner.robot_length / 2,
                +planner.robot_width / 2, -planner.robot_length / 2,
                +planner.robot_width / 2, +planner.robot_length / 2;
            Matrix2d rot_bot;
            float theta = car_x(2) - PI / 2;
            rot_bot << cos(theta), -sin(theta), sin(theta), cos(theta);
            Matrix<double, 1, 2> pos0;
            pos0 << car_x(0), car_x(1);
            MatrixXd trans = pos0.replicate(4, 1);
            MatrixXd rec_pos = (diag_pos * rot_bot.transpose()) + trans;
            cv::line(bg, cv_offset(rec_pos(0, 0) + 2.56, rec_pos(0, 1)), cv_offset(rec_pos(1, 0) + 2.56, rec_pos(1, 1)), cv::Scalar(100, 0, 200), int(figure_size / 200));
            cv::line(bg, cv_offset(rec_pos(1, 0) + 2.56, rec_pos(1, 1)), cv_offset(rec_pos(2, 0) + 2.56, rec_pos(2, 1)), cv::Scalar(100, 0, 200), int(figure_size / 200));
            cv::line(bg, cv_offset(rec_pos(2, 0) + 2.56, rec_pos(2, 1)), cv_offset(rec_pos(3, 0) + 2.56, rec_pos(3, 1)), cv::Scalar(100, 0, 200), int(figure_size / 200));
            cv::line(bg, cv_offset(rec_pos(3, 0) + 2.56, rec_pos(3, 1)), cv_offset(rec_pos(0, 0) + 2.56, rec_pos(0, 1)), cv::Scalar(100, 0, 200), int(figure_size / 200));

            // draw car safe circle
            cv::circle(bg, cv_offset(car_x(0) + 2.56, car_x(1), bg.cols, bg.rows), int(scale_up * 0.5), cv::Scalar(0, 0, 255), 2);

            // draw car heading arrow
            cv::arrowedLine(bg, cv_offset(car_x(0) + 2.56, car_x(1), bg.cols, bg.rows),
                cv_offset(car_x(0) + 2.56 + std::cos(car_x(2)), car_x(1) + std::sin(car_x(2)), bg.cols, bg.rows),
                cv::Scalar(255, 0, 255),
                int(figure_size / 300));

            if ((car_x.head(2) - goal).norm() <= planner.robot_radius)
            {
                cv::circle(bg, cv_offset(goal(0) + 2.56, goal(1), bg.cols, bg.rows), int(scale_up * 0.5), cv::Scalar(0, 205, 0), int(figure_size / 150));
                // draw reaching goal
                if (!planner.RESET_STATE) {
                    for (unsigned int j = 0; j < traj_.cols(); j++)
                    {
                        cv::circle(bg, cv_offset(traj_(0, j), traj_(1, j), bg.cols, bg.rows),
                            int(figure_size / 200), cv::Scalar(0, 0, 255), -1);
                    }
                }

            }
            //cout << "reach before publish dwa state" << endl;
            //if (planner.PUBLISH_DWA_STATE) {
            //	cout<<"reach publish dwa state"<<endl;
            //	cv::Mat outImg;
            //	//cv::resize(bg, outImg, cv::Size(200, 200), 0, 0, cv::INTER_AREA);
            //	cv::imshow("dwa", bg);
            //	cv::waitKey(10);
            //	std::string int_count = std::to_string(count0);
            //	cv::imwrite("./pngs/" + std::string(5 - int_count.length(), '0').append(int_count) + ".png", bg);
            //	auto message = sensor_msgs::msg::CompressedImage();
            //	std_msgs::msg::Header header; // empty header
            //	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, bg);
            //	img_bridge.toCompressedImageMsg(message); // from cv_bridge to sensor_msgs::Image
            //	publisherImg_->publish(message);
            //	
            //}

            cv::imshow("dwa", bg);
            cv::waitKey(10);

            std::string int_count = std::to_string(count0);
            cv::imwrite("./pngs/" + std::string(5 - int_count.length(), '0').append(int_count) + ".png", bg);

        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char** argv)
{
    planner.readJsonFromFile();
    //  init rclcpp
    rclcpp::init(argc, argv);
    // make multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    auto speedpublisher = std::make_shared<DWA_SollspeedPublisher>();
    auto dwasubscriber = std::make_shared<DWA_Subscriber>();
    // auto speedsubscriber = std::make_shared<DWA_RealspeedSubscriber>();
    // auto goalsubscriber = std::make_shared<DWA_GoalspeedSubscriber>();
    auto visualizer = std::make_shared<DWA_Visualizer>();
    //cout << "max_speed:" << planner.max_speed << endl;
    car_x << 0., -0.3, PI / 2, 0, 0;
    //carx << 2.5, -0.3, PI / 2, 0, 0;
    goal << 0.5, 0.5;
    MatrixXd obin(20, 2);
    obin << -1, -1,
        -1.5, -1.5,
        -2, -2,
        0, 2,
        0.5, 2.5,
        4.0, 2.0,
        4.5, 2.0,
        5.0, 4.0,
        5.0, 4.5,
        5.0, 5.0,
        5.0, 6.0,
        5.0, 9.0,
        8.0, 9.0,
        7.0, 9.0,
        8.0, 10.0,
        9.0, 11.0,
        12.0, 13.0,
        12.0, 12.0,
        15.0, 15.0,
        13.0, 13.0;
    oblist = obin.transpose();
    motor_ist << 0, 0;
    std::cout << "start dwa planning!" << endl;

    if (planner.SHOW_ANIMATION)
    {
        cv::namedWindow("dwa", cv::WINDOW_NORMAL);
        cv::resizeWindow("dwa", 720, 720);
    }
    // add to the executor
    executor.add_node(speedpublisher);
    executor.add_node(dwasubscriber);
    //executor.add_node(goalsubscriber);
    executor.add_node(visualizer);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}