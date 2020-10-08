//******************************************************************
//yifan DWA
//******************************************************************

#include<iostream>
#include<ctime>
#include<Eigen/Dense>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/int8_multi_array.hpp>

#include "DWA_Planner.h"
 //#include "include/yaml-cpp/yaml.h"

// ����һ��double���͵�NAN
using std::placeholders::_1;
using namespace dwa_planner;
using namespace std;
using namespace Eigen;

// init global value
Control controlspeed, motor_ist, u_;
MatrixXd oblist;
State x;
Goal goal;
MatrixXd traj_(5, 1);
DWA_result plan;
DWA planner;

class DWA_SollspeedPublisher : public rclcpp::Node
{
public:
  DWA_SollspeedPublisher()
  : Node("DWA_SollspeedPublisher"), count_(0)
  {
    publisherSollspeed_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("dwa_sollspeed", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&DWA_SollspeedPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32MultiArray();

	plan = planner.dwa_control(motor_ist, x, goal, oblist);
	//motor_ist = plan.u;
	controlspeed = plan.u;
	u_ = planner.speed_change(plan.u, "MOTOR_TO_PC");
	x = planner.motion(x, u_, planner.dt);

	if (planner.TEMPORARY_GOAL_ARRIVED) {
		std::cout << "goal arrived!" << endl;
		controlspeed<< 0,0;
		planner.TEMPORARY_GOAL_ARRIVED=false;
	}

    std::vector<float> sollspeed;
    sollspeed.push_back(controlspeed(0));
	sollspeed.push_back(controlspeed(1));
	sollspeed.push_back(controlspeed(0));
	sollspeed.push_back(controlspeed(1));

    message.data = sollspeed;
    publisherSollspeed_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisherSollspeed_;
  size_t count_;
};

class DWA_RealspeedSubscriber : public rclcpp::Node
{
public:
  DWA_RealspeedSubscriber()
  : Node("DWA_RealspeedSubscriber")
  {
    subscriptionSpeed_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "dwa_sollspeed", 10, std::bind(&DWA_RealspeedSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      auto RealSpeed = msg->data;
	  motor_ist << RealSpeed[0],RealSpeed[1];
	  //cout<<"speed recieved.."<<endl;
      //std::cout<<"broadcast speed :" << RealSpeed[0]<<","<<RealSpeed[1] <<","<<RealSpeed[2]<<","<<RealSpeed[3]<<std::endl;
   }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionSpeed_;
};

class DWA_Mapsubscriber : public rclcpp::Node
{
public:
  DWA_Mapsubscriber()
  : Node("DWA_Mapsubscriber")
  {
    subscriptionMap_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
      "map", 10, std::bind(&DWA_Mapsubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) const
  {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      auto RealMap = msg->data;
		//oblist << RealMap.data;
	  //cout<<"Map recieved.."<<endl;
   }
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscriptionMap_;
};

class DWA_GoalspeedSubscriber : public rclcpp::Node
{
public:
  DWA_GoalspeedSubscriber()
  : Node("DWA_GoalspeedSubscriber")
  {
    subscriptionGoal_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "dwa_goal", 10, std::bind(&DWA_GoalspeedSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      auto RealGoal = msg->data;
	  goal << RealGoal[0],RealGoal[1];
	  cout<<"goal:"<<goal<<endl;
      //std::cout<<"broadcast speed :" << RealSpeed[0]<<","<<RealSpeed[1] <<","<<RealSpeed[2]<<","<<RealSpeed[3]<<std::endl;
   }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionGoal_;
};

int main(int argc, char **argv) {
	//  init rclcpp
    rclcpp::init(argc, argv);
	// make multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

	auto speedpublisher = std::make_shared<DWA_SollspeedPublisher>();
	auto speedsubscriber = std::make_shared<DWA_RealspeedSubscriber>();
	auto mapsubscriber = std::make_shared<DWA_Mapsubscriber>();
	auto goalsubscriber = std::make_shared<DWA_GoalspeedSubscriber>();
	
	//cout << "max_speed:" << planner.max_speed << endl;
	x << 2.5, -0.3, PI / 2, 0, 0;
	goal << 7.5, 10;
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
	int count = 0;
	// add to the executor
    executor.add_node(speedpublisher);
    executor.add_node(speedsubscriber);
	executor.add_node(mapsubscriber);
	executor.add_node(goalsubscriber);
	executor.spin();
	//rclcpp::spin(std::make_shared<DWASollspeedPublisher>());
	rclcpp::shutdown();
	return 0;
	
}