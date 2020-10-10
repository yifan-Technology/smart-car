//******************************************************************
//yifan DWA
//******************************************************************

#include<iostream>
#include<ctime>
#include<Eigen/Dense>
#include <jsoncpp/json/json.h>
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
clock_t start = clock();

float figure_size = 1000;
float scale_up = 50;
int count0 = 0;

cv::Point2i cv_offset(
	double x, double y, int image_width = 1000, int image_height = 1000) {
	cv::Point2i output;
	output.x = int(x * 50) + image_width / 5;
	output.y = image_height - int(y * 50) - image_height / 5;
	return output;
}

class DWA_SollspeedPublisher : public rclcpp::Node
{
public:
  DWA_SollspeedPublisher()
  : Node("DWA_SollspeedPublisher"), count_(0)
  {
    publisherSollspeed_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("dwa_sollspeed", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&DWA_SollspeedPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32MultiArray();

    plan = planner.dwa_control(motor_ist, x, planner.test_goal, oblist);
    //motor_ist = plan.u;
    controlspeed = plan.u;
    u_ = planner.speed_change(plan.u, "MOTOR_TO_PC");
    x = planner.motion(x, u_, planner.dt);
    traj_.conservativeResize(traj_.rows(), traj_.cols() + 1);
    traj_.col(traj_.cols() - 1) = x;
    //cout<<"current state: " << x.transpose() <<endl;

    if (planner.TEMPORARY_GOAL_ARRIVED) {
      std::cout << "goal arrived!" << endl;
      controlspeed<< 0,0;
      planner.TEMPORARY_GOAL_ARRIVED = true;
    }

    std::vector<float> sollspeed;
    sollspeed.push_back(controlspeed(0));
    sollspeed.push_back(controlspeed(1));
    sollspeed.push_back(controlspeed(0));
    sollspeed.push_back(controlspeed(1));

      message.data = sollspeed;
      publisherSollspeed_->publish(message);

    

    if (planner.MEASURE_TIME) {
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

class DWA_Visualizer : public rclcpp::Node
{
public:
  DWA_Visualizer()
  : Node("DWA_Visualizer")
  {
    subscriptionSpeed_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "dwa_sollspeed", 10, std::bind(&DWA_Visualizer::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
////// visualization
	
		if (planner.SHOW_ANIMATION) {

			cv::Mat bg(figure_size, figure_size, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::circle(bg, cv_offset(planner.test_goal(0), planner.test_goal(1), bg.cols, bg.rows),	20, cv::Scalar(255, 0, 0), int(figure_size/200));

			for (unsigned int j = 0; j < oblist.cols(); j++) {
				cv::circle(bg, cv_offset(oblist(0, j), oblist(1, j), bg.cols, bg.rows), int(figure_size / 200), cv::Scalar(0, 0, 0), -1);}

			for (unsigned int j = 0; j < traj_.cols(); j++) {
				cv::circle(bg, cv_offset(traj_(0, j), traj_(1, j), bg.cols, bg.rows),	int(figure_size/500), cv::Scalar(10, 55, 100), -1);}

			for (unsigned int i = 0; i < plan.all_traj.size(); i++) {
				for (unsigned int j = 0; j < plan.all_traj[i].cols(); j++){
					cv::circle(bg, cv_offset(plan.all_traj[i](0, j), plan.all_traj[i](1, j), bg.cols, bg.rows), int(figure_size / 500), cv::Scalar(0, 255, 0), -1);
				}
			}
			for (unsigned int j = 0; j < plan.traj.cols(); j++) {
				cv::circle(bg, cv_offset(plan.traj(0, j), plan.traj(1, j), bg.cols, bg.rows), int(figure_size / 250), cv::Scalar(139, 134 ,0), -1);
			}

			MatrixXd diag_pos(4,2);
			diag_pos << - planner.robot_width / 2,  + planner.robot_length / 2, 
								 - planner.robot_width / 2,  - planner.robot_length / 2,
								+ planner.robot_width / 2,  - planner.robot_length / 2,
								+ planner.robot_width / 2, + planner.robot_length / 2;
		
			Matrix2d rot_bot;
			float theta = x(2) -PI/2;
			rot_bot << cos(theta), -sin(theta), sin(theta), cos(theta);
			Matrix<double, 1,2> pos0;
			pos0<<x(0), x(1);
			MatrixXd trans = pos0.replicate(4,1);
			MatrixXd rec_pos = (diag_pos*rot_bot.transpose()) + trans;
			cv::line(bg, cv_offset(rec_pos(0,0), rec_pos(0,1)), cv_offset(rec_pos(1, 0), rec_pos(1, 1)), cv::Scalar(0, 0, 200), int(figure_size / 200));
			cv::line(bg, cv_offset(rec_pos(1, 0), rec_pos(1, 1)), cv_offset(rec_pos(2, 0), rec_pos(2, 1)), cv::Scalar(0, 0, 200), int(figure_size / 200));
			cv::line(bg, cv_offset(rec_pos(2, 0), rec_pos(2, 1)), cv_offset(rec_pos(3, 0), rec_pos(3, 1)), cv::Scalar(0, 0, 200), int(figure_size / 200));
			cv::line(bg, cv_offset(rec_pos(3, 0), rec_pos(3, 1)), cv_offset(rec_pos(0, 0), rec_pos(0, 1)), cv::Scalar(0, 0, 200), int(figure_size / 200));
			cv::circle(bg, cv_offset(x(0), x(1), bg.cols, bg.rows), int(scale_up*0.5), cv::Scalar(0, 0, 255), 2);
			cv::arrowedLine(bg,	cv_offset(x(0), x(1), bg.cols, bg.rows),
				cv_offset(x(0) + std::cos(x(2)), x(1) + std::sin(x(2)), bg.cols, bg.rows),
				cv::Scalar(255, 0, 255),
				int(figure_size / 300));
			if ((x.head(2) - goal).norm() <= planner.robot_radius) {

				for (unsigned int j = 0; j < traj_.cols(); j++) {
					cv::circle(bg, cv_offset(traj_(0, j), traj_(1, j), bg.cols, bg.rows),
						int(figure_size / 200), cv::Scalar(0, 0, 255), -1);
				}
			}
			cv::imshow("dwa", bg);
			cv::waitKey(150);
			
			std::string int_count = std::to_string(count0);
			cv::imwrite("./pngs/" + std::string(5 - int_count.length(), '0').append(int_count) + ".png", bg);

		}
   }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionSpeed_;
};


int main(int argc, char **argv) {
  planner.readJsonFromFile();
	//  init rclcpp
    rclcpp::init(argc, argv);
	// make multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

	auto speedpublisher = std::make_shared<DWA_SollspeedPublisher>();
	auto speedsubscriber = std::make_shared<DWA_RealspeedSubscriber>();
	auto mapsubscriber = std::make_shared<DWA_Mapsubscriber>();
	auto goalsubscriber = std::make_shared<DWA_GoalspeedSubscriber>();
	auto visualizer = std::make_shared<DWA_Visualizer>();
	//cout << "max_speed:" << planner.max_speed << endl;
	x << 2.5, -0.3, PI / 2, 0, 0;
	//goal << 6.5, 10;
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

  if(planner.SHOW_ANIMATION){
    cv::namedWindow("dwa", cv::WINDOW_NORMAL);
    cv::resizeWindow("dwa", 720, 720);
  }
	// add to the executor
  executor.add_node(speedpublisher);
  executor.add_node(speedsubscriber);
	executor.add_node(mapsubscriber);
	executor.add_node(goalsubscriber);
  executor.add_node(visualizer);
	executor.spin();
	rclcpp::shutdown();
	return 0;
	
}