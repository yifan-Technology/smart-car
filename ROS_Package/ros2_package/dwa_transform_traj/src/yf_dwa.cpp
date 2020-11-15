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
#include <math.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "DWA_Planner.h"

using std::placeholders::_1;
using namespace dwa_planner;
using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;


// init global value
Control controlspeed, motor_ist, u_, motor_speed0;
Pose lastPose;
MatrixXd oblist;
State car_x, car_x0;
Goal goal, zwGoal;
MatrixXd traj_(5, 1);
DWA_result plan, vorplan;
DWA planner;
clock_t start = clock();
MatrixXd sollTraj0(3,2), sollTrajB, sollTrajW;
int goal_i = 0;
bool PoseInitialized = false;

float figure_size = 500;
float scale_up = 70;
int count0 = 0;

cv::Point2i cv_offset(
	double x, double y, int image_width = figure_size, int image_height = figure_size)
{
	cv::Point2i output;
	output.x = int(x * scale_up) + image_width / scale_up * 10;
	output.y = image_height - int(y * scale_up) - image_height / scale_up * 10;
	return output;
}

class DWA_SollspeedPublisher : public rclcpp::Node
{
public:
	DWA_SollspeedPublisher()
		: Node("DWA_SollspeedPublisher"), count_(0)
	{
		publisherSollspeed_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("soll_speed1", 10);
		timer_ = this->create_wall_timer(
			20ms, std::bind(&DWA_SollspeedPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto message = std_msgs::msg::Float32MultiArray();
		cout << "goal " << goal.transpose() << endl;

		if (planner.PLOT_ESTIMATE_TRAJ) {
			int vorplan_count = 0;

			planner.RESET_STATE = false;
			motor_speed0 = motor_ist;
			planner.TEMPORARY_GOAL_ARRIVED = false;
			if (traj_.cols() > 50) {
				MatrixXd traj_;
			}
			while (vorplan_count < 50) {
				vorplan = planner.dwa_control(motor_speed0, car_x0, goal, oblist);
				u_ = planner.speed_change(vorplan.u, "MOTOR_TO_PC");
				car_x0 = planner.motion(car_x0, u_, planner.dt);
				motor_speed0 = vorplan.u;
				//cout << "motor_speed0:" << motor_speed0 << endl;
				traj_.conservativeResize(traj_.rows(), traj_.cols() + 1);
				traj_.col(traj_.cols() - 1) = car_x0;
				vorplan_count++;
			}
			planner.RESET_STATE = true;
			cout << "traj_ size:" << traj_.cols() << "  vorplan count: " << vorplan_count << endl;
		}

		plan = planner.dwa_control(motor_ist, car_x, zwGoal, oblist);
		sollTrajB = planner.traj_W2B_transformation(lastPose, sollTrajW);
		cout << "soll traj:" << endl << sollTrajB << endl;
		zwGoal = sollTrajB.col(goal_i);
		cout << "zw goal i: " << goal_i << "----" << zwGoal.transpose() << endl;
		controlspeed = plan.u;

		/*if (!planner.RESET_STATE)
		{
			u_ = planner.speed_change(plan.u, "MOTOR_TO_PC");
			car_x = planner.motion(car_x, u_, planner.dt);
			traj_.conservativeResize(traj_.rows(), traj_.cols() + 1);
			traj_.col(traj_.cols() - 1) = car_x;
		}*/

		if (planner.TEMPORARY_GOAL_ARRIVED)
		{
			std::cout << "temporary goal arrived!" << endl;
			controlspeed << 0, 0;
			planner.TEMPORARY_GOAL_ARRIVED = false;
			goal_i++;
		}

		if (goal_i > sollTrajW.cols()) {
			std::cout << "final goal arrived!" << endl;
			controlspeed << 0, 0;
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

		subscriptionMaplist_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
			"/yf_camera/obList", 1, std::bind(&DWA_Subscriber::topicOblist_callback, this, _1));

		subscriptionGoal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/yf_camera/goal", 1, std::bind(&DWA_Subscriber::topicGoal_callback, this, _1));

		subscriptionPose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/yf_camera/Pose", 1, std::bind(&DWA_Subscriber::topicPose_callback, this, _1));
	}

private:
	void topicRealspeed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr speedmsg) const
	{
		auto RealSpeed = speedmsg->data;
		float motor_left = (RealSpeed[0] + RealSpeed[2]) / 2;
		float motor_right = (RealSpeed[4] + RealSpeed[6]) / 2;
		motor_ist << motor_left, motor_right;
	}

	void topicOblist_callback(const std_msgs::msg::Float32MultiArray::SharedPtr mapmsg) const
	{
		std::vector<float> v = mapmsg->data;
		try
		{
			float* v2 = new float[20000];
			v2 = v.data();
			Map<MatrixXf> ob(v2, 2, mapmsg->data.size() / 2);
			oblist = planner.remove_human_shape(ob.cast<double>(), goal);
		}
		catch (std::bad_alloc& ba)
		{
			std::cerr << "bad_alloc caught: " << ba.what() << '\n';
		}
	}

	void topicGoal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message) const
	{
		Goal temp_goal;
		if (!isnan(message->pose.position.x) && !isnan(message->pose.position.z)) {
			temp_goal << message->pose.position.x + 0.06, message->pose.position.z;
			goal = temp_goal;
		}
	}

	void topicPose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message) const
	{
		Pose pose_temp;
		double transx = message->pose.position.x;
		double transy = message->pose.position.y;
		double transz = message->pose.position.z;
		double rotx = message->pose.orientation.x;
		double roty = message->pose.orientation.y;
		double rotz = message->pose.orientation.z;

		transx = (double)round(transx * 1000) / 1000;
		transy = (double)round(transy * 1000) / 1000;
		transz = (double)round(transz * 1000) / 1000;
		rotx = (double)round(rotx * 1000) / 1000;
		roty = (double)round(roty * 1000) / 1000;
		rotz = (double)round(rotz * 1000) / 1000;

		/*bool transNan = isnan(transx) || isnan(transy) || isnan(transz);
		bool rotNan = isnan(rotx) || isnan(roty) || isnan(rotz);*/
		/*if (!(transNan || rotNan)){*/
		pose_temp << rotx, rotz, -roty, transx, transz, transy;  // TODO: CHECK IF ROT X AND Y IS REVERSERD
		lastPose = pose_temp;
		planner.pose_temp = pose_temp;
		if (!PoseInitialized) {
			planner.pose_bias = pose_temp;
			PoseInitialized = true;
		}
	}

	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionSpeed_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionMaplist_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriptionGoal_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriptionPose_;
};

class DWA_Parametrize : public rclcpp::Node
{
public:
	DWA_Parametrize()
		: Node("DWA_Parametrize")
	{
		this->declare_parameter<double>("set_goalx", 0.5);
		this->declare_parameter<double>("set_goaly", 1.0);
		this->declare_parameter<double>("max_speed", 3.0);
		this->declare_parameter<double>("min_speed", -0.3);
		this->declare_parameter<double>("max_yaw_rate", 540.0);
		this->declare_parameter<double>("max_accel", 3.0);
		this->declare_parameter<double>("max_delta_yaw_rate", 540.0);
		this->declare_parameter<double>("v_resolution", 0.15);
		this->declare_parameter<double>("yaw_rate_resolution", 15.0);
		this->declare_parameter<double>("min_wheel_speed", 180.0);
		this->declare_parameter<double>("dt", 0.2);
		this->declare_parameter<double>("predict_time", 2.0);
		this->declare_parameter<double>("to_goal_cost_gain", 0.3);
		this->declare_parameter<double>("obstacle_cost_gain", 0.4);
		this->declare_parameter<double>("speed_adjust_param", 0.8);
		this->declare_parameter<double>("speed_cost_gain_max", 1.6);
		this->declare_parameter<double>("speed_cost_gain_min", 0.1);

		this->declare_parameter<bool>("SHOW_ANIMATION", true);
		this->declare_parameter<bool>("GOAL_ARRIVAED", false);
		this->declare_parameter<bool>("RESET_STATE", true);
		this->declare_parameter<bool>("HUMAN_SHAPE", false);
		this->declare_parameter<bool>("MAP_TO_OBCOORD", false);
		this->declare_parameter<bool>("TRANSFORM_MAP", false);
		this->declare_parameter<bool>("MEASURE_TIME", false);
		this->declare_parameter<bool>("TEMPORARY_GOAL_ARRIVED", false);
		this->declare_parameter<bool>("PUBLISH_DWA_STATE", false);
		this->declare_parameter<bool>("SET_GOAL", false);
		this->declare_parameter<bool>("DEADZONE_CHECK", false);
		this->declare_parameter<bool>("PRINT_COST", false);
		this->declare_parameter<bool>("PLOT_ESTIMATE_TRAJ", false);
		timer_ = this->create_wall_timer(
			2000ms, std::bind(&DWA_Parametrize::respond, this));
	}
	void respond()
	{
		float setGoalx, setGoaly;
		this->get_parameter("set_goalx", setGoalx);
		this->get_parameter("set_goaly", setGoaly);
		this->get_parameter("max_speed", planner.max_speed);
		this->get_parameter("min_speed", planner.min_speed);
		this->get_parameter("max_yaw_rate", planner.max_yaw_rate);
		this->get_parameter("max_accel", planner.max_accel);
		this->get_parameter("max_delta_yaw_rate", planner.max_delta_yaw_rate);
		this->get_parameter("v_resolution", planner.v_resolution);
		this->get_parameter("yaw_rate_resolution", planner.yaw_rate_resolution);
		this->get_parameter("min_wheel_speed", planner.min_wheel_speed);
		this->get_parameter("dt", planner.dt);
		this->get_parameter("predict_time", planner.predict_time);
		this->get_parameter("to_goal_cost_gain", planner.to_goal_cost_gain);
		this->get_parameter("obstacle_cost_gain", planner.obstacle_cost_gain);
		this->get_parameter("speed_adjust_param", planner.speed_adjust_param);
		this->get_parameter("speed_cost_gain_max", planner.speed_cost_gain_max);
		this->get_parameter("speed_cost_gain_min", planner.speed_cost_gain_min);

		this->get_parameter("SHOW_ANIMATION", planner.SHOW_ANIMATION);
		this->get_parameter("GOAL_ARRIVAED", planner.GOAL_ARRIVAED);
		this->get_parameter("RESET_STATE", planner.RESET_STATE);
		this->get_parameter("HUMAN_SHAPE", planner.HUMAN_SHAPE);
		this->get_parameter("MAP_TO_OBCOORD", planner.MAP_TO_OBCOORD);
		this->get_parameter("MEASURE_TIME", planner.MEASURE_TIME);
		this->get_parameter("TRANSFORM_MAP", planner.TRANSFORM_MAP);
		this->get_parameter("TEMPORARY_GOAL_ARRIVED", planner.TEMPORARY_GOAL_ARRIVED);
		this->get_parameter("PUBLISH_DWA_STATE", planner.PUBLISH_DWA_STATE);
		this->get_parameter("DEADZONE_CHECK", planner.DEADZONE_CHECK);
		this->get_parameter("PRINT_COST", planner.PRINT_COST);
		this->get_parameter("PLOT_ESTIMATE_TRAJ", planner.PLOT_ESTIMATE_TRAJ);

		//cout<<"set param finished!"<<endl;
		planner.set_goal << setGoalx, setGoaly;
		planner.predict_time0 = planner.predict_time;
		//cout << "max_speed:" << planner.max_speed << endl;
	}
private:
	rclcpp::TimerBase::SharedPtr timer_;
};

class DWA_Visualizer : public rclcpp::Node
{
public:
	DWA_Visualizer()
		: Node("DWA_Visualizer"), count_(0)
	{
		publisherImg_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("dwa/Planner/compressed", 1);
		timer_ = this->create_wall_timer(
			50ms, std::bind(&DWA_Visualizer::timer_callback, this));
	}

private:
	cv_bridge::CvImage img_bridge;
	void timer_callback()
	{
		//  visualization
		if (planner.SHOW_ANIMATION)
		{
			//cout << "bis visulizer check" << endl;
			cv::Mat bg(figure_size, figure_size, CV_8UC3, cv::Scalar(255, 255, 255));

			// draw goal
			cv::circle(bg, cv_offset(goal(0) + 2.56, goal(1), bg.cols, bg.rows), int(scale_up * 0.8), cv::Scalar(255, 0, 0), int(figure_size / 200));

			// draw calibration 1m
			//cv::circle(bg, 0,0, 10, cv::Scalar(155, 155, 115), int(figure_size / 200));
			//cv::circle(bg, cv_offset(2.56, 1, bg.cols, bg.rows), int(scale_up0.5), cv::Scalar(155, 155, 115), int(figure_size / 200));

			// draw maximum detect range square
			cv::rectangle(bg, (cv_offset(0, 0, bg.cols, bg.rows)), (cv_offset(5, 5, bg.cols, bg.rows)), cv::Scalar(10, 205, 215), int(figure_size / 200));

			// draw all obstacle
			for (unsigned int j = 0; j < oblist.cols(); j++)
			{
				cv::circle(bg, cv_offset(oblist(0, j) + 2.56, oblist(1, j), bg.cols, bg.rows), int(figure_size / 200), cv::Scalar(0, 0, 0), -1);
			}

			// draw soll trajectory in bodyframe
			for (unsigned int j = 0; j < sollTrajB.cols(); j++)
			{
				cv::circle(bg, cv_offset(sollTrajB(0, j) + 2.56, sollTrajB(1, j), bg.cols, bg.rows), int(figure_size / 100), cv::Scalar(106, 208, 116), -1);
			}

			/*traj_.conservativeResize(traj_.rows(), 1);
			traj_ << 0., -0.3, PI / 2, 0, 0;*/
			// draw car all posible trajectory
			for (unsigned int i = 0; i < plan.all_traj.size(); i++)
			{
				for (unsigned int j = 0; j < plan.all_traj[i].cols(); j++)
				{
					cv::circle(bg, cv_offset(plan.all_traj[i](0, j) + 2.56, plan.all_traj[i](1, j), bg.cols, bg.rows), int(figure_size / 300), cv::Scalar(0, 255, 0), -1);
				}
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
			cv::line(bg, cv_offset(rec_pos(0, 0) + 2.56, rec_pos(0, 1)), cv_offset(rec_pos(1, 0) + 2.56, rec_pos(1, 1)), cv::Scalar(200, 0, 100), int(figure_size / 200));
			cv::line(bg, cv_offset(rec_pos(1, 0) + 2.56, rec_pos(1, 1)), cv_offset(rec_pos(2, 0) + 2.56, rec_pos(2, 1)), cv::Scalar(200, 0, 100), int(figure_size / 200));
			cv::line(bg, cv_offset(rec_pos(2, 0) + 2.56, rec_pos(2, 1)), cv_offset(rec_pos(3, 0) + 2.56, rec_pos(3, 1)), cv::Scalar(200, 0, 100), int(figure_size / 200));
			cv::line(bg, cv_offset(rec_pos(3, 0) + 2.56, rec_pos(3, 1)), cv_offset(rec_pos(0, 0) + 2.56, rec_pos(0, 1)), cv::Scalar(200, 0, 100), int(figure_size / 200));

			// draw car safe circle
			cv::circle(bg, cv_offset(car_x(0) + 2.56, car_x(1), bg.cols, bg.rows), int(scale_up * 0.5), cv::Scalar(200, 20, 20), 2);

			//// draw car heading arrow
			//cv::arrowedLine(bg, cv_offset(car_x(0) + 2.56, car_x(1), bg.cols, bg.rows),
			//	cv_offset(car_x(0) + 2.56 + std::cos(car_x(2)), car_x(1) + std::sin(car_x(2)), bg.cols, bg.rows),
			//	cv::Scalar(255, 0, 255),
			//	int(figure_size / 300));

			// draw car optimal trajectory
			for (unsigned int j = 0; j < plan.traj.cols(); j++)
			{
				cv::circle(bg, cv_offset(plan.traj(0, j) + 2.56, plan.traj(1, j), bg.cols, bg.rows), int(figure_size / 200), cv::Scalar(10, 10, 255), -1);
			}

			if ((car_x.head(2) - goal).norm() <= planner.robot_radius + 0.3)
			{
				cv::circle(bg, cv_offset(goal(0) + 2.56, goal(1), bg.cols, bg.rows), int(scale_up * 0.8), cv::Scalar(0, 205, 0), int(figure_size / 150));
				// draw reaching goal
				if (!planner.RESET_STATE) {
					for (unsigned int j = 0; j < traj_.cols(); j++)
					{
						cv::circle(bg, cv_offset(traj_(0, j), traj_(1, j), bg.cols, bg.rows),
							int(figure_size / 200), cv::Scalar(0, 0, 255), -1);
					}
				}

			}

			if (planner.PLOT_ESTIMATE_TRAJ) {
				traj_.conservativeResize(traj_.rows(), 50);
				cout << "traj_ size in visualizer: " << traj_.cols() << endl;
				for (unsigned int j = 0; j < traj_.cols(); j++)
				{
					cv::circle(bg, cv_offset(traj_(0, j) + 2.56, traj_(1, j), bg.cols, bg.rows), int(figure_size / 200), cv::Scalar(100, 0, 10), -1);
				}
			}

			//cout << "reach before publish dwa state" << endl;
			if (planner.PUBLISH_DWA_STATE) {
				//	cout<<"reach publish dwa state"<<endl;
				//	cv::Mat outImg;
				//	//cv::resize(bg, outImg, cv::Size(200, 200), 0, 0, cv::INTER_AREA);
				auto message = sensor_msgs::msg::CompressedImage();
				std_msgs::msg::Header header; // empty header
				img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, bg);
				img_bridge.toCompressedImageMsg(message); // from cv_bridge to sensor_msgs::Image
				publisherImg_->publish(message);

			}
			else {
				cv::imshow("dwa", bg);
				cv::waitKey(10);
				std::string int_count = std::to_string(count0);
				cv::imwrite("./pngs/" + std::string(5 - int_count.length(), '0').append(int_count) + ".png", bg);
			}
		}
	}
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage >::SharedPtr publisherImg_;
	rclcpp::TimerBase::SharedPtr timer_;
	size_t count_;
};

int main(int argc, char** argv)
{
	//planner.readJsonFromFile();
	//  init rclcpp
	rclcpp::init(argc, argv);
	// make multithreaded executor
	rclcpp::executors::MultiThreadedExecutor executor;

	auto speedpublisher = std::make_shared<DWA_SollspeedPublisher>();
	auto dwasubscriber = std::make_shared<DWA_Subscriber>();
	auto visualizer = std::make_shared<DWA_Visualizer>();
	auto parameter = std::make_shared<DWA_Parametrize>();
	//cout << "max_speed:" << planner.max_speed << endl;
	car_x << 0., -0.3, PI / 2, 0, 0;
	car_x0 << 0., -0.3, PI / 2, 0, 0;
	goal << 0., 0.1;
	zwGoal << 0., 0.1;
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
	sollTraj0 << 1.0, 0.0,
		1.5, 0.0,
		1.5, 1.0;
	/*1.5, 1.5,
	1.5, 2.0,
	1.5, 3.0,
	1.0, 3.0,
	0.0, 3.0;*/
	/*cout<<"soll traj size:"<< sollTraj0.size()<<endl;
	cout << "soll traj cols:" << sollTraj0.cols() << "soll traj rows:" << sollTraj0.rows() << endl;*/
	sollTrajW = sollTraj0.transpose();
	/*double* tempv2 = sollTraj0.data();
	Map<MatrixXd> sollTraj(tempv2, 2, sollTraj0.size() / 2);*/
	/*int solltrajsize = sollTraj0.size() / 2;
	sollTraj = sollTraj0.conservativeResize(2, solltrajsize);*/
	motor_ist << 0, 0;
	std::cout << "start dwa planning!" << endl;

	if (planner.SHOW_ANIMATION)
	{
		cv::namedWindow("dwa", cv::WINDOW_NORMAL);
		cv::resizeWindow("dwa", 500, 500);
	}

	// add to the executor
	executor.add_node(speedpublisher);
	executor.add_node(dwasubscriber);
	executor.add_node(visualizer);
	executor.add_node(parameter);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}