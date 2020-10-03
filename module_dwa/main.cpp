//******************************************************************
//yifan DWA
//******************************************************************

#include<iostream>
#include<ctime>
#include<Eigen/Dense>
#include<limits>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

//#include "DWA_Planner.h"
 //#include "include/yaml-cpp/yaml.h"
#define PI 3.141592653
// 产生一个double类型的NAN


using namespace std;
using namespace Eigen;
using State = Matrix<double, 5, 1 >;
using Control = Matrix<double, 2, 1>;
using Goal = Matrix<double, 2, 1>;
using Window = Matrix<double, 4, 1>;
using All_Traj = std::vector<MatrixXd>;
struct DWA_result { Control u; MatrixXd traj; All_Traj all_traj; };

cv::Point2i cv_offset(
	double x, double y, int image_width = 1000, int image_height = 1000) {
	cv::Point2i output;
	output.x = int(x * 50) + image_width / 5;
	output.y = image_height - int(y * 50) - image_height / 5;
	return output;
};

class DWA {

public:
	//YAML::Node car_state = YAML::LoadFile("../dwa_config.yaml");
	//double max_speed = car_state['max_speed'].as<double>; // 0.8   [m / s] 
	//double min_speed = car_state['min_speed'].as<double>; //  - 0.8  [m / s]
	//double max_yaw_rate = car_state['max_yaw_rate'] * PI / 180;  // 180.0 * PI / 180.0   [rad / s]
	//double max_accel = car_state['max_accel'].as<double>; // 4.0   [m / ss]
	//double max_delta_yaw_rate = car_state['max_delta_yaw_rate'] * PI / 180;  // 360.0 * PI / 180.0   [rad / ss]
	//double v_resolution = car_state['v_resolution'].as<double>; // 0.2   [m / s]
	//double yaw_rate_resolution = car_state['yaw_rate_resolution'] * PI / 180;  // 15. * PI / 180.0   [rad / s]
	//double min_wheel_speed = car_state['min_wheel_speed'].as<double>; //100 [rpm]
	//double dt = car_state['dt'].as<double>; // 0.2  [s] Time tick for motion prediction
	//double predict_time = car_state['predict_time'].as<double>; // 0.8   [s]  less and more flexible
	//double to_goal_cost_gain = car_state['to_goal_cost_gain'].as<double>; // 0.16
	//double obstacle_cost_gain = car_state['obstacle_cost_gain'].as<double>;//  0.6
	//double speed_adjust_param = car_state['speed_adjust_param'];
	//double speed_cost_gain_max = car_state['speed_cost_gain_max'];
	//double speed_cost_gain_min = car_state['speed_cost_gain_min'];
	//double dist_to_goal = car_state['dist_to_goal'].as<double>; // 1e10
	//bool SHOW_ANIMATION = flag_state['SHOW_ANIMATION'].as<bool>; // true
	//bool GOAL_ARRIVAED = flag_state['GOAL_ARRIVAED'].as<bool>; // false
	//bool RESET_STATE = flag_state['RESET_STATE'].as<bool>; // false
	//bool HUMAN_SHAPE = flag_state['HUMAN_SHAPE'].as<bool>; // false
	//bool MAP_TO_OBCOORD = flag_state['MAP_TO_OBCOORD'].as<bool>; // true
	//bool MEASURE_TIME = flag_state['MEASURE_TIME'].as<bool>; // false
	//bool TEMPORARY_GOAL_ARRIVED = false;
	double max_speed = 1.8; // 0.8   [m / s] 
	double min_speed = -1.8; //  - 0.8  [m / s]
	double max_yaw_rate = 720 * PI / 180;  // 180.0 * PI / 180.0   [rad / s]
	double max_accel = 4; // 4.0   [m / ss]
	double max_delta_yaw_rate = 360 * PI / 180;  // 360.0 * PI / 180.0   [rad / ss]
	double v_resolution = 0.1; // 0.2   [m / s]
	double yaw_rate_resolution = 10 * PI / 180;  // 15. * PI / 180.0   [rad / s]
	double min_wheel_speed = 100; //100 [rpm]
	double dt = 0.1; // 0.2  [s] Time tick for motion prediction
	double predict_time = 1.5; // 0.8   [s]  less and more flexible

	double to_goal_cost_gain = 0.8; // 0.16
	double obstacle_cost_gain = 0.3;//  0.6
	double speed_adjust_param = 0.9;
	double speed_cost_gain_max = 5;
	double speed_cost_gain_min = 0.1;
	double dist_to_goal = 1e10; // 1e10

	bool SHOW_ANIMATION = true; // true
	bool GOAL_ARRIVAED = false; // false
	bool RESET_STATE = false; // false
	bool HUMAN_SHAPE = false; // false
	bool MAP_TO_OBCOORD = true; // true
	bool MEASURE_TIME = true; // false
	bool TEMPORARY_GOAL_ARRIVED = false;
	double m = 5;
	double g = 9.80665;
	double mu = 0.01;
	double x0 = 0.01;
	double wheel_len_dist = 0.475; //car wheelcenter len distance
	double wheel_quer_dist = 0.460;
	double car_h = 0.4;
	double wheel_radius = 0.095;

	/*State x_0;
	x_0 << 2.5, -0.3, (PI / 2), 0.0, 0.0;*/

	double robot_radius = 0.5;  // [m] for collision check
	double robot_length = 0.661 + 0.1;  // [m] for collision check
	double robot_width = 0.504 + 0.1;  // [m] for collision check

	State koordinaten_transfomation(Control wheel_speed, double theta) {
		double omega_l = wheel_speed[0];
		double omega_r = wheel_speed[1];

		double vx = wheel_radius * (omega_l + omega_r) / 2;
		double omega = wheel_radius * (-omega_l + omega_r) / wheel_quer_dist;
		double vy = -x0 * omega;

		double dX = cos(theta) * vx - sin(theta) * vy;
		double dY = sin(theta) * vx + cos(theta) * vy;

		State state_transformed;
		state_transformed << dX, dY, theta, vx, omega;

		return state_transformed;
	}

	State motion(State x, Control u, double dt) {
		x(2) += x(4) * dt;
		State x_tr = koordinaten_transfomation(u, x(2));
		x(0) += x_tr(0) * dt;
		x(1) += x_tr(1) * dt;
		x(3) = x_tr(3);
		x(4) = x_tr(4);
		return x;
	};

	Control  speed_change(Control u_in, string mode) {
		double speed_gain = 60 / (2 * PI) * (3591 / 187);
		if (mode == "MOTOR_TO_PC") {
			return u_in / speed_gain;
		}
		else if (mode == "PC_TO_MOTOR") {
			return u_in * speed_gain;
		}
	}

	MatrixXd obmap2coordinaten(MatrixXd obmap, double res) {
		MatrixXd loc(2, 1);
		for (int i = 0; i < obmap.rows(); i++) {
			for (int j = 0; j < obmap.cols(); j++) {
				if (obmap(i, j) == 1) {
					loc.col(loc.cols() - 1) << i, j;
					loc.conservativeResize(loc.rows(), loc.cols() + 1);
				}
			}
		}
		loc.conservativeResize(loc.rows(), loc.cols() - 1);
		return loc * res;
	}

	double dynamic_speed_cost(double dmin) {
		double ds = speed_adjust_param * max_speed / max_accel;
		//cout<<"ds"<<ds<<endl;
		if (dmin <= ds) {
			return speed_cost_gain_min + (speed_cost_gain_max - speed_cost_gain_min) * pow(dmin / ds, 1.8);
		}
		else {
			return speed_cost_gain_max;
		}

	}

	/*Goal goal_inverse_transforamtion(State x, Goal goal) {
		double theta = x(2) - PI / 2;
		Matrix2d Rot_i2b;
		Rot_i2b << cos(theta), sin(theta), -sin(theta), cos(theta);

		Goal goal_i = goal - x.topRows<2>();
		return Rot_i2b * goal_i + x_0.topRows<2>();
	}

	MatrixXd ob_inverse_transforamtion(State x, MatrixXd ob_cood) {
		double theta = x(2) - PI / 2;
		Matrix2d Rot_i2b;
		Rot_i2b << cos(theta), sin(theta), -sin(theta), cos(theta);

		return Rot_i2b * (ob_cood - x.topRows<2>()) + x_0.topRows<2>().replicate(1, ob_cood.cols());
	}*/

	Window calc_dynamic_window(State state) {
		/*"""
		calculation dynamic window based on current state x
		"""*/
		//Dynamic window from robot specification

		Window Vs;
		Vs << min_speed, max_speed, -max_yaw_rate, max_yaw_rate;

		//cout<<"Vs"<<endl<<Vs<<endl;
		// Dynamic window from motion model
		Window Vd;
		Vd << (state(3) - max_accel * dt),
			(state(3) + max_accel * dt),
			(state(4) - max_delta_yaw_rate * dt),
			(state(4) + max_delta_yaw_rate * dt);

		//  [v_min, v_max, yaw_rate_min, yaw_rate_max]
		Window dw;
		dw << max(Vs(0), Vd(0)),
			min(Vs(1), Vd(1)),
			max(Vs(2), Vd(2)),
			min(Vs(3), Vd(3));

		return dw;
	}

	MatrixXd predict_trajectory(State x_init, double v_soll, double omega_soll) {
		/*"""
	predict trajectory with an input
	"""*/
		Control speed_soll;
		speed_soll << v_soll, omega_soll;
		State x = x_init;
		MatrixXd trajectory(5, 1);
		trajectory << x.array();

		double count_time = 0;
		Matrix2d vtrans;
		vtrans << 1, -wheel_quer_dist / 2, 1, wheel_quer_dist / 2;
		Control wheel_speed = vtrans * speed_soll / wheel_radius;

		while (count_time <= predict_time) {
			/*now is 2s;*/
			x = motion(x, wheel_speed, dt);
			trajectory.conservativeResize(trajectory.rows(), trajectory.cols() + 1);
			trajectory.col(trajectory.cols() - 1) = x;
			count_time += dt;
		}
		return trajectory;
	}

	double calc_to_goal_cost(MatrixXd trajectory, Goal goal) {
		/*"""
		calc to goal cost with angle difference
		"""*/
		double dx = goal(0) - trajectory(0, trajectory.cols() - 1);
		double dy = goal(1) - trajectory(1, trajectory.cols() - 1);
		double	error_angle = atan2(dy, dx);
		return abs(error_angle - trajectory(2, trajectory.cols() - 1));
		/*cost = abs(np.arctan2(sin(cost_angle), cos(cost_angle)))*/
	}

	double calc_obstacle_cost(MatrixXd trajectory, MatrixXd obstacle) {
		/*"""
		calc obstacle cost inf : collision
		"""*/

		MatrixXd ox = obstacle.row(0).replicate(trajectory.cols(), 1);  // (15, )
		MatrixXd oy = obstacle.row(1).replicate(trajectory.cols(), 1);
		MatrixXd tr_x = trajectory.row(0).transpose().replicate(1, obstacle.cols());
		MatrixXd tr_y = trajectory.row(1).transpose().replicate(1, obstacle.cols());

		// 因为障碍物和轨迹点数量不等, 所以增加一列来表示
		MatrixXd rho_square = (tr_x - ox).array().square() + (tr_y - oy).array().square();
		/*cout<<"rho"<<rho_square<<endl;*/
		if (rho_square.minCoeff() <= robot_radius * robot_radius) {
			return INFINITY;
		}
		else {
			//cout << "obst_cost:" << 1.0 / (rho_square).minCoeff() << endl;
			return 1.0 / (rho_square).minCoeff();
		}
	}

	DWA_result calc_control_and_trajectory(State state, Window dw, Goal goal, MatrixXd obstacle) {
		/*"""
		calculation final input with dynamic window
		"""*/
		All_Traj all_traj;
		State x_init = state;
		double min_cost = INFINITY;
		Control best_u;
		best_u << 0, 0;
		MatrixXd best_traj = state;
		//cout<<"best traj "<<best_traj<<endl;
		double dmin, ob_cost;
		DWA_result Result;

		//evaluate all trajectory with sampled input in dynamic window;
		for (double v = dw(0); v < dw(1); v += v_resolution) {
			for (double omega = dw(2); omega < dw(3); omega += yaw_rate_resolution) {
				MatrixXd predict_traj = predict_trajectory(x_init, v, omega);
				all_traj.push_back(predict_traj);
				//cout<<"all traj"<<predict_traj<<endl;
				//calc cost;
				double to_goal_cost = to_goal_cost_gain * calc_to_goal_cost(predict_traj, goal);
				double dist_square = calc_obstacle_cost(predict_traj, obstacle);
				/*cout<<"to_goal_cost"<<to_goal_cost<<endl;*/
				//cout<<"dist square "<<dist_square<<endl;
				if (dist_square != INFINITY) {
					ob_cost = obstacle_cost_gain * dist_square;
					dmin = 1 / sqrt(dist_square);
					/*cout<<"ob_cost"<<ob_cost<<endl;
					cout<<"dmin "<<dmin<<endl;*/
				}
				else {
					//cout<<"nan appear"<<INFINITY<<endl;
					ob_cost = INFINITY;
					dmin = 0;
				}
				//cout<<"dyn speed gain "<<dyn_gain<<endl;
				//cout<< "traj sp "<<traj_sp<<endl;
				double speed_cost = dynamic_speed_cost(dmin) * (max_speed - predict_traj(3, predict_traj.cols() - 1));
				//cout<<"speed cost "<<speed_cost<<endl;
				double final_cost = to_goal_cost + speed_cost + ob_cost;
				//cout<< "final_cost "<<final_cost<<endl;

				/* search minimum trajectory*/
				if (min_cost >= final_cost) {
					min_cost = final_cost;
					best_u << v, omega;
					//cout<<"best u"<<best_u<<endl;
					best_traj = predict_traj;
				}
			}
			//cout<<"v "<<v<<endl;
		}
		//cout<<"best u "<<best_u<<endl;
		Result.u = best_u;
		Result.traj = best_traj;
		Result.all_traj = all_traj;
		return Result;
	}

	DWA_result dwa_control(Control motor_ist, State x_pre, Goal zw_goal, MatrixXd ob_list) {
		/*"""
		Dynamic Window Approach control
		"""*/
		State temp_x = x_pre;
		Control u_ist = speed_change(motor_ist, "MOTOR_TO_PC");
		State x_trans = koordinaten_transfomation(u_ist, temp_x(2));
		//cout<<"x_trans:"<<x_trans<<endl;
		if (RESET_STATE) {
			temp_x.topRows<3>() << 2.5, -0.3, PI / 2;
			temp_x.bottomRows<2>() = x_trans.bottomRows<2>();
		}

		Window dw = calc_dynamic_window(temp_x);
		//cout<<"dw"<<dw<<endl;
		DWA_result dwa_result = calc_control_and_trajectory(temp_x, dw, zw_goal, ob_list);

		Control u_cal = dwa_result.u;
		/*	cout<<"u_cal:"<<u_cal<<endl;*/
		Matrix2d vtrans;
		vtrans << 1, -wheel_quer_dist, 1, wheel_quer_dist;
		Control u_soll = vtrans * u_cal / wheel_radius;
		Control motor_soll = speed_change(u_soll, "PC_TO_MOTOR");

		/*check reaching goal*/

		double dist_to_goal = (temp_x.topRows<2>() - zw_goal).norm();  /* generate u = wheel_speed_soll*/
		/*cout << "dist to goal:" << dist_to_goal << endl;*/
		if ((motor_soll.norm() < 1.414 * min_wheel_speed) && (dist_to_goal >= robot_radius)) {
			motor_soll = min_wheel_speed * motor_soll.array().sign();
			cout << "deadzone checked" << endl;
		}

		if (dist_to_goal <= robot_radius) {
			TEMPORARY_GOAL_ARRIVED = true;
		}

		//State x_next = motion(x_pre, u_ist, dt);
		dwa_result.u = motor_soll;

		return  dwa_result;
	}
};
void plot_robot(State x, cv::Mat bg, DWA planner) {
	Matrix2d diag_pos;
	diag_pos << x(0) - planner.robot_width / 2, x(0) + planner.robot_width / 2,
		x(1) - planner.robot_length / 2, x(1) + planner.robot_length / 2;
	/*cout << "diag_pos " << endl << diag_pos << endl;*/
	Matrix2d rot_bot;
	rot_bot << cos(x(2)), -sin(x(2)), sin(x(2)), cos(x(2));
	MatrixXd rec_pos = rot_bot * diag_pos;
	/*		cout << "rec_pos " << endl << rec_pos << endl;*/
	cv::rectangle(bg, cv_offset(diag_pos(0, 0), diag_pos(1, 0), bg.cols, bg.rows), cv_offset(diag_pos(0, 1), diag_pos(1, 1), bg.cols, bg.rows), cv::Scalar(0, 200, 0), 10);

}

int main() {

	DWA planner;
	//cout << "max_speed:" << planner.max_speed << endl;
	State x;
	x << 2.5, -0.3, PI / 2, 0, 0;
	Goal goal;
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
	MatrixXd oblist = obin.transpose();
	//cout << "oblist" << endl << oblist << endl;
	Control motor_ist, u_;
	motor_ist << 0, 0;
	MatrixXd traj_(5, 1);
	DWA_result plan;
	bool terminal = false;
	float figure_size = 1000;
	float scale_up = 50;

	cv::namedWindow("dwa", cv::WINDOW_NORMAL);
	cv::resizeWindow("dwa", 720, 720);
	std::cout << "start dwa planning!" << endl;
	int count = 0;


	while (1) {
		clock_t start = clock();
		/*if (cv::waitKey(100) == 27)
			break;*/
			//cout<<"obst: "<<oblist<<endl;
		plan = planner.dwa_control(motor_ist, x, goal, oblist);
		u_ = planner.speed_change(plan.u, "MOTOR_TO_PC");
		/*cout << "rpm speed:" << endl << u_ * planner.wheel_radius << endl;*/
		x = planner.motion(x, u_, planner.dt);
		//cout<<"state:"<<endl<<x<<endl;
		traj_.conservativeResize(traj_.rows(), traj_.cols() + 1);
		traj_.col(traj_.cols() - 1) = x;

		////// visualization
		if (planner.SHOW_ANIMATION) {

			cv::Mat bg(figure_size, figure_size, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::circle(bg, cv_offset(goal(0), goal(1), bg.cols, bg.rows),	20, cv::Scalar(255, 0, 0), int(figure_size/200));

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
				terminal = true;
				for (unsigned int j = 0; j < traj_.cols(); j++) {
					cv::circle(bg, cv_offset(traj_(0, j), traj_(1, j), bg.cols, bg.rows),
						int(figure_size / 200), cv::Scalar(0, 0, 255), -1);
				}
			}
			cv::imshow("dwa", bg);
			cv::waitKey(5);
			
			std::string int_count = std::to_string(count);
			cv::imwrite("./pngs/" + std::string(5 - int_count.length(), '0').append(int_count) + ".png", bg);

		}
		if (planner.MEASURE_TIME) {
			clock_t end = clock();
			double elapsed_time = (double(end) - double(start)) / (CLOCKS_PER_SEC);
			std::cout << "FPS is:" << 1 / elapsed_time << endl;
		}

		if (planner.TEMPORARY_GOAL_ARRIVED) {
			terminal = true;
			std::cout << "goal arrived!" << endl;
			planner.TEMPORARY_GOAL_ARRIVED=false;
			break;
		}
	}

	//std::cout << "state: " << fixed << setprecision(2) << traj_ << endl;
	return 0;
}