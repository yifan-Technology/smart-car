#include<iostream>
#include<vector>
#include<cmath>
#include<ctime>
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

//#include "include/yaml-cpp/yaml.h"
#include "DWA_Planner.h"
#define PI 3.141592653


using namespace std;
using namespace Eigen;
//using State = Matrix<double, 5, 1 >;
//using Control = Matrix<double, 2, 1>;
//using Goal = Matrix<double, 2, 1>;
//using Window = Matrix<double, 4, 1>;
//using All_Traj = Array<MatrixXd, 1, 1>;
//struct DWA_result { Control u; MatrixXd traj; All_Traj all_traj; };

namespace dwa_planner {


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


	State DWA::koordinaten_transfomation(Control wheel_speed, double theta) {
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

	State DWA::motion(State x, Control u, double dt) {
		x(2) += x(4) * dt;
		State x_tr = koordinaten_transfomation(u, x(2));
		x(0) += x_tr(0) * dt;
		x(1) += x_tr(1) * dt;
		x(3) = x_tr(3);
		x(4) = x_tr(4);
		return x;
	};

	Control  DWA::speed_change(Control u_in, string mode) {
		double speed_gain = 60 / (2 * PI) * (3591 / 187);
		if (mode == "MOTOR_TO_PC") {
			return u_in / speed_gain;
		}
		else if (mode == "PC_TO_MOTOR") {
			return u_in * speed_gain;
		}
	}

	MatrixXd DWA::obmap2coordinaten(MatrixXd obmap, double res) {
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

	double DWA::dynamic_speed_cost(double dmin) {
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

	Window DWA::calc_dynamic_window(State state) {
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

	MatrixXd DWA::predict_trajectory(State x_init, double v_soll, double omega_soll) {
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

	double DWA::calc_to_goal_cost(MatrixXd trajectory, Goal goal) {
		/*"""
		calc to goal cost with angle difference
		"""*/
		double dx = goal(0) - trajectory(0, trajectory.cols() - 1);
		double dy = goal(1) - trajectory(1, trajectory.cols() - 1);
		double	error_angle = atan2(dy, dx);
		return abs(error_angle - trajectory(2, trajectory.cols() - 1));
		/*cost = abs(np.arctan2(sin(cost_angle), cos(cost_angle)))*/
	}

	double DWA::calc_obstacle_cost(MatrixXd trajectory, MatrixXd obstacle) {
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

	DWA_result DWA::calc_control_and_trajectory(State state, Window dw, Goal goal, MatrixXd obstacle) {
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

	DWA_result DWA::dwa_control(Control motor_ist, State x_pre, Goal zw_goal, MatrixXd ob_list) {
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



