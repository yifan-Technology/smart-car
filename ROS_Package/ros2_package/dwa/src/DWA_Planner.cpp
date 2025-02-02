#include<vector>
#include<cmath>
#include<ctime>
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <jsoncpp/json/json.h>

#include "DWA_Planner.h"

//#define PI 3.1415926535
using namespace std;
using namespace Eigen;

namespace dwa_planner {

	void DWA::readJsonFromFile() {

		std::ifstream ifs;
		ifs.open("/home/yf/dev_ws/src/dwa/src/dwa_config.json");
		std::stringstream buffer;
		buffer << ifs.rdbuf();
		ifs.close();

		auto str = buffer.str();
		Json::Reader reader;
		Json::Value value;
		if (reader.parse(str, value)) {
			//   auto all_member = value.getMemberNames();
			//   for (auto member : all_member) {
			//         std::cout << member << std::endl;}
			set_goal << value["Car_State"]["set_goalx"].asFloat(), value["Car_State"]["set_goaly"].asFloat();
			max_speed = value["Car_State"]["max_speed"].asFloat(); // 0.8   [m / s] 
			//std::cout << "max_speed:" << max_speed << std::endl;
			//std::cout << "max speed's type:" << value["Car_State"]["max_speed"].type() << std::endl;
			min_speed = value["Car_State"]["min_speed"].asFloat(); //  - 0.8  [m / s]
			max_yaw_rate = value["Car_State"]["max_yaw_rate"].asFloat();  // 180.0 * PI / 180.0   [rad / s]
			max_accel = value["Car_State"]["max_accel"].asFloat(); // 4.0   [m / ss]
			max_delta_yaw_rate = value["Car_State"]["max_delta_yaw_rate"].asFloat();  // 360.0 * PI / 180.0   [rad / ss]
			v_resolution = value["Car_State"]["v_resolution"].asFloat(); // 0.2   [m / s]
			yaw_rate_resolution = value["Car_State"]["yaw_rate_resolution"].asFloat();  // 15. * PI / 180.0   [rad / s]
			min_wheel_speed = value["Car_State"]["min_wheel_speed"].asFloat(); //100 [rpm]
			dt = value["Car_State"]["dt"].asFloat(); // 0.2  [s] Time tick for motion prediction
			predict_time = value["Car_State"]["predict_time"].asFloat(); // 0.8   [s]  less and more flexible
			to_goal_cost_gain = value["Car_State"]["to_goal_cost_gain"].asFloat(); // 0.16
			obstacle_cost_gain = value["Car_State"]["obstacle_cost_gain"].asFloat();//  0.6
			speed_adjust_param = value["Car_State"]["speed_adjust_param"].asFloat();
			speed_cost_gain_max = value["Car_State"]["speed_cost_gain_max"].asFloat();
			speed_cost_gain_min = value["Car_State"]["speed_cost_gain_min"].asFloat();
			dist_to_goal = value["Car_State"]["dist_to_goal"].asFloat(); // 1e10
			SHOW_ANIMATION = value["Flag_State"]["SHOW_ANIMATION"].asBool(); // true
			GOAL_ARRIVAED = value["Flag_State"]["GOAL_ARRIVAED"].asBool(); // false
			RESET_STATE = value["Flag_State"]["RESET_STATE"].asBool(); // false
			HUMAN_SHAPE = value["Flag_State"]["HUMAN_SHAPE"].asBool(); // false
			MAP_TO_OBCOORD = value["Flag_State"]["MAP_TO_OBCOORD"].asBool(); // true
			MEASURE_TIME = value["Flag_State"]["MEASURE_TIME"].asBool(); // false
			TEMPORARY_GOAL_ARRIVED = value["Flag_State"]["TEMPORARY_GOAL_ARRIVED"].asBool();
			PUBLISH_DWA_STATE = value["Flag_State"]["PUBLISH_DWA_STATE"].asBool();

			value.clear();
		}
		else {
			cout << "cannot read json" << endl;
		}
	}

	double DWA::degree2radian(double degree) {
		return degree * PI / 180;
	}
	double DWA::RandInnovation(double fMin, double fMax)
	{
		double f = (double)rand() / RAND_MAX;
		return fMin + f * (fMax - fMin);
	}

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
	}

	Control  DWA::speed_change(Control u_in, string mode) {
		double speed_gain = 60 / (2 * PI) * (3591 / 187);
		if (mode == "MOTOR_TO_PC") {
			return u_in / speed_gain;
		}
		else if (mode == "PC_TO_MOTOR") {
			return u_in * speed_gain;
		}
		else {
			return u_in;
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

	MatrixXd DWA::remove_human_shape(MatrixXd ob_list, Goal target) {
		MatrixXd reduced_oblist(2, 1);
		for (int i = 0; i < ob_list.cols(); i++) {
			if ((target - ob_list.col(i)).norm() > 0.3) {
				reduced_oblist.col(reduced_oblist.cols() - 1) << ob_list(0, i), ob_list(1, i);
				reduced_oblist.conservativeResize(2, reduced_oblist.cols() + 1);
			}
		}
		reduced_oblist.conservativeResize(2, reduced_oblist.cols() - 1);
		return reduced_oblist;
	}

	double DWA::dynamic_speed_cost(double dmin) {
		double ds = speed_adjust_param * max_speed / max_accel; // bremse distance
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
		double max_yaw_rate_r = degree2radian(max_yaw_rate);
		double max_delta_yaw_rate_r = degree2radian(max_delta_yaw_rate);
		Window Vs;
		Vs << min_speed, max_speed, -max_yaw_rate_r, max_yaw_rate_r;

		// Dynamic window from motion model
		Window Vd;
		Vd << (state(3) - max_accel * dt),
			(state(3) + max_accel * dt),
			(state(4) - max_delta_yaw_rate_r * dt),
			(state(4) + max_delta_yaw_rate_r * dt);

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

		// calculate distance of every (x,y) between traj and obstacles
		MatrixXd rho_square = (tr_x - ox).array().square() + (tr_y - oy).array().square();

		if (rho_square.minCoeff() <= robot_radius * robot_radius) {
			return INFINITY;
		}
		else {
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
		double dmin, ob_cost;
		DWA_result Result;
		double yaw_rate_resolution_r = degree2radian(yaw_rate_resolution);
		MatrixXd cost_list(6, 1);
		double turnning_sign;
		vector<double> v_temp, omega_temp, v_range, omega_range, v_range0, omega_range0;

		// add less choice for turning
		v_range0.push_back(0);
		v_range0.push_back(0.001);

		omega_range0.push_back(0.5 * PI);
		omega_range0.push_back(-0.5 * PI);
		omega_range0.push_back(PI);
		omega_range0.push_back(-PI);
		omega_range0.push_back(1.5 * PI);
		omega_range0.push_back(-1.5 * PI);

		// prefer turning around
		double goal_theta = atan2(goal(1), goal(0)) * 180 / PI;
		bool SAFE_TURNING = true;

		for (int i = 0; i < obstacle.cols(); i++) {
			if (obstacle.col(i).norm() < 1.0) {
				SAFE_TURNING = false;
				break;
			}
		}

		if ((goal_theta > 135 || goal_theta < 45) && SAFE_TURNING) {
			v_temp = v_range0;
			omega_temp = omega_range0;
		}
		else {
			// remove small speed deadzone
			for (double v = dw(0); v < dw(1); v += v_resolution) {
				if (abs(v) > 0.15) {
					v_range.push_back(v);
				}
			}
			//v_range.push_back(0.);
			for (double omega = dw(2); omega < dw(3); omega += yaw_rate_resolution_r) {
				if (abs(omega) > 0.25) {
					omega_range.push_back(omega);
				}
			}
			omega_range.push_back(0.); // for straight heading

			v_temp = v_range;
			omega_temp = omega_range;
		}

		for (auto v : v_temp) {
			for (auto omega : omega_temp) {
				MatrixXd predict_traj = predict_trajectory(x_init, v, omega);
				all_traj.push_back(predict_traj);

				//calc cost;
				double to_goal_cost = to_goal_cost_gain * calc_to_goal_cost(predict_traj, goal);
				double dist_square = calc_obstacle_cost(predict_traj, obstacle);

				if (dist_square != INFINITY) {
					ob_cost = obstacle_cost_gain * dist_square;
					dmin = 1 / sqrt(dist_square);
				}
				else {
					ob_cost = INFINITY;
					dmin = 0;
				}

				double speed_cost = dynamic_speed_cost(dmin) * (max_speed - predict_traj(3, predict_traj.cols() - 1));
				double final_cost = to_goal_cost + speed_cost + ob_cost;

				if (PRINT_COST) {
					cost_list << v, omega, to_goal_cost, speed_cost, ob_cost, final_cost;
					cout << cost_list.transpose() << endl;
				}

				/* search minimum trajectory*/
				if (min_cost >= final_cost) {
					min_cost = final_cost;
					best_u << v, omega;
					best_traj = predict_traj;
				}
			}
		}
		// random innovation retreat strategy
		if (min_cost == INFINITY) {
			double v_innovate = RandInnovation(-0.2, -0.1);
			if (goal(0) < 0) {
				turnning_sign = 1;
			}
			else {
				turnning_sign = -1;
			}
			double omega_innovate = turnning_sign * RandInnovation(PI / 4, PI / 2);
			best_u << v_innovate, omega_innovate;
			EMERGENCY_STOP = true;
			cout<<"emergency u: "<<best_u.transpose()<<endl;
		}
		else {
			EMERGENCY_STOP = false;
		}
		Result.u = best_u;
		Result.traj = best_traj;
		Result.all_traj = all_traj;

		if (PRINT_COST) {
			cout << "##############################################################" << endl;
			cout << "***************chosen speed :" << best_u.transpose() << "******************" << endl;
		}
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
			temp_x.topRows<3>() << 0, -0.3, PI / 2;
			temp_x.bottomRows<2>() = u_ist;
		}

		Window dw = calc_dynamic_window(temp_x);
		DWA_result dwa_result = calc_control_and_trajectory(temp_x, dw, zw_goal, ob_list);
		Control u_cal = dwa_result.u;
		Matrix2d vtrans;
		vtrans << 1, -wheel_quer_dist / 2, 1, wheel_quer_dist / 2;
		Control u_soll = vtrans * u_cal / wheel_radius;
		Control motor_soll = speed_change(u_soll, "PC_TO_MOTOR");

		/*check reaching goal*/
		double dist_to_goal = (temp_x.topRows<2>() - zw_goal).norm();  /* generate u = wheel_speed_soll*/

		//if ((motor_soll.norm() < 1.414 * min_wheel_speed) && (dist_to_goal >= robot_radius) && DEADZONE_CHECK) {
		//	motor_soll << 500, -500;
		//	//motor_soll = min_wheel_speed * motor_soll.array().sign();
		//	cout << "deadzone checked" << endl;
		//	DEADZONE_CHECK = false;
		//}
		//if ((motor_soll.norm() < 1.414 * min_wheel_speed) && (dist_to_goal >= robot_radius)) {
		//	/*motor_soll <<500, -500;*/
		//	motor_soll = min_wheel_speed * motor_soll.array().sign();
		//	cout << "=================deadzone checked===================" << endl;
		//}

		// when too close, predict less time
		if (dist_to_goal <= robot_radius + 0.8) {
			predict_time = 0.8;
		}
		else {
			predict_time = predict_time0;
		}
		if (dist_to_goal <= robot_radius + 0.5) {
			TEMPORARY_GOAL_ARRIVED = true;
		}
		//car_x = motion(x_pre, u_ist, dt);
		dwa_result.u = motor_soll;

		return  dwa_result;
	}
}