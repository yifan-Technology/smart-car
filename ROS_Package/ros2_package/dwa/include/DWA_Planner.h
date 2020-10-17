#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <iostream>
#include<Eigen/Dense>
#include<iostream>
#include<fstream>
#include <jsoncpp/json/json.h>

using namespace Eigen;
using State = Matrix<double, 5, 1 >;
using Control = Matrix<double, 2, 1>;
using Goal = Matrix<double, 2, 1>;
using Window = Matrix<double, 4, 1>;
using All_Traj = std::vector<MatrixXd>;
//using All_Traj = Matrix<MatrixXd,1, 1>;
struct DWA_result { Control u; MatrixXd traj; All_Traj all_traj; };
#define PI 3.1415926535

namespace dwa_planner {
	using namespace std;

	class DWA {

	public:
		//! Constructor
		Goal set_goal;
		double max_speed;
		double min_speed;//  - 0.8  [m / s]
		double max_yaw_rate; // 180.0 * PI / 180.0   [rad / s]
		double max_accel; // 4.0   [m / ss]
		double max_delta_yaw_rate;  // 360.0 * PI / 180.0   [rad / ss]
		double v_resolution;// 0.2   [m / s]
		double yaw_rate_resolution;// 15. * PI / 180.0   [rad / s]
		double min_wheel_speed;//100 [rpm]
		double dt;// 0.2  [s] Time tick for motion prediction
		double predict_time; // 0.8   [s]  less and more flexible

		double to_goal_cost_gain;// 0.16
		double obstacle_cost_gain;//  0.6
		double speed_adjust_param;
		double speed_cost_gain_max;
		double speed_cost_gain_min;

		bool SHOW_ANIMATION; // true
		bool GOAL_ARRIVAED; // false
		bool RESET_STATE; // false
		bool HUMAN_SHAPE; // false
		bool MAP_TO_OBCOORD; // true
		bool MEASURE_TIME; // false
		bool TEMPORARY_GOAL_ARRIVED;
		bool PUBLISH_DWA_STATE;

		double m = 5;
		double g = 9.80665;
		double mu = 0.01;
		double x0 = 0.01;
		double wheel_len_dist = 0.475; //car wheelcenter len distance
		double wheel_quer_dist = 0.460;
		double car_h = 0.4;
		double wheel_radius = 0.095;
		double robot_radius = 0.5;  // [m] for collision check
		double robot_length = 0.661 + 0.1;  // [m] for collision check
		double robot_width = 0.504 + 0.1;  // [m] for collision check
		double dist_to_goal = 1e10; // 1e10

		State car_x;
        Matrix2d vtrans;
		vtrans << 1, -wheel_quer_dist/2, 1, wheel_quer_dist/2;

		void readJsonFromFile();

		State koordinaten_transfomation(Control wheel_speed, double theta);

		State motion(State x, Control u, double dt);

		Control speed_change(Control u_in, string mode);

		MatrixXd obmap2coordinaten(MatrixXd obmap, double res);

		double dynamic_speed_cost(double dmin);

		Goal goal_inverse_transforamtion(State x, Goal goal);

		MatrixXd ob_inverse_transforamtion(State x, MatrixXd ob_cood);

		Window calc_dynamic_window(State state);

		MatrixXd predict_trajectory(State x_init, double v_soll, double omega_soll);

		double calc_to_goal_cost(MatrixXd trajectory, Goal goal);

		double calc_obstacle_cost(MatrixXd trajectory, MatrixXd obstacle);

		DWA_result calc_control_and_trajectory(State state, Window dw, Goal goal, MatrixXd obstacle);

		DWA_result dwa_control(Control motor_ist, State x_pre, Goal zw_goal, MatrixXd ob_list);
	};
} // namespace dwa_planner

#endif// DWA_PLANNER_H