#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H
#include <iostream>
#include<Eigen/Dense>

using namespace Eigen;
using State = Matrix<double, 5, 1 >;
using Control = Matrix<double, 2, 1>;
using Goal = Matrix<double, 2, 1>;
using Window = Matrix<double, 4, 1>;
using All_Traj = std::vector<MatrixXd>;
//using All_Traj = Matrix<MatrixXd,1, 1>;
struct DWA_result { Control u; MatrixXd traj; All_Traj all_traj; };
#define PI 3.141592653

namespace dwa_planner {
	using namespace std;
	class DWA {

	public:
		//! Constructor

		double max_speed = 1.8; // 0.8   [m / s] 
		double min_speed = -1.8; //  - 0.8  [m / s]
		double max_yaw_rate = 720 * PI / 180;  // 180.0 * PI / 180.0   [rad / s]
		double max_accel = 1.5; // 4.0   [m / ss]
		double max_delta_yaw_rate = 360 * PI / 180;  // 360.0 * PI / 180.0   [rad / ss]
		double v_resolution = 0.1; // 0.2   [m / s]
		double yaw_rate_resolution = 10 * PI / 180;  // 15. * PI / 180.0   [rad / s]
		double min_wheel_speed = 100; //100 [rpm]
		double dt = 0.1; // 0.2  [s] Time tick for motion prediction
		double predict_time = 2.; // 0.8   [s]  less and more flexible

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
		double robot_radius = 0.5;  // [m] for collision check
		double robot_length = 0.661 + 0.1;  // [m] for collision check
		double robot_width = 0.504 + 0.1;  // [m] for collision check

		State koordinaten_transfomation(Control wheel_speed, double theta);

		State motion(State x, Control u, double dt);

		Control  speed_change(Control u_in, string mode);

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
}; // namespace dwa_planner

#endif#// DWA_PLANNER_H