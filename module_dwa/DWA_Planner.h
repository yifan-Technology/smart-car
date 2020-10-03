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


namespace dwa_planner {
	using namespace std;
	class DWA {

	public:
		//! Constructor
		DWA();

		double max_speed;
		double min_speed;
		double max_yaw_rate;
		double max_accel;
		double max_delta_yaw_rate;
		double v_resolution;
		double yaw_rate_resolution;
		double min_wheel_speed;
		double dt;
		double predict_time;

		double to_goal_cost_gain;
		double obstacle_cost_gain;
		double speed_adjust_param;
		double speed_cost_gain_max;
		double speed_cost_gain_min;
		double dist_to_goal;

		bool SHOW_ANIMATION;
		bool GOAL_ARRIVAED;
		bool RESET_STATE;
		bool HUMAN_SHAPE;
		bool MAP_TO_OBCOORD;
		bool MEASURE_TIME;
		bool TEMPORARY_GOAL_ARRIVED;
		double m;
		double g;
		double mu;
		double x0;
		double car_a;
		double car_b;
		double car_c;
		double car_h;
		double wheel_radius;
		State x_0;
		double robot_radius;
		double robot_width;
		double robot_length;

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