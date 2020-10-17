from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    with open("dwa_config.yaml", "r") as ymlfile:
        flux = yaml.load(ymlfile.read())
        car_state = flux["Car_State"]
        flag_state = flux["Flag_State"]
        # print(1)
    return LaunchDescription([
        Node(
            package="dwa",
            node_executable="DWA_Parametrize",
            node_name="custom_DWA_parameter_node",
            output="screen",
            parameters=[
                {"set_goalx": car_state["set_goalx"]},
                {"set_goaly": car_state["set_goaly"]},
                {"max_speed": car_state["max_speed"]},
                {"min_speed": car_state["min_speed"]},  # -0.8  # [m/s]
                {"max_yaw_rate": car_state["max_yaw_rate"]},  # 180.0 * np.pi / 180.0  # [rad/s]
                {"max_accel": car_state["max_accel"]},  # 4.0  # [m/ss]
                {"max_delta_yaw_rate": car_state["max_delta_yaw_rate"]},  # 360.0 * np.pi / 180.0  # [rad/ss]
                {"v_resolution": car_state["v_resolution"]},  # 0.2  # [m/s]
                {"yaw_rate_resolution": car_state["yaw_rate_resolution"]},  # 15. * np.pi / 180.0  # [rad/s]
                {"min_wheel_speed": car_state["min_wheel_speed"]},  # [rpm]
                {"dt": car_state["dt"]},  # 0.2  # [s] Time tick for motion prediction
                {"predict_time": car_state["predict_time"]},  # 0.8  # [s]  less and more flexible
                {"to_goal_cost_gain": car_state["to_goal_cost_gain"]},  # 0.16
                {"obstacle_cost_gain": car_state["obstacle_cost_gain"]},  # 0.6
                {"speed_adjust_param": car_state["speed_adjust_param"]},
                {"speed_cost_gain_max": car_state["speed_cost_gain_max"]},
                {"speed_cost_gain_min": car_state["speed_cost_gain_min"]},

                {"SHOW_ANIMATION": flag_state["SHOW_ANIMATION"]},  # True
                {"GOAL_ARRIVAED": flag_state["GOAL_ARRIVAED"]},  # False
                {"RESET_STATE": flag_state["RESET_STATE"]},  # False
                {"HUMAN_SHAPE": flag_state["HUMAN_SHAPE"]},  # False
                {"MAP_TO_OBCOORD": flag_state["MAP_TO_OBCOORD"]},
                {"TRANSFORM_MAP": flag_state["TRANSFORM_MAP"]},# True
                {"MEASURE_TIME": flag_state["MEASURE_TIME"]},  # False
                {"TEMPORARY_GOAL_ARRIVED": flag_state["TEMPORARY_GOAL_ARRIVED"]},
                {"PUBLISH_DWA_STATE": flag_state["PUBLISH_DWA_STATE"]},
                {"SET_GOAL": flag_state["PUBLISH_DWA_STATE"]}
            ]
        )
    ])


generate_launch_description()
