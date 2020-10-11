// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DWA_Publisher : public rclcpp::Node
{
public:
  DWA_Publisher()
      : Node("DWA_Publisher"), count_(0)
  {
    //publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/yf_camera/goal", 1);
    publisherGoal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/yf_camera/goal", 1);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&DWA_Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::PoseStamped();
    // std::vector<float> realgoal;
    // realgoal.push_back(10);
    // realgoal.push_back(10);
    // message.data = realgoal;
    message.pose.orientation.x = 0;
    message.pose.orientation.y = 0;
    message.pose.orientation.z = 0;
    message.pose.position.x = 10;
    message.pose.position.y = 10;
    message.pose.position.z = 0;
    publisherGoal_->publish(message);
    std::cout << "current goal: " << message.pose.position.x << "," << message.pose.position.y << std::endl;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherGoal_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DWA_Publisher>());
  rclcpp::shutdown();
  return 0;
}
