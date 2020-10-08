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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std::placeholders::_1;

class DWASubscriber : public rclcpp::Node
{
public:
  DWASubscriber()
  : Node("DWA_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "dwa_sollspeed", 10, std::bind(&DWASubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      auto RealSpeed = msg->data;
      std::cout<<"broadcast speed :" << RealSpeed[0]<<","<<RealSpeed[1] <<","<<RealSpeed[2]<<","<<RealSpeed[3]<<std::endl;
   }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DWASubscriber>());
  rclcpp::shutdown();
  return 0;
}
