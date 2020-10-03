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

//yaml
#include "yaml-cpp/yaml.h"
//------------------------------
#include "DWA_Planner.h"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};



int main(int argc, char * argv[])
{
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  // rclcpp::shutdown();

  YAML::Node config = YAML::LoadFile("./dev_ws/src/dwa/src/config.yaml");
  YAML::Node dwaNode = config["dwa"]; 
  for (unsigned int i = 0; i < dwaNode.size(); i++) {
            const YAML::Node& cNode = dwaNode[i];
            std::cout << cNode << "\n";
            std::cout << "===================" << std::endl; 
 
            DWA_CONFIG dwaS;
            dwaS.max_speed = cNode["max_speed"].as<double>();
	    dwaS.min_speed = cNode["min_speed"].as<double>();
	    std::cout << "max_speed"<< dwaS.max_speed << std::endl; 
	    std::cout << "min_speed"<< dwaS.min_speed << std::endl;  
        }

   std::cout << "finish YAML:\n";
   return 0;
}
