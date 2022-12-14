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

/**
 * @file publisher_member_function.cpp
 * @author Badrinarayanan Raghunathan Srikumar
 * @brief A simple publisher and also a server to add two integers
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
size_t sum_;

/**
 * @brief Publisher class to handle server
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
/**
 * @brief Construct a new Minimal Publisher object
 * 
 */
  MinimalPublisher() : Node("minimal_publisher") {
    auto freq = rcl_interfaces::msg::ParameterDescriptor();
    freq.description = "Publishing frequency";
    this->declare_parameter("frequency", 3.0, freq);
    // auto frequency =
      // this->get_parameter("frequency").get_parameter_value().get<std::float_t>();
  // initialze publisheer
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

    auto serviceCallbackPtr =
        std::bind(&MinimalPublisher::add, this,
                  std::placeholders::_1, std::placeholders::_2);
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", serviceCallbackPtr);
  }

 private:
/**
 * @brief Timer callback to publish message
 * 
 */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = std::to_string(sum_);
    RCLCPP_INFO_STREAM(this->get_logger(),
    "Publishing output:" << message.data);
    publisher_->publish(message);
  }
  /**
   * @brief Service callback to perform addition when request is received
   * 
   * @param request 
   * @param response 
   */
  void add(const std::shared_ptr<example_interfaces::srv::
     AddTwoInts::Request> request, std::shared_ptr<example_interfaces
     ::srv::AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    sum_ = response->sum;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "Service request\na: %ld" " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "Output response: [%ld]", (long int)response->sum);
  }

  // members
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  size_t sum;
};
/*
void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = defaultMessage + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:" << message.data);
    publisher_->publish(message);
}

*/
int main(int argc, char * argv[]) {
  /**
  rclcpp::init(argc, argv);
  
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ = node->create_publisher<std_msgs::msg::String>("topic", 10);

  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(500ms, &timer_callback);

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
     node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  while(rclcpp::ok())
  {
    auto message = std_msgs::msg::String();
    message.data = "The sum is : " + std::to_string(sum_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    
  }
  */

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
