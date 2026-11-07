#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP


#include <memory>
#include <string> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "pid-control-cpp/pid.hpp"


namespace pid_controller_node{

    class RosPIDControllerNode : public rclcpp::Node{

        public: 

            explicit RosPIDControllerNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

            ~RosPIDControllerNode() = default; 

        private: 

            std::unique<PID> pid_controller_; 

            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_; 
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr controller_pub_; 
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pid_enable_sub_; 
            

    };



}



#endif