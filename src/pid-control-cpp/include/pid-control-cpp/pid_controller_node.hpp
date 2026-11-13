#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP


#include <memory>
#include <string> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "pid-control-cpp/pid.hpp"


namespace pid_controller{

    class RosPIDControllerNode : public rclcpp::Node{

        public: 

            explicit RosPIDControllerNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

            ~RosPIDControllerNode() = default; 

        private: 

            std::unique_ptr<PID> pid_controller_; 

            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_; 
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr controller_pub_; 
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pid_enable_sub_; 
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_sub_; 

             
            OnSetParametersCallbackHandle::SharedPtr params_callback_handler_; 

            double current_state_{0.0}; 
            double current_setpoint_{0.0};
            bool has_received_state_{false};
            bool has_received_setpoint_{false}; 
            bool auto_mode_{true};
            bool reverse_action_{false}; 

            rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& params);


            void setpointCallback(const std_msgs::msg::Float64::SharedPtr msg);
            void stateCallback(const std_msgs::msg::Float64::SharedPtr msg);
            void pidEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

            void initCOntroller();        

    };

} //pid_controller 



#endif