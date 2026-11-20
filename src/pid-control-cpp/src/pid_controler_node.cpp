#include "pid-control-cpp/pid_controller_node.hpp"

namespace pid_controller{

    RosPIDControllerNode::RosPIDControllerNode(const rclcpp::NodeOptions& node_options) : Node("pid_controller", node_options)
    {

        this->declare_parameter<double>("p_gain", 0.0); 
        this->declare_parameter<double>("i_gain", 0.0); 
        this->declare_parameter<double>("d_gain", 0.0);
        this->declare_parameter<double>("setpoint", 0.0); 
        this->declare_parameter<double>("output_min", -1.0);
        this->declare_parameter<double>("output_max", 1.0); 
        this->declare_parameter<double>("integral_min", -1.0);
        this->declare_parameter<double>("integral_max", 1.0); 
        this->declare_parameter("state_topic", "/state");
        this->declare_parameter("setpoint_topic", "/setpoint");
        this->declare_parameter("pid_enable_topic", "/pid_enable"); 
        this->declare_parameter("control_effort", "/control_effort"); 
        this->declare_parameter<bool>("reverse_action", false); 
        this->declare_parameter<bool>("auto_start", true);

        initController(); 

        std::string STATE_TOPIC = this->get_parameter("state_topic").as_string(); 
        std::string SETPOINT_TOPIC = this->get_parameter("setpoint_topic").as_string(); 
        std::string PID_ENABLE_TOPIC = this->get_parameter("pid_enable_topic").as_string(); 
        std::string CONTROL_EFFORT_TOPIC = this->get_parameter("control_effort").as_string(); 

        params_callback_handler_ = this->add_on_set_parameters_callback(
            std::bind(&RosPIDControllerNode::parametersCallback, this, std::placeholders::_1));
            

        controller_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                CONTROL_EFFORT_TOPIC, 10);
        state_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            STATE_TOPIC, 10, std::bind(&RosPIDControllerNode::stateCallback, this, std::placeholders::_1));
        setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            SETPOINT_TOPIC, 10, std::bind(&RosPIDControllerNode::setpointCallback, this, std::placeholders::_1));
        pid_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            PID_ENABLE_TOPIC, 10, std::bind(&RosPIDControllerNode::pidEnableCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "PID controller node initialized with: Kp=%.4f, Ki=%.4f, Kd=%.4f", 
            pid_controller_->kp, pid_controller_->ki, pid_controller_->kd); 
        RCLCPP_INFO(this->get_logger(), "Subscribed to topics: %s (state) and %s (setpoint), and %s (enable)", 
            STATE_TOPIC.c_str(), SETPOINT_TOPIC.c_str(), PID_ENABLE_TOPIC.c_str()); 
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s (control_effort)", CONTROL_EFFORT_TOPIC.c_str());

    }


    void RosPIDControllerNode::initController(){

        double kp = this->get_parameter("p_gain").as_double(); 
        double ki = this->get_parameter("i_gain").as_double(); 
        double kd = this->get_parameter("d_gain").as_double(); 

        double output_min = this->get_parameter("output_min").as_double();
        double output_max = this->get_parameter("output_max").as_double(); 
        double integral_min = this->get_parameter("integral_min").as_double(); 
        double integral_max = this->get_parameter("integral_max").as_double(); 


        //initializing the pid class with input values

        pid_controller_ = 
            std::make_unique<PID>(kp, ki, kd, 0.0, 
                std::make_pair(output_min, output_max), std::make_pair(integral_min, integral_max)); 

    }


    void RosPIDControllerNode::stateCallback(const std_msgs::msg::Float64::SharedPtr msg){

        current_state_ = msg->data; 
        has_received_state_ = true; 
        RCLCPP_INFO(this->get_logger(), "Received state value: %.4f", current_state_); 

        if(has_received_state_ && auto_mode_){
            
            double control_effort = pid_controller_->updatePID(current_state_); 

            if(reverse_action_){
                control_effort = -control_effort; 
            }
            
            RCLCPP_INFO(this->get_logger(), "Control loop has been updated: state = %.4f, setpoint = %.4f, controll_effort = %.4f", current_state_, current_setpoint_, control_effort);

            auto output_msg = std::make_unique<std_msgs::msg::Float64>(); 
            output_msg->data = control_effort; 
            controller_pub_->publish(std::move(output_msg)); 
        }
        else if(!auto_mode_){
            RCLCPP_DEBUG(this->get_logger(), "PID controller is disabled, not calculating control");
        }
        else if(!has_received_setpoint_){
            RCLCPP_DEBUG(this->get_logger(), "No setpoint received, not calculating control"); 
        }
    }

    void RosPIDControllerNode::setpointCallback(const std_msgs::msg::Float64::SharedPtr msg){

        double new_setpoint = msg->data; 
        has_received_setpoint_ = true; 
        RCLCPP_INFO(this->get_logger(), "received new setpoint with value: %.4f", new_setpoint);

        if(current_setpoint_ != new_setpoint){
            current_setpoint_ = new_setpoint; 
            pid_controller_->setpoint(current_setpoint_); 
            RCLCPP_INFO(this->get_logger(), "Updated current setpoint to %.4f", current_setpoint_); 
        }else{
            RCLCPP_DEBUG(this->get_logger(), "Current setpoint remains unchanged (%.4f), not updating value", current_setpoint_);
        }
    }
    void RosPIDControllerNode::pidEnableCallback(const std_msgs::msg::Bool::SharedPtr msg){

        auto_mode_ = msg->data; 

        if(!auto_mode_){
            pid_controller_->reset();
        }

        RCLCPP_INFO(this->get_logger(), "PID controller %s", auto_mode_? "enabled" : "disabled"); 
    }




}