#include <pid-control-cpp/pid.hpp>

namespace pid_controller{

    PID::PID(
        double p, 
        double i, 
        double d, 
        double setpoint_val, 
        std::pair<double, double> output_limits_val,
        std::pair<double, double> integral_limits_val
        ) : kp(p), 
            ki(i), 
            kd(d),
            setpoint_(setpoint_val), 
            output_limits_(output_limits_val), 
            integral_limits_(integral_limits_val),
            output{0.0},
            last_time_(std::chrono::steady_clock::now()){}

    
    /**
     * @brief Calculates the PID output value for a given reference input and feedback
     * 
     * @param current_value Takes in current value of process variable 
     * @return control output
    */       
    double PID::updatePID(double current_value){

        auto current_time = std::chrono::steady_clock::now(); 

        //Calculate time delta
        std::chrono::duration<double> time_delta = current_time - last_time_; 
        double dt = time_delta.count(); 

        //safety checks: avoiding division by 0
        if(first_update_ || dt < 1e-16){
            first_update_ = false; 
            last_time_ = current_time; 
            return output; 
        }

        //Calculating error 
        double error = calculateError(setpoint_, current_value);

        //Setting PID terms
        proportional_ = kp*error; 

        integral_ = ki * error * dt; 

        //applying integral limits 
        integral_ = std::max(std::min(integral_, integral_limits_.second), integral_limits_.second); 

        double d_error = (error - last_error_)/dt; 

        derivative_ = kd * d_error; 

        output = proportional_ + integral_ + derivative_; 

        //applying output limits
        output = std::max(std::min(output, output_limits_.second), output_limits_.first); 

        //storing error and time values for next iteration. 
        last_error_ = error; 
        last_time_ = current_time;
        
        return output; 
    }

    void PID::reset(){ 

        proportional_ = 0.0;
        integral_ = 0.0;
        derivative_ = 0.0;
        last_error_ = 0.0; 
        last_time_ = std::chrono::steady_clock::now();  
        output = 0.0; 
        first_update_ = true; 
    }

    void PID::setpoint(double new_setpoint){
        setpoint_ = new_setpoint; 
    }

    void PID::tunePID(double p, double i, double d){

        kp = p; 
        ki = i;
        kd = d; 
    }

    double PID::calculateError(double desired_state, double measured_state){

        return desired_state - measured_state; 
    }
} //namespace pid_controller

