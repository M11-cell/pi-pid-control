#ifndef PID_HPP
#define PID_HPP

#include <chrono> 
#include <limits>
#include <cmath> 


namespace pid_controller
{
    class PID{

        public: 

            /**
             * @brief PID controller constructor 
             * 
             * @param p = Positional gain 
             * @param i = integral gain 
             * @param d = derivative gain 
             * @param setpoint = target/desired value system must maintain
             * @param output_limits = max and min limits for output clamping 
             * @param integral_limits = Range limits for integral clamping
            */

            PID(
                double p = 0.0, 
                double i = 0.0, 
                double d = 0.0,
                double setpoint = 0.0, 
                std::pair<double, double> output_limits = {-std::numeric_limits<double>::infinity(), 
                    std::numeric_limits<double>::infinity()},
                std::pair<double, double> integral_limits = {-std::numeric_limits<double>::infinity(), 
                    std::numeric_limits<double>::infinity()}
            );

            /**
             * @brief Calculates the PID output value for a given reference input and feedback
             * 
             * @param current_value Takes in current value of process variable 
             * @return control output
            */
            double updatePID(double current_value); 

            /**
             * @brief Resets the PID controller
            */
            void reset(); 

            /**
             * @brief Updates the setpoint value
             * 
             * @param new_setpoint new target setpoint
            */
            void setpoint(double new_setpoint);


            /**
             * @brief Calculates the PID error value for a given reference input and feedback
             * 
             * @param desired_state Takes in the setpoint value as the desired state
             * @param measured_state the last calculated output
             * @return positional error 
            */
            double calculateError(double desired_state, double measured_state);



            double kp; 
            double ki; 
            double kd; 
            double setpoint_;
            std::pair<double, double> output_limits_; 
            std::pair<double,double> integral_limits_; 
            double output; //last caluclated output 

        private:  
        
            double proportional_{0.0};
            double integral_{0.0};
            double derivative_{0.0};

            std::chrono::time_point<std::chrono::steady_clock> last_time_;
            double last_error_{0.0};
            bool first_update_{true}; 
    };
} // namespace pid_controller



#endif // PID_HPP