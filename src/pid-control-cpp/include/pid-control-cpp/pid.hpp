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
            double update(double current_value); 

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

        private:    




    };
} // namespace pid_controller



#endif