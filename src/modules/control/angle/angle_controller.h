// /Users/danielpi/work/LumosMotorSim/src/modules/control/angle/angle_controller.h
#pragma once

#include <cmath>

namespace modules
{
    namespace control
    {
        namespace angle
        {
            // Physical constants
            constexpr double PI = 3.14159265358979323846;
            constexpr double TWO_PI = 2.0 * PI;

            /**
             * PID controller configuration for angular control
             */
            struct PIDConfig
            {
                double kp = 1.0;              // Proportional gain
                double ki = 0.0;              // Integral gain
                double kd = 0.0;              // Derivative gain
                double max_integral = 10.0;   // Anti-windup: maximum integral term
                double max_output = 100.0;    // Output saturation limit
                double min_output = -100.0;   // Minimum output limit
            };

            /**
             * Angular PID controller with proper angle wrapping
             *
             * This controller handles angular positions in radians and properly
             * wraps angles to compute the shortest path error. Useful for motor
             * position control where angles wrap around at 2π.
             */
            class AngleController
            {
            public:
                explicit AngleController(const PIDConfig &config = PIDConfig());

                /**
                 * Set the target angle (setpoint)
                 * @param target Target angle in radians
                 */
                void setTarget(double target);

                /**
                 * Get the current target angle
                 * @return Target angle in radians
                 */
                double getTarget() const { return target_; }

                /**
                 * Update controller and compute control output
                 * @param current Current angle in radians
                 * @param dt Time step in seconds
                 * @return Control output (e.g., torque command)
                 */
                double update(double current, double dt);

                /**
                 * Reset controller state (clears integral, previous error)
                 */
                void reset();

                /**
                 * Get the current error
                 * @return Angular error in radians
                 */
                double getError() const { return error_; }

                /**
                 * Get the integral term
                 * @return Accumulated integral error
                 */
                double getIntegral() const { return integral_; }

                /**
                 * Get the derivative term
                 * @return Error derivative
                 */
                double getDerivative() const { return derivative_; }

                /**
                 * Update controller gains
                 * @param config New PID configuration
                 */
                void setConfig(const PIDConfig &config);

                /**
                 * Get current controller configuration
                 * @return Current PID configuration
                 */
                const PIDConfig &getConfig() const { return config_; }

            private:
                /**
                 * Wrap angle to [-π, π)
                 * @param angle Angle in radians
                 * @return Wrapped angle
                 */
                double wrapAngle(double angle) const;

                /**
                 * Compute shortest angular distance from a to b
                 * Returns a value in [-π, π)
                 * @param a From angle (radians)
                 * @param b To angle (radians)
                 * @return Shortest angular distance
                 */
                double angleDifference(double a, double b) const;

                PIDConfig config_;
                double target_ = 0.0;
                double error_ = 0.0;
                double prev_error_ = 0.0;
                double integral_ = 0.0;
                double derivative_ = 0.0;
                bool first_update_ = true;
            };

            /**
             * Cascaded angle + velocity controller
             *
             * Uses an outer position loop that generates velocity commands,
             * and an inner velocity loop that generates torque/force commands.
             * This is a common architecture for motor control.
             */
            class CascadedAngleController
            {
            public:
                /**
                 * Constructor
                 * @param angle_config Position loop PID gains
                 * @param velocity_config Velocity loop PID gains
                 */
                CascadedAngleController(const PIDConfig &angle_config,
                                       const PIDConfig &velocity_config);

                /**
                 * Set the target angle (setpoint)
                 * @param target Target angle in radians
                 */
                void setTarget(double target);

                /**
                 * Get the current target angle
                 * @return Target angle in radians
                 */
                double getTarget() const { return angle_target_; }

                /**
                 * Update controller and compute control output
                 * @param current_angle Current angle in radians
                 * @param current_velocity Current angular velocity in rad/s
                 * @param dt Time step in seconds
                 * @return Control output (e.g., torque command)
                 */
                double update(double current_angle, double current_velocity, double dt);

                /**
                 * Reset both controllers
                 */
                void reset();

                /**
                 * Get position error
                 * @return Angular error in radians
                 */
                double getAngleError() const { return angle_error_; }

                /**
                 * Get velocity error
                 * @return Velocity error in rad/s
                 */
                double getVelocityError() const { return velocity_error_; }

                /**
                 * Get commanded velocity from position loop
                 * @return Velocity command in rad/s
                 */
                double getVelocityCommand() const { return velocity_cmd_; }

                /**
                 * Set maximum velocity limit for position loop
                 * @param max_vel Maximum velocity in rad/s
                 */
                void setMaxVelocity(double max_vel);

            private:
                double wrapAngle(double angle) const;
                double angleDifference(double a, double b) const;

                // Position loop
                PIDConfig angle_config_;
                double angle_target_ = 0.0;
                double angle_error_ = 0.0;
                double angle_prev_error_ = 0.0;
                double angle_integral_ = 0.0;
                double max_velocity_ = 100.0;  // rad/s

                // Velocity loop
                PIDConfig velocity_config_;
                double velocity_cmd_ = 0.0;
                double velocity_error_ = 0.0;
                double velocity_prev_error_ = 0.0;
                double velocity_integral_ = 0.0;

                bool first_update_ = true;
            };

        } // namespace angle
    }     // namespace control
} // namespace modules
