// /Users/danielpi/work/LumosMotorSim/src/modules/control/velocity/velocity_controller.h
#pragma once

#include <cmath>

namespace modules
{
    namespace control
    {
        namespace velocity
        {
            /**
             * PID controller configuration for velocity control
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
             * Velocity PID controller
             *
             * Controls velocity (linear or angular) using PID control.
             * Outputs force/torque command based on velocity error.
             */
            class VelocityController
            {
            public:
                explicit VelocityController(const PIDConfig &config = PIDConfig());

                /**
                 * Set the target velocity (setpoint)
                 * @param target Target velocity (rad/s or m/s)
                 */
                void setTarget(double target);

                /**
                 * Get the current target velocity
                 * @return Target velocity
                 */
                double getTarget() const { return target_; }

                /**
                 * Update controller and compute control output
                 * @param current Current velocity (rad/s or m/s)
                 * @param dt Time step in seconds
                 * @return Control output (torque in Nm or force in N)
                 */
                double update(double current, double dt);

                /**
                 * Update controller with measured acceleration (for improved D term)
                 * @param current Current velocity
                 * @param acceleration Measured acceleration (rad/s² or m/s²)
                 * @param dt Time step in seconds
                 * @return Control output
                 */
                double updateWithAcceleration(double current, double acceleration, double dt);

                /**
                 * Reset controller state (clears integral, previous error)
                 */
                void reset();

                /**
                 * Get the current velocity error
                 * @return Velocity error
                 */
                double getError() const { return error_; }

                /**
                 * Get the integral term
                 * @return Accumulated integral error
                 */
                double getIntegral() const { return integral_; }

                /**
                 * Get the derivative term
                 * @return Error derivative (acceleration error)
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
                PIDConfig config_;
                double target_ = 0.0;
                double error_ = 0.0;
                double prev_error_ = 0.0;
                double integral_ = 0.0;
                double derivative_ = 0.0;
                bool first_update_ = true;
            };

            /**
             * Acceleration feedforward configuration
             */
            struct FeedforwardConfig
            {
                double kv = 0.0;     // Velocity feedforward gain (damping compensation)
                double ka = 0.0;     // Acceleration feedforward gain (inertia compensation)
                double kf = 0.0;     // Friction feedforward gain
            };

            /**
             * Velocity controller with feedforward compensation
             *
             * Combines PID feedback with feedforward terms for improved
             * tracking performance and disturbance rejection.
             *
             * Output = PID(error) + kv*target_vel + ka*target_accel + kf*sign(target_vel)
             */
            class VelocityControllerWithFF
            {
            public:
                VelocityControllerWithFF(const PIDConfig &pid_config = PIDConfig(),
                                        const FeedforwardConfig &ff_config = FeedforwardConfig());

                /**
                 * Set the target velocity
                 * @param target Target velocity
                 */
                void setTarget(double target);

                /**
                 * Get the current target velocity
                 * @return Target velocity
                 */
                double getTarget() const { return target_velocity_; }

                /**
                 * Update controller with velocity only
                 * @param current Current velocity
                 * @param dt Time step in seconds
                 * @return Control output
                 */
                double update(double current, double dt);

                /**
                 * Update controller with velocity and acceleration
                 * Uses acceleration for better derivative term and feedforward
                 * @param current Current velocity
                 * @param current_accel Current acceleration
                 * @param dt Time step in seconds
                 * @return Control output
                 */
                double updateWithAcceleration(double current, double current_accel, double dt);

                /**
                 * Set target velocity trajectory (velocity and acceleration)
                 * Enables acceleration feedforward
                 * @param velocity Target velocity
                 * @param acceleration Target acceleration
                 */
                void setTrajectory(double velocity, double acceleration);

                /**
                 * Reset controller state
                 */
                void reset();

                /**
                 * Get velocity error
                 * @return Velocity error
                 */
                double getError() const { return error_; }

                /**
                 * Get feedforward component of output
                 * @return Feedforward output
                 */
                double getFeedforward() const { return feedforward_; }

                /**
                 * Get feedback (PID) component of output
                 * @return Feedback output
                 */
                double getFeedback() const { return feedback_; }

                /**
                 * Update configurations
                 */
                void setPIDConfig(const PIDConfig &config);
                void setFFConfig(const FeedforwardConfig &config);

                const PIDConfig &getPIDConfig() const { return pid_config_; }
                const FeedforwardConfig &getFFConfig() const { return ff_config_; }

            private:
                double computeFeedforward() const;

                PIDConfig pid_config_;
                FeedforwardConfig ff_config_;

                double target_velocity_ = 0.0;
                double target_acceleration_ = 0.0;

                double error_ = 0.0;
                double prev_error_ = 0.0;
                double integral_ = 0.0;
                double derivative_ = 0.0;

                double feedforward_ = 0.0;
                double feedback_ = 0.0;

                bool first_update_ = true;
            };

            /**
             * Velocity profile generator for smooth motion
             *
             * Generates trapezoidal or S-curve velocity profiles
             * with configurable acceleration and jerk limits.
             */
            class VelocityProfileGenerator
            {
            public:
                enum class ProfileType
                {
                    TRAPEZOIDAL,  // Constant acceleration (limited)
                    S_CURVE       // Smooth S-curve (jerk-limited)
                };

                struct ProfileConfig
                {
                    double max_acceleration;  // Maximum acceleration (rad/s² or m/s²)
                    double max_jerk;          // Maximum jerk (rad/s³ or m/s³)
                    ProfileType type;

                    ProfileConfig()
                        : max_acceleration(10.0),
                          max_jerk(100.0),
                          type(ProfileType::TRAPEZOIDAL)
                    {
                    }
                };

                explicit VelocityProfileGenerator(const ProfileConfig &config);

                /**
                 * Set new target velocity
                 * @param target Target velocity
                 * @param current Current velocity
                 */
                void setTarget(double target, double current);

                /**
                 * Update profile and get commanded velocity
                 * @param dt Time step
                 * @return Commanded velocity for this timestep
                 */
                double update(double dt);

                /**
                 * Get current commanded velocity
                 * @return Velocity command
                 */
                double getVelocity() const { return current_velocity_; }

                /**
                 * Get current commanded acceleration
                 * @return Acceleration command
                 */
                double getAcceleration() const { return current_acceleration_; }

                /**
                 * Check if profile has reached target
                 * @return True if at target velocity
                 */
                bool isComplete() const;

                /**
                 * Reset profile generator
                 */
                void reset();

                /**
                 * Update configuration
                 */
                void setConfig(const ProfileConfig &config);
                const ProfileConfig &getConfig() const { return config_; }

            private:
                void updateTrapezoidal(double dt);
                void updateSCurve(double dt);

                ProfileConfig config_;
                double target_velocity_ = 0.0;
                double current_velocity_ = 0.0;
                double current_acceleration_ = 0.0;
                double current_jerk_ = 0.0;
            };

        } // namespace velocity
    }     // namespace control
} // namespace modules
