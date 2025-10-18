// /Users/danielpi/work/LumosMotorSim/src/modules/control/angle/angle_controller.cpp
#include "angle_controller.h"
#include <algorithm>

namespace modules
{
    namespace control
    {
        namespace angle
        {
            // ========== AngleController Implementation ==========

            AngleController::AngleController(const PIDConfig &config)
                : config_(config)
            {
            }

            void AngleController::setTarget(double target)
            {
                target_ = wrapAngle(target);
            }

            double AngleController::update(double current, double dt)
            {
                if (dt <= 0.0)
                {
                    return 0.0;
                }

                // Wrap current angle
                current = wrapAngle(current);

                // Compute error using shortest path
                error_ = angleDifference(current, target_);

                // Proportional term
                double p_term = config_.kp * error_;

                // Integral term with anti-windup
                integral_ += error_ * dt;
                integral_ = std::clamp(integral_, -config_.max_integral, config_.max_integral);
                double i_term = config_.ki * integral_;

                // Derivative term
                if (first_update_)
                {
                    derivative_ = 0.0;
                    first_update_ = false;
                }
                else
                {
                    derivative_ = (error_ - prev_error_) / dt;
                }
                double d_term = config_.kd * derivative_;

                // Compute output
                double output = p_term + i_term + d_term;

                // Apply output limits
                output = std::clamp(output, config_.min_output, config_.max_output);

                // Update previous error
                prev_error_ = error_;

                return output;
            }

            void AngleController::reset()
            {
                error_ = 0.0;
                prev_error_ = 0.0;
                integral_ = 0.0;
                derivative_ = 0.0;
                first_update_ = true;
            }

            void AngleController::setConfig(const PIDConfig &config)
            {
                config_ = config;
            }

            double AngleController::wrapAngle(double angle) const
            {
                // Wrap to [-π, π)
                angle = std::fmod(angle + PI, TWO_PI);
                if (angle < 0.0)
                    angle += TWO_PI;
                return angle - PI;
            }

            double AngleController::angleDifference(double from, double to) const
            {
                // Compute shortest angular distance
                double diff = to - from;

                // Wrap to [-π, π)
                diff = std::fmod(diff + PI, TWO_PI);
                if (diff < 0.0)
                    diff += TWO_PI;
                return diff - PI;
            }

            // ========== CascadedAngleController Implementation ==========

            CascadedAngleController::CascadedAngleController(const PIDConfig &angle_config,
                                                             const PIDConfig &velocity_config)
                : angle_config_(angle_config), velocity_config_(velocity_config)
            {
            }

            void CascadedAngleController::setTarget(double target)
            {
                angle_target_ = wrapAngle(target);
            }

            double CascadedAngleController::update(double current_angle, double current_velocity, double dt)
            {
                if (dt <= 0.0)
                {
                    return 0.0;
                }

                // Wrap current angle
                current_angle = wrapAngle(current_angle);

                // ===== Outer Loop: Position Control =====
                // Compute angle error using shortest path
                angle_error_ = angleDifference(current_angle, angle_target_);

                // Proportional term
                double angle_p_term = angle_config_.kp * angle_error_;

                // Integral term with anti-windup
                angle_integral_ += angle_error_ * dt;
                angle_integral_ = std::clamp(angle_integral_,
                                             -angle_config_.max_integral,
                                             angle_config_.max_integral);
                double angle_i_term = angle_config_.ki * angle_integral_;

                // Derivative term (derivative of position error = negative velocity)
                double angle_derivative;
                if (first_update_)
                {
                    angle_derivative = 0.0;
                    first_update_ = false;
                }
                else
                {
                    angle_derivative = (angle_error_ - angle_prev_error_) / dt;
                }
                double angle_d_term = angle_config_.kd * angle_derivative;

                // Compute velocity command from position loop
                velocity_cmd_ = angle_p_term + angle_i_term + angle_d_term;

                // Limit velocity command
                velocity_cmd_ = std::clamp(velocity_cmd_, -max_velocity_, max_velocity_);

                // Update previous angle error
                angle_prev_error_ = angle_error_;

                // ===== Inner Loop: Velocity Control =====
                // Compute velocity error
                velocity_error_ = velocity_cmd_ - current_velocity;

                // Proportional term
                double velocity_p_term = velocity_config_.kp * velocity_error_;

                // Integral term with anti-windup
                velocity_integral_ += velocity_error_ * dt;
                velocity_integral_ = std::clamp(velocity_integral_,
                                                -velocity_config_.max_integral,
                                                velocity_config_.max_integral);
                double velocity_i_term = velocity_config_.ki * velocity_integral_;

                // Derivative term (acceleration error)
                double velocity_derivative = (velocity_error_ - velocity_prev_error_) / dt;
                double velocity_d_term = velocity_config_.kd * velocity_derivative;

                // Compute torque output from velocity loop
                double output = velocity_p_term + velocity_i_term + velocity_d_term;

                // Apply output limits
                output = std::clamp(output, velocity_config_.min_output, velocity_config_.max_output);

                // Update previous velocity error
                velocity_prev_error_ = velocity_error_;

                return output;
            }

            void CascadedAngleController::reset()
            {
                angle_error_ = 0.0;
                angle_prev_error_ = 0.0;
                angle_integral_ = 0.0;
                velocity_cmd_ = 0.0;
                velocity_error_ = 0.0;
                velocity_prev_error_ = 0.0;
                velocity_integral_ = 0.0;
                first_update_ = true;
            }

            void CascadedAngleController::setMaxVelocity(double max_vel)
            {
                max_velocity_ = std::abs(max_vel);
            }

            double CascadedAngleController::wrapAngle(double angle) const
            {
                // Wrap to [-π, π)
                angle = std::fmod(angle + PI, TWO_PI);
                if (angle < 0.0)
                    angle += TWO_PI;
                return angle - PI;
            }

            double CascadedAngleController::angleDifference(double from, double to) const
            {
                // Compute shortest angular distance
                double diff = to - from;

                // Wrap to [-π, π)
                diff = std::fmod(diff + PI, TWO_PI);
                if (diff < 0.0)
                    diff += TWO_PI;
                return diff - PI;
            }

        } // namespace angle
    }     // namespace control
} // namespace modules
