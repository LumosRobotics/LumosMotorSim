// /Users/danielpi/work/LumosMotorSim/src/modules/control/velocity/velocity_controller.cpp
#include "velocity_controller.h"
#include <algorithm>

namespace modules
{
    namespace control
    {
        namespace velocity
        {
            // ========== VelocityController Implementation ==========

            VelocityController::VelocityController(const PIDConfig &config)
                : config_(config)
            {
            }

            void VelocityController::setTarget(double target)
            {
                target_ = target;
            }

            double VelocityController::update(double current, double dt)
            {
                if (dt <= 0.0)
                {
                    return 0.0;
                }

                // Compute error
                error_ = target_ - current;

                // Proportional term
                double p_term = config_.kp * error_;

                // Integral term with anti-windup
                integral_ += error_ * dt;
                integral_ = std::clamp(integral_, -config_.max_integral, config_.max_integral);
                double i_term = config_.ki * integral_;

                // Derivative term (rate of change of error = -acceleration)
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

            double VelocityController::updateWithAcceleration(double current, double acceleration, double dt)
            {
                if (dt <= 0.0)
                {
                    return 0.0;
                }

                // Compute error
                error_ = target_ - current;

                // Proportional term
                double p_term = config_.kp * error_;

                // Integral term with anti-windup
                integral_ += error_ * dt;
                integral_ = std::clamp(integral_, -config_.max_integral, config_.max_integral);
                double i_term = config_.ki * integral_;

                // Derivative term using measured acceleration
                // d(error)/dt = d(target - current)/dt = -d(current)/dt = -acceleration
                derivative_ = -acceleration;
                double d_term = config_.kd * derivative_;

                first_update_ = false;

                // Compute output
                double output = p_term + i_term + d_term;

                // Apply output limits
                output = std::clamp(output, config_.min_output, config_.max_output);

                // Update previous error
                prev_error_ = error_;

                return output;
            }

            void VelocityController::reset()
            {
                error_ = 0.0;
                prev_error_ = 0.0;
                integral_ = 0.0;
                derivative_ = 0.0;
                first_update_ = true;
            }

            void VelocityController::setConfig(const PIDConfig &config)
            {
                config_ = config;
            }

            // ========== VelocityControllerWithFF Implementation ==========

            VelocityControllerWithFF::VelocityControllerWithFF(const PIDConfig &pid_config,
                                                               const FeedforwardConfig &ff_config)
                : pid_config_(pid_config), ff_config_(ff_config)
            {
            }

            void VelocityControllerWithFF::setTarget(double target)
            {
                target_velocity_ = target;
                target_acceleration_ = 0.0;  // No acceleration command
            }

            void VelocityControllerWithFF::setTrajectory(double velocity, double acceleration)
            {
                target_velocity_ = velocity;
                target_acceleration_ = acceleration;
            }

            double VelocityControllerWithFF::update(double current, double dt)
            {
                if (dt <= 0.0)
                {
                    return 0.0;
                }

                // Compute error
                error_ = target_velocity_ - current;

                // ===== Feedback (PID) =====
                double p_term = pid_config_.kp * error_;

                integral_ += error_ * dt;
                integral_ = std::clamp(integral_, -pid_config_.max_integral, pid_config_.max_integral);
                double i_term = pid_config_.ki * integral_;

                if (first_update_)
                {
                    derivative_ = 0.0;
                    first_update_ = false;
                }
                else
                {
                    derivative_ = (error_ - prev_error_) / dt;
                }
                double d_term = pid_config_.kd * derivative_;

                feedback_ = p_term + i_term + d_term;

                // ===== Feedforward =====
                feedforward_ = computeFeedforward();

                // ===== Total Output =====
                double output = feedback_ + feedforward_;

                // Apply output limits
                output = std::clamp(output, pid_config_.min_output, pid_config_.max_output);

                prev_error_ = error_;

                return output;
            }

            double VelocityControllerWithFF::updateWithAcceleration(double current, double current_accel, double dt)
            {
                if (dt <= 0.0)
                {
                    return 0.0;
                }

                // Compute error
                error_ = target_velocity_ - current;

                // ===== Feedback (PID) =====
                double p_term = pid_config_.kp * error_;

                integral_ += error_ * dt;
                integral_ = std::clamp(integral_, -pid_config_.max_integral, pid_config_.max_integral);
                double i_term = pid_config_.ki * integral_;

                // Use measured acceleration for derivative
                derivative_ = -current_accel;
                double d_term = pid_config_.kd * derivative_;

                first_update_ = false;

                feedback_ = p_term + i_term + d_term;

                // ===== Feedforward =====
                feedforward_ = computeFeedforward();

                // ===== Total Output =====
                double output = feedback_ + feedforward_;

                // Apply output limits
                output = std::clamp(output, pid_config_.min_output, pid_config_.max_output);

                prev_error_ = error_;

                return output;
            }

            void VelocityControllerWithFF::reset()
            {
                error_ = 0.0;
                prev_error_ = 0.0;
                integral_ = 0.0;
                derivative_ = 0.0;
                feedforward_ = 0.0;
                feedback_ = 0.0;
                first_update_ = true;
            }

            void VelocityControllerWithFF::setPIDConfig(const PIDConfig &config)
            {
                pid_config_ = config;
            }

            void VelocityControllerWithFF::setFFConfig(const FeedforwardConfig &config)
            {
                ff_config_ = config;
            }

            double VelocityControllerWithFF::computeFeedforward() const
            {
                double ff = 0.0;

                // Velocity feedforward (compensate for damping)
                ff += ff_config_.kv * target_velocity_;

                // Acceleration feedforward (compensate for inertia)
                ff += ff_config_.ka * target_acceleration_;

                // Friction feedforward (compensate for coulomb friction)
                if (target_velocity_ > 0.0)
                {
                    ff += ff_config_.kf;
                }
                else if (target_velocity_ < 0.0)
                {
                    ff -= ff_config_.kf;
                }

                return ff;
            }

            // ========== VelocityProfileGenerator Implementation ==========

            VelocityProfileGenerator::VelocityProfileGenerator(const ProfileConfig &config)
                : config_(config)
            {
            }

            void VelocityProfileGenerator::setTarget(double target, double current)
            {
                target_velocity_ = target;
                current_velocity_ = current;
                current_acceleration_ = 0.0;
                current_jerk_ = 0.0;
            }

            double VelocityProfileGenerator::update(double dt)
            {
                if (dt <= 0.0)
                {
                    return current_velocity_;
                }

                if (config_.type == ProfileType::TRAPEZOIDAL)
                {
                    updateTrapezoidal(dt);
                }
                else
                {
                    updateSCurve(dt);
                }

                return current_velocity_;
            }

            void VelocityProfileGenerator::updateTrapezoidal(double dt)
            {
                double error = target_velocity_ - current_velocity_;

                if (std::abs(error) < 1e-6)
                {
                    // Already at target
                    current_acceleration_ = 0.0;
                    return;
                }

                // Determine desired acceleration direction
                double desired_accel = (error > 0.0) ? config_.max_acceleration : -config_.max_acceleration;

                // Check if we need to start decelerating to avoid overshoot
                // Using v² = v₀² + 2a·Δx, but for velocity: if we decelerate now, can we stop at target?
                double stopping_distance = (current_velocity_ * current_velocity_) / (2.0 * config_.max_acceleration);
                double target_stopping = (target_velocity_ * target_velocity_) / (2.0 * config_.max_acceleration);

                if (error > 0.0)
                {
                    // Moving toward higher velocity
                    // Start decelerating if we're close enough
                    double velocity_at_target_if_decel = std::sqrt(current_velocity_ * current_velocity_ -
                                                                    2.0 * config_.max_acceleration * error);
                    if (velocity_at_target_if_decel >= 0.0 && current_acceleration_ > 0.0)
                    {
                        // We might overshoot, start decelerating
                        desired_accel = -config_.max_acceleration;
                    }
                }
                else
                {
                    // Moving toward lower velocity
                    double velocity_at_target_if_accel = std::sqrt(current_velocity_ * current_velocity_ +
                                                                    2.0 * config_.max_acceleration * std::abs(error));
                    if (current_acceleration_ < 0.0)
                    {
                        // Check if we need to start accelerating (reducing deceleration)
                        desired_accel = config_.max_acceleration;
                    }
                }

                // Apply acceleration limit
                current_acceleration_ = desired_accel;

                // Update velocity
                current_velocity_ += current_acceleration_ * dt;

                // Clamp to target if very close
                if (std::abs(target_velocity_ - current_velocity_) < std::abs(current_acceleration_ * dt))
                {
                    current_velocity_ = target_velocity_;
                    current_acceleration_ = 0.0;
                }
            }

            void VelocityProfileGenerator::updateSCurve(double dt)
            {
                double error = target_velocity_ - current_velocity_;

                if (std::abs(error) < 1e-6)
                {
                    // Already at target
                    current_acceleration_ = 0.0;
                    current_jerk_ = 0.0;
                    return;
                }

                // Simple S-curve: use jerk to smoothly change acceleration
                double desired_accel_sign = (error > 0.0) ? 1.0 : -1.0;
                double desired_accel = desired_accel_sign * config_.max_acceleration;

                // Compute jerk to reach desired acceleration
                double accel_error = desired_accel - current_acceleration_;

                if (std::abs(accel_error) > 1e-6)
                {
                    current_jerk_ = std::clamp(accel_error / dt, -config_.max_jerk, config_.max_jerk);
                }
                else
                {
                    current_jerk_ = 0.0;
                }

                // Update acceleration with jerk
                current_acceleration_ += current_jerk_ * dt;
                current_acceleration_ = std::clamp(current_acceleration_,
                                                   -config_.max_acceleration,
                                                   config_.max_acceleration);

                // Update velocity
                current_velocity_ += current_acceleration_ * dt;

                // Clamp to target if very close
                if (std::abs(target_velocity_ - current_velocity_) < 1e-3)
                {
                    current_velocity_ = target_velocity_;
                    current_acceleration_ = 0.0;
                    current_jerk_ = 0.0;
                }
            }

            bool VelocityProfileGenerator::isComplete() const
            {
                return std::abs(target_velocity_ - current_velocity_) < 1e-6 &&
                       std::abs(current_acceleration_) < 1e-6;
            }

            void VelocityProfileGenerator::reset()
            {
                target_velocity_ = 0.0;
                current_velocity_ = 0.0;
                current_acceleration_ = 0.0;
                current_jerk_ = 0.0;
            }

            void VelocityProfileGenerator::setConfig(const ProfileConfig &config)
            {
                config_ = config;
            }

        } // namespace velocity
    }     // namespace control
} // namespace modules
