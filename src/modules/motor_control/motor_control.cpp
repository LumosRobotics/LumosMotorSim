// /Users/danielpi/work/LumosMotorSim/src/modules/motor_control/motor_control.cpp
#include "motor_control.h"

namespace modules
{
    namespace motor_control
    {
        // ========== SixStepController Implementation ==========

        SixStepController::SixStepController(const SixStepConfig &config)
            : config_(config)
        {
        }

        PWMOutput SixStepController::update(HallState hall_state, double duty_cycle)
        {
            current_hall_state_ = hall_state;
            duty_cycle = clampDutyCycle(duty_cycle);

            PWMOutput pwm;

            // Six-step commutation table
            // Hall sensors indicate rotor position, we energize two phases per sector
            switch (hall_state)
            {
            case HallState::STATE_1: // 001: A+ C-
                pwm.duty_a = duty_cycle;
                pwm.duty_b = 0.0;
                pwm.duty_c = config_.complementary_pwm ? (1.0 - duty_cycle) : 0.0;
                break;

            case HallState::STATE_2: // 010: B+ A-
                pwm.duty_a = config_.complementary_pwm ? (1.0 - duty_cycle) : 0.0;
                pwm.duty_b = duty_cycle;
                pwm.duty_c = 0.0;
                break;

            case HallState::STATE_3: // 011: B+ C-
                pwm.duty_a = 0.0;
                pwm.duty_b = duty_cycle;
                pwm.duty_c = config_.complementary_pwm ? (1.0 - duty_cycle) : 0.0;
                break;

            case HallState::STATE_4: // 100: C+ B-
                pwm.duty_a = 0.0;
                pwm.duty_b = config_.complementary_pwm ? (1.0 - duty_cycle) : 0.0;
                pwm.duty_c = duty_cycle;
                break;

            case HallState::STATE_5: // 101: C+ A-
                pwm.duty_a = config_.complementary_pwm ? (1.0 - duty_cycle) : 0.0;
                pwm.duty_b = 0.0;
                pwm.duty_c = duty_cycle;
                break;

            case HallState::STATE_6: // 110: A+ B-
                pwm.duty_a = duty_cycle;
                pwm.duty_b = config_.complementary_pwm ? (1.0 - duty_cycle) : 0.0;
                pwm.duty_c = 0.0;
                break;

            default: // Invalid hall state
                pwm.duty_a = 0.0;
                pwm.duty_b = 0.0;
                pwm.duty_c = 0.0;
                break;
            }

            return pwm;
        }

        void SixStepController::setConfig(const SixStepConfig &config)
        {
            config_ = config;
        }

        void SixStepController::pwmToVoltages(const PWMOutput &pwm,
                                             double &v_a, double &v_b, double &v_c) const
        {
            // In complementary PWM mode:
            // - High duty cycle means phase connected to V+ through PWM
            // - Low duty cycle means phase connected to GND through complementary switch
            // Average voltage = duty_cycle * V_bus
            //
            // In non-complementary mode:
            // - duty_cycle > 0 means PWM to V+
            // - duty_cycle = 0 means floating (high-Z)

            if (config_.complementary_pwm)
            {
                // For complementary PWM, the average voltage relative to virtual neutral is:
                // V_phase = (2*duty - 1) * V_bus / 2
                // But for simpler voltage output in six-step, we use:
                v_a = (pwm.duty_a - 0.5) * config_.bus_voltage;
                v_b = (pwm.duty_b - 0.5) * config_.bus_voltage;
                v_c = (pwm.duty_c - 0.5) * config_.bus_voltage;
            }
            else
            {
                // Non-complementary: simple duty cycle to voltage
                v_a = pwm.duty_a * config_.bus_voltage;
                v_b = pwm.duty_b * config_.bus_voltage;
                v_c = pwm.duty_c * config_.bus_voltage;
            }
        }

        double SixStepController::clampDutyCycle(double duty) const
        {
            return std::clamp(duty, config_.min_duty_cycle, config_.max_duty_cycle);
        }

        // ========== FOCController Implementation ==========

        FOCController::FOCController(const FOCConfig &config)
            : config_(config)
        {
        }

        PWMOutput FOCController::updateDQ(double v_d, double v_q, double theta_elec)
        {
            // Inverse Park transform: dq -> alpha-beta
            inverseParkTransform(v_d, v_q, theta_elec, v_alpha_, v_beta_);

            // Generate PWM from alpha-beta voltages
            return updateAlphaBeta(v_alpha_, v_beta_);
        }

        PWMOutput FOCController::updateAlphaBeta(double v_alpha, double v_beta)
        {
            v_alpha_ = v_alpha;
            v_beta_ = v_beta;

            // Generate PWM using selected method
            if (config_.use_svpwm)
            {
                return generateSVPWM(v_alpha, v_beta);
            }
            else
            {
                return generateSPWM(v_alpha, v_beta);
            }
        }

        void FOCController::setConfig(const FOCConfig &config)
        {
            config_ = config;
        }

        void FOCController::pwmToVoltages(const PWMOutput &pwm,
                                         double &v_a, double &v_b, double &v_c) const
        {
            // Convert duty cycles to phase voltages
            // For centered PWM: V_phase = (duty_cycle - 0.5) * V_bus
            v_a = (pwm.duty_a - 0.5) * config_.bus_voltage;
            v_b = (pwm.duty_b - 0.5) * config_.bus_voltage;
            v_c = (pwm.duty_c - 0.5) * config_.bus_voltage;
        }

        void FOCController::inverseParkTransform(double v_d, double v_q, double theta,
                                                double &v_alpha, double &v_beta) const
        {
            double cos_theta = std::cos(theta);
            double sin_theta = std::sin(theta);

            v_alpha = v_d * cos_theta - v_q * sin_theta;
            v_beta = v_d * sin_theta + v_q * cos_theta;
        }

        PWMOutput FOCController::generateSPWM(double v_alpha, double v_beta) const
        {
            // Inverse Clarke transform: alpha-beta -> abc
            double v_a = v_alpha;
            double v_b = -0.5 * v_alpha + (std::sqrt(3.0) / 2.0) * v_beta;
            double v_c = -0.5 * v_alpha - (std::sqrt(3.0) / 2.0) * v_beta;

            // Convert voltages to duty cycles
            // Normalize by V_bus and center around 0.5
            PWMOutput pwm;
            pwm.duty_a = clampDutyCycle(0.5 + v_a / config_.bus_voltage);
            pwm.duty_b = clampDutyCycle(0.5 + v_b / config_.bus_voltage);
            pwm.duty_c = clampDutyCycle(0.5 + v_c / config_.bus_voltage);

            return pwm;
        }

        PWMOutput FOCController::generateSVPWM(double v_alpha, double v_beta) const
        {
            // Space Vector PWM implementation
            // Determine the sector (1-6) based on angle
            double angle = std::atan2(v_beta, v_alpha);
            if (angle < 0.0)
                angle += TWO_PI;

            int sector = static_cast<int>(angle / (PI / 3.0)) + 1;
            if (sector > 6)
                sector = 6;

            // Calculate the magnitude and normalize
            double v_magnitude = std::sqrt(v_alpha * v_alpha + v_beta * v_beta);

            // Maximum voltage in SVPWM is V_bus / sqrt(3)
            double v_max = config_.bus_voltage / std::sqrt(3.0);

            // Saturation check
            if (v_magnitude > v_max)
            {
                double scale = v_max / v_magnitude;
                v_alpha *= scale;
                v_beta *= scale;
                v_magnitude = v_max;
            }

            // Calculate sector angle (angle within current 60-degree sector)
            double sector_angle = angle - (sector - 1) * (PI / 3.0);

            // Calculate time durations for active vectors (T1, T2) and zero vector (T0)
            // These are normalized by the PWM period
            double T1 = (v_magnitude / v_max) * std::sin(PI / 3.0 - sector_angle) / std::sin(PI / 3.0);
            double T2 = (v_magnitude / v_max) * std::sin(sector_angle) / std::sin(PI / 3.0);
            double T0 = 1.0 - T1 - T2;

            // Ensure times are valid
            T1 = std::clamp(T1, 0.0, 1.0);
            T2 = std::clamp(T2, 0.0, 1.0);
            T0 = std::clamp(T0, 0.0, 1.0);

            // Normalize if needed
            double T_sum = T0 + T1 + T2;
            if (T_sum > 1.0)
            {
                T0 /= T_sum;
                T1 /= T_sum;
                T2 /= T_sum;
            }

            // Calculate duty cycles based on sector
            // Using centered SVPWM (7-segment switching sequence)
            PWMOutput pwm;

            switch (sector)
            {
            case 1: // Sector 1: 0° to 60°
                pwm.duty_a = T1 + T2 + T0 / 2.0;
                pwm.duty_b = T2 + T0 / 2.0;
                pwm.duty_c = T0 / 2.0;
                break;
            case 2: // Sector 2: 60° to 120°
                pwm.duty_a = T1 + T0 / 2.0;
                pwm.duty_b = T1 + T2 + T0 / 2.0;
                pwm.duty_c = T0 / 2.0;
                break;
            case 3: // Sector 3: 120° to 180°
                pwm.duty_a = T0 / 2.0;
                pwm.duty_b = T1 + T2 + T0 / 2.0;
                pwm.duty_c = T2 + T0 / 2.0;
                break;
            case 4: // Sector 4: 180° to 240°
                pwm.duty_a = T0 / 2.0;
                pwm.duty_b = T1 + T0 / 2.0;
                pwm.duty_c = T1 + T2 + T0 / 2.0;
                break;
            case 5: // Sector 5: 240° to 300°
                pwm.duty_a = T2 + T0 / 2.0;
                pwm.duty_b = T0 / 2.0;
                pwm.duty_c = T1 + T2 + T0 / 2.0;
                break;
            case 6: // Sector 6: 300° to 360°
                pwm.duty_a = T1 + T2 + T0 / 2.0;
                pwm.duty_b = T0 / 2.0;
                pwm.duty_c = T1 + T0 / 2.0;
                break;
            default:
                pwm.duty_a = 0.5;
                pwm.duty_b = 0.5;
                pwm.duty_c = 0.5;
                break;
            }

            // Apply limits
            pwm.duty_a = clampDutyCycle(pwm.duty_a);
            pwm.duty_b = clampDutyCycle(pwm.duty_b);
            pwm.duty_c = clampDutyCycle(pwm.duty_c);

            return pwm;
        }

        double FOCController::clampDutyCycle(double duty) const
        {
            return std::clamp(duty, config_.min_duty_cycle, config_.max_duty_cycle);
        }

    } // namespace motor_control
} // namespace modules
