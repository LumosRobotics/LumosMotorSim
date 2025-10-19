// /Users/danielpi/work/LumosMotorSim/src/modules/motor_control/motor_control.h
#pragma once

#include <cmath>
#include <array>
#include <algorithm>

namespace modules
{
    namespace motor_control
    {
        constexpr double PI = 3.14159265358979323846;
        constexpr double TWO_PI = 2.0 * PI;

        /**
         * PWM output for 3-phase motor control
         * Values are duty cycles from 0.0 (0%) to 1.0 (100%)
         */
        struct PWMOutput
        {
            double duty_a = 0.0;  // Phase A duty cycle [0, 1]
            double duty_b = 0.0;  // Phase B duty cycle [0, 1]
            double duty_c = 0.0;  // Phase C duty cycle [0, 1]
        };

        /**
         * Hall sensor state (3-bit value from 1-6, 0 and 7 are invalid)
         */
        enum class HallState : uint8_t
        {
            STATE_1 = 1,  // 001
            STATE_2 = 2,  // 010
            STATE_3 = 3,  // 011
            STATE_4 = 4,  // 100
            STATE_5 = 5,  // 101
            STATE_6 = 6,  // 110
            INVALID = 0
        };

        /**
         * Six-step commutation configuration
         */
        struct SixStepConfig
        {
            double bus_voltage = 12.0;        // DC bus voltage (V)
            double max_duty_cycle = 0.95;     // Maximum duty cycle limit [0, 1]
            double min_duty_cycle = 0.0;      // Minimum duty cycle (deadband)
            bool complementary_pwm = true;    // Use complementary PWM (low-side inverted)
            double deadtime_us = 1.0;         // Deadtime in microseconds
        };

        /**
         * Six-Step Commutation Controller
         *
         * Implements trapezoidal (six-step) commutation for BLDC motors.
         * Uses hall sensor inputs to determine commutation state and applies
         * PWM duty cycle to the appropriate phase pair.
         *
         * Commutation Table:
         * Hall State | Energized Phases | Phase A | Phase B | Phase C
         * -----------|------------------|---------|---------|--------
         *     1      |     A+ C-        |   PWM   |   OFF   |   GND
         *     2      |     B+ A-        |   GND   |   PWM   |   OFF
         *     3      |     B+ C-        |   OFF   |   PWM   |   GND
         *     4      |     C+ B-        |   OFF   |   GND   |   PWM
         *     5      |     C+ A-        |   GND   |   OFF   |   PWM
         *     6      |     A+ B-        |   PWM   |   GND   |   OFF
         */
        class SixStepController
        {
        public:
            explicit SixStepController(const SixStepConfig &config = SixStepConfig());

            /**
             * Update commutation and generate PWM outputs
             * @param hall_state Current hall sensor state (1-6)
             * @param duty_cycle Commanded duty cycle [0, 1]
             * @return PWM output for each phase
             */
            PWMOutput update(HallState hall_state, double duty_cycle);

            /**
             * Set the control configuration
             */
            void setConfig(const SixStepConfig &config);

            /**
             * Get current configuration
             */
            const SixStepConfig &getConfig() const { return config_; }

            /**
             * Get current hall state
             */
            HallState getHallState() const { return current_hall_state_; }

            /**
             * Get current commutation sector (1-6, or 0 if invalid)
             */
            uint8_t getSector() const { return static_cast<uint8_t>(current_hall_state_); }

            /**
             * Convert PWM duty cycles to phase voltages
             * @param pwm PWM output
             * @param v_a Output phase A voltage
             * @param v_b Output phase B voltage
             * @param v_c Output phase C voltage
             */
            void pwmToVoltages(const PWMOutput &pwm, double &v_a, double &v_b, double &v_c) const;

        private:
            SixStepConfig config_;
            HallState current_hall_state_ = HallState::INVALID;

            /**
             * Apply duty cycle limits and saturation
             */
            double clampDutyCycle(double duty) const;
        };

        /**
         * Field-Oriented Control (FOC) configuration
         */
        struct FOCConfig
        {
            double bus_voltage = 12.0;          // DC bus voltage (V)
            double max_duty_cycle = 0.95;       // Maximum duty cycle [0, 1]
            double min_duty_cycle = 0.0;        // Minimum duty cycle

            // Space Vector PWM settings
            bool use_svpwm = true;              // Use SVPWM instead of SPWM
            double deadtime_us = 1.0;           // Deadtime in microseconds

            // Current control limits
            double max_phase_current = 10.0;    // Maximum phase current (A)
        };

        /**
         * Field-Oriented Control (FOC) Controller
         *
         * Implements Field-Oriented Control (also known as vector control) for BLDC/PMSM motors.
         * Converts DQ-frame voltage commands into 3-phase PWM outputs using either:
         * - Sinusoidal PWM (SPWM)
         * - Space Vector PWM (SVPWM)
         *
         * The FOC algorithm performs:
         * 1. Clarke transform: abc -> alpha-beta
         * 2. Park transform: alpha-beta -> dq (using rotor angle)
         * 3. PI control in DQ frame (done externally)
         * 4. Inverse Park transform: dq -> alpha-beta
         * 5. Inverse Clarke transform: alpha-beta -> abc
         * 6. PWM generation (SPWM or SVPWM)
         */
        class FOCController
        {
        public:
            explicit FOCController(const FOCConfig &config = FOCConfig());

            /**
             * Update FOC and generate PWM outputs from DQ voltages
             * @param v_d D-axis voltage command (V)
             * @param v_q Q-axis voltage command (V)
             * @param theta_elec Electrical rotor angle (radians)
             * @return PWM output for each phase
             */
            PWMOutput updateDQ(double v_d, double v_q, double theta_elec);

            /**
             * Update FOC and generate PWM outputs from alpha-beta voltages
             * @param v_alpha Alpha-axis voltage (V)
             * @param v_beta Beta-axis voltage (V)
             * @return PWM output for each phase
             */
            PWMOutput updateAlphaBeta(double v_alpha, double v_beta);

            /**
             * Set the control configuration
             */
            void setConfig(const FOCConfig &config);

            /**
             * Get current configuration
             */
            const FOCConfig &getConfig() const { return config_; }

            /**
             * Convert PWM duty cycles to phase voltages
             * @param pwm PWM output
             * @param v_a Output phase A voltage
             * @param v_b Output phase B voltage
             * @param v_c Output phase C voltage
             */
            void pwmToVoltages(const PWMOutput &pwm, double &v_a, double &v_b, double &v_c) const;

            /**
             * Get current alpha-beta voltages
             */
            void getAlphaBetaVoltages(double &v_alpha, double &v_beta) const
            {
                v_alpha = v_alpha_;
                v_beta = v_beta_;
            }

        private:
            FOCConfig config_;

            // Internal state
            double v_alpha_ = 0.0;
            double v_beta_ = 0.0;

            /**
             * Inverse Park transform: dq -> alpha-beta
             */
            void inverseParkTransform(double v_d, double v_q, double theta,
                                     double &v_alpha, double &v_beta) const;

            /**
             * Generate PWM using Sinusoidal PWM (SPWM)
             */
            PWMOutput generateSPWM(double v_alpha, double v_beta) const;

            /**
             * Generate PWM using Space Vector PWM (SVPWM)
             */
            PWMOutput generateSVPWM(double v_alpha, double v_beta) const;

            /**
             * Apply duty cycle limits
             */
            double clampDutyCycle(double duty) const;
        };

    } // namespace motor_control
} // namespace modules
