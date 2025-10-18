// /Users/danielpi/work/LumosMotorSim/src/modules/simulator/bldc_motor.h
#pragma once

#include <array>
#include <cmath>

namespace modules
{
    namespace simulator
    {
        // Physical constants
        constexpr double PI = 3.14159265358979323846;
        constexpr double COPPER_RESISTIVITY = 1.68e-8; // Ohm*m at 20°C
        constexpr double MU_0 = 4.0 * PI * 1e-7;       // Permeability of free space

        /**
         * Configuration structure for BLDC motor physical parameters
         */
        struct BldcMotorConfig
        {
            // ===== Electrical Parameters =====
            int num_phases = 3;            // Number of phases (typically 3)
            int num_pole_pairs = 4;        // Number of pole pairs (poles = 2 * pole_pairs)
            int num_slots = 12;            // Number of stator slots
            int turns_per_coil = 50;       // Number of turns in each coil
            double wire_diameter = 0.5e-3; // Wire diameter in meters (0.5mm)

            // Phase resistance (if 0, will be auto-calculated from wire properties)
            double phase_resistance = 0.0; // Ohms

            // Phase inductance (if 0, will be auto-calculated)
            double phase_inductance = 0.0; // Henries

            // ===== Magnetic Parameters =====
            double magnet_flux_linkage = 0.01; // Permanent magnet flux linkage (Weber-turns)
            double magnet_br = 1.2;            // Magnet remanence (Tesla) - typical NdFeB

            // Back-EMF constant (V/rad/s) - if 0, calculated from flux linkage
            double ke = 0.0;

            // Torque constant (Nm/A) - if 0, calculated from ke
            double kt = 0.0;

            // ===== Mechanical Parameters =====
            double rotor_inertia = 1e-5;        // Rotor moment of inertia (kg*m^2)
            double rotor_radius = 0.02;         // Rotor radius (m)
            double stator_inner_radius = 0.021; // Stator inner radius (m)
            double air_gap = 0.001;             // Air gap (m)
            double active_length = 0.03;        // Active length of motor (m)

            // Friction coefficients
            double viscous_friction = 1e-6; // Viscous friction coefficient (Nm/(rad/s))
            double coulomb_friction = 0.0;  // Coulomb friction torque (Nm)

            // ===== Cogging Torque Parameters =====
            bool enable_cogging = false;
            double cogging_amplitude = 0.0; // Amplitude of cogging torque (Nm)

            // ===== Material Properties =====
            double stator_permeability = 2000.0; // Relative permeability of stator steel

            /**
             * Calculate derived parameters based on physical properties
             */
            void calculateDerivedParameters()
            {
                // Calculate phase resistance if not provided
                if (phase_resistance <= 0.0)
                {
                    // Estimate wire length per phase (rough approximation)
                    double mean_turn_length = 2.0 * PI * stator_inner_radius + 2.0 * active_length;
                    double wire_length = mean_turn_length * turns_per_coil * (num_slots / num_phases);
                    double wire_area = PI * (wire_diameter / 2.0) * (wire_diameter / 2.0);
                    phase_resistance = COPPER_RESISTIVITY * wire_length / wire_area;
                }

                // Calculate phase inductance if not provided (very rough approximation)
                if (phase_inductance <= 0.0)
                {
                    // Simplified inductance calculation
                    double slot_area = PI * stator_inner_radius * active_length / num_slots;
                    double N_total = turns_per_coil * (num_slots / num_phases);
                    phase_inductance = MU_0 * stator_permeability * N_total * N_total *
                                       slot_area / air_gap;
                }

                // Calculate back-EMF constant if not provided
                if (ke <= 0.0)
                {
                    ke = magnet_flux_linkage * num_pole_pairs;
                }

                // Calculate torque constant if not provided
                // For 3-phase BLDC: kt = (3/2) * num_pole_pairs * flux_linkage
                if (kt <= 0.0)
                {
                    kt = 1.5 * num_pole_pairs * magnet_flux_linkage;
                }
            }
        };

        /**
         * State of the BLDC motor
         */
        struct BldcMotorState
        {
            // Mechanical state
            double theta = 0.0;      // Rotor electrical angle (radians)
            double theta_mech = 0.0; // Rotor mechanical angle (radians)
            double omega = 0.0;      // Rotor electrical angular velocity (rad/s)
            double omega_mech = 0.0; // Rotor mechanical angular velocity (rad/s)

            // Electrical state (per phase)
            std::array<double, 3> phase_currents = {0.0, 0.0, 0.0}; // Phase currents (A)
            std::array<double, 3> phase_voltages = {0.0, 0.0, 0.0}; // Applied phase voltages (V)
            std::array<double, 3> back_emf = {0.0, 0.0, 0.0};       // Back-EMF voltages (V)

            // Torques
            double electromagnetic_torque = 0.0; // Electromagnetic torque (Nm)
            double load_torque = 0.0;            // External load torque (Nm)
            double friction_torque = 0.0;        // Friction torque (Nm)
            double cogging_torque = 0.0;         // Cogging torque (Nm)

            // DQ frame (for FOC algorithms)
            double i_d = 0.0; // D-axis current (A)
            double i_q = 0.0; // Q-axis current (A)
            double v_d = 0.0; // D-axis voltage (V)
            double v_q = 0.0; // Q-axis voltage (V)
        };

        /**
         * BLDC Motor simulation model
         *
         * This class implements a simplified BLDC motor model suitable for
         * testing control algorithms (FOC, six-step commutation, etc.)
         *
         * The model includes:
         * - 3-phase electrical dynamics (R-L circuit with back-EMF)
         * - Electromagnetic torque generation
         * - Rigid body rotor dynamics
         * - Optional cogging torque
         * - Clarke and Park transforms for FOC
         */
        class BldcMotor
        {
        public:
            explicit BldcMotor(const BldcMotorConfig &config = BldcMotorConfig());

            /**
             * Step the motor simulation forward by dt seconds
             * @param dt Time step (seconds)
             */
            void step(double dt);

            /**
             * Set the applied phase voltages (for 3-phase control)
             * @param v_a Phase A voltage (V)
             * @param v_b Phase B voltage (V)
             * @param v_c Phase C voltage (V)
             */
            void setPhaseVoltages(double v_a, double v_b, double v_c);

            /**
             * Set the applied DQ voltages (for FOC control)
             * @param v_d D-axis voltage (V)
             * @param v_q Q-axis voltage (V)
             */
            void setDQVoltages(double v_d, double v_q);

            /**
             * Set external load torque
             * @param torque Load torque (Nm) - positive resists motion
             */
            void setLoadTorque(double torque);

            /**
             * Reset the motor state
             * @param theta_mech Initial mechanical angle (radians)
             * @param omega_mech Initial mechanical angular velocity (rad/s)
             */
            void reset(double theta_mech = 0.0, double omega_mech = 0.0);

            // Accessors
            const BldcMotorState &state() const { return state_; }
            const BldcMotorConfig &config() const { return config_; }

            // Convenience accessors for common measurements
            double position() const { return state_.theta_mech; }                // Mechanical angle (rad)
            double velocity() const { return state_.omega_mech; }                // Mechanical velocity (rad/s)
            double velocityRPM() const { return state_.omega_mech * 30.0 / PI; } // RPM
            double torque() const { return state_.electromagnetic_torque; }      // Torque (Nm)
            double current(int phase) const { return state_.phase_currents[phase]; }

            /**
             * Measure terminal voltage on a floating phase
             *
             * In six-step commutation, one phase is typically left floating (high-Z).
             * The voltage on this phase can be measured and equals the back-EMF when
             * the current is zero (floating state).
             *
             * This simulates what you would measure with a comparator or ADC on the
             * floating phase terminal relative to the neutral point.
             *
             * @param phase Phase number (0=A, 1=B, 2=C)
             * @return Terminal voltage (V) - equals back-EMF when floating
             */
            double measureFloatingPhaseVoltage(int phase) const;

        private:
            BldcMotorConfig config_;
            BldcMotorState state_;

            /**
             * Update back-EMF based on rotor position and velocity
             */
            void updateBackEMF();

            /**
             * Update phase currents using electrical dynamics
             * @param dt Time step
             */
            void updateCurrents(double dt);

            /**
             * Calculate electromagnetic torque from currents and position
             */
            void updateTorque();

            /**
             * Update rotor dynamics
             * @param dt Time step
             */
            void updateMechanics(double dt);

            /**
             * Calculate cogging torque based on position
             */
            double calculateCoggingTorque() const;

            /**
             * Clarke transform: abc -> alpha-beta
             */
            void clarkTransform(double a, double b, double c, double &alpha, double &beta) const;

            /**
             * Park transform: alpha-beta -> dq
             */
            void parkTransform(double alpha, double beta, double theta, double &d, double &q) const;

            /**
             * Inverse Park transform: dq -> alpha-beta
             */
            void inverseParkTransform(double d, double q, double theta, double &alpha, double &beta) const;

            /**
             * Inverse Clarke transform: alpha-beta -> abc
             */
            void inverseClarkTransform(double alpha, double beta, double &a, double &b, double &c) const;
        };

    } // namespace simulator
} // namespace modules
