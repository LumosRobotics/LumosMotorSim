// /Users/danielpi/work/LumosMotorSim/src/modules/simulator/bldc_motor.cpp
#include "bldc_motor.h"
#include <cmath>

namespace modules
{
    namespace simulator
    {
        BldcMotor::BldcMotor(const BldcMotorConfig &config)
            : config_(config)
        {
            // Calculate derived parameters
            config_.calculateDerivedParameters();
            reset();
        }

        void BldcMotor::step(double dt)
        {
            // 1. Update back-EMF based on current position/velocity
            updateBackEMF();

            // 2. Update phase currents based on applied voltages and back-EMF
            updateCurrents(dt);

            // 3. Calculate electromagnetic torque
            updateTorque();

            // 4. Update mechanical state (position, velocity)
            updateMechanics(dt);
        }

        void BldcMotor::setPhaseVoltages(double v_a, double v_b, double v_c)
        {
            state_.phase_voltages[0] = v_a;
            state_.phase_voltages[1] = v_b;
            state_.phase_voltages[2] = v_c;

            // Also update DQ voltages via transforms
            double v_alpha, v_beta;
            clarkTransform(v_a, v_b, v_c, v_alpha, v_beta);
            parkTransform(v_alpha, v_beta, state_.theta, state_.v_d, state_.v_q);
        }

        void BldcMotor::setDQVoltages(double v_d, double v_q)
        {
            state_.v_d = v_d;
            state_.v_q = v_q;

            // Convert back to phase voltages
            double v_alpha, v_beta;
            inverseParkTransform(v_d, v_q, state_.theta, v_alpha, v_beta);
            inverseClarkTransform(v_alpha, v_beta,
                                  state_.phase_voltages[0],
                                  state_.phase_voltages[1],
                                  state_.phase_voltages[2]);
        }

        void BldcMotor::setLoadTorque(double torque)
        {
            state_.load_torque = torque;
        }

        void BldcMotor::reset(double theta_mech, double omega_mech)
        {
            state_.theta_mech = theta_mech;
            state_.omega_mech = omega_mech;

            // Convert mechanical to electrical
            state_.theta = theta_mech * config_.num_pole_pairs;
            state_.omega = omega_mech * config_.num_pole_pairs;

            // Reset currents and voltages
            state_.phase_currents.fill(0.0);
            state_.phase_voltages.fill(0.0);
            state_.back_emf.fill(0.0);

            // Reset torques
            state_.electromagnetic_torque = 0.0;
            state_.load_torque = 0.0;
            state_.friction_torque = 0.0;
            state_.cogging_torque = 0.0;

            // Reset DQ
            state_.i_d = 0.0;
            state_.i_q = 0.0;
            state_.v_d = 0.0;
            state_.v_q = 0.0;
        }

        void BldcMotor::updateBackEMF()
        {
            // Back-EMF is sinusoidal with electrical angle
            // e_a = ke * omega * sin(theta)
            // e_b = ke * omega * sin(theta - 2π/3)
            // e_c = ke * omega * sin(theta - 4π/3)

            state_.back_emf[0] = config_.ke * state_.omega * std::sin(state_.theta);
            state_.back_emf[1] = config_.ke * state_.omega * std::sin(state_.theta - 2.0 * PI / 3.0);
            state_.back_emf[2] = config_.ke * state_.omega * std::sin(state_.theta - 4.0 * PI / 3.0);
        }

        void BldcMotor::updateCurrents(double dt)
        {
            // For each phase: L * di/dt = V - R*i - e_backEMF
            // Using forward Euler: i(t+dt) = i(t) + dt/L * (V - R*i - e)

            const double R = config_.phase_resistance;
            const double L = config_.phase_inductance;

            for (int phase = 0; phase < 3; ++phase)
            {
                double V = state_.phase_voltages[phase];
                double i = state_.phase_currents[phase];
                double e = state_.back_emf[phase];

                // Current derivative
                double di_dt = (V - R * i - e) / L;

                // Update current (Euler integration)
                state_.phase_currents[phase] = i + di_dt * dt;
            }

            // Update DQ currents via transforms
            double i_alpha, i_beta;
            clarkTransform(state_.phase_currents[0],
                           state_.phase_currents[1],
                           state_.phase_currents[2],
                           i_alpha, i_beta);

            parkTransform(i_alpha, i_beta, state_.theta, state_.i_d, state_.i_q);
        }

        void BldcMotor::updateTorque()
        {
            // Electromagnetic torque calculation
            // For sinusoidal BLDC (PMSM): T = (3/2) * p * λ_m * i_q
            // Simplified: T = kt * i_q
            // Or in abc frame: T = (e_a*i_a + e_b*i_b + e_c*i_c) / omega

            // Method 1: Using DQ frame (cleaner for FOC)
            state_.electromagnetic_torque = config_.kt * state_.i_q;

            // Method 2: Using ABC frame (cross-check)
            // if (std::abs(state_.omega) > 1e-6)
            // {
            //     double power = state_.back_emf[0] * state_.phase_currents[0] +
            //                    state_.back_emf[1] * state_.phase_currents[1] +
            //                    state_.back_emf[2] * state_.phase_currents[2];
            //     state_.electromagnetic_torque = power / state_.omega;
            // }

            // Add cogging torque if enabled
            if (config_.enable_cogging)
            {
                state_.cogging_torque = calculateCoggingTorque();
            }
            else
            {
                state_.cogging_torque = 0.0;
            }
        }

        void BldcMotor::updateMechanics(double dt)
        {
            // Calculate friction torque
            // Viscous: T_v = b * omega
            // Coulomb: T_c = T_coulomb * sign(omega)
            double viscous = config_.viscous_friction * state_.omega_mech;
            double coulomb = 0.0;
            if (state_.omega_mech > 0.0)
                coulomb = config_.coulomb_friction;
            else if (state_.omega_mech < 0.0)
                coulomb = -config_.coulomb_friction;

            state_.friction_torque = viscous + coulomb;

            // Net torque on rotor
            double T_net = state_.electromagnetic_torque +
                           state_.cogging_torque -
                           state_.load_torque -
                           state_.friction_torque;

            // Angular acceleration: alpha = T / J
            double alpha = T_net / config_.rotor_inertia;

            // Update mechanical velocity (Euler integration)
            state_.omega_mech += alpha * dt;

            // Update mechanical position
            state_.theta_mech += state_.omega_mech * dt;

            // Normalize mechanical angle to [0, 2π)
            state_.theta_mech = std::fmod(state_.theta_mech, 2.0 * PI);
            if (state_.theta_mech < 0.0)
                state_.theta_mech += 2.0 * PI;

            // Update electrical angle and velocity
            state_.theta = state_.theta_mech * config_.num_pole_pairs;
            state_.omega = state_.omega_mech * config_.num_pole_pairs;

            // Normalize electrical angle to [0, 2π)
            state_.theta = std::fmod(state_.theta, 2.0 * PI);
            if (state_.theta < 0.0)
                state_.theta += 2.0 * PI;
        }

        double BldcMotor::calculateCoggingTorque() const
        {
            // Simplified cogging torque model
            // Cogging has period of 2π / (num_slots * num_pole_pairs)
            // Multiple harmonics can exist, but we'll use fundamental

            int num_cogging_periods = config_.num_slots * config_.num_pole_pairs;
            double cogging_angle = state_.theta_mech * num_cogging_periods;

            return -config_.cogging_amplitude * std::sin(cogging_angle);
        }

        void BldcMotor::clarkTransform(double a, double b, double c, double &alpha, double &beta) const
        {
            // Clarke transform (power-invariant form)
            // alpha = a
            // beta = (a + 2*b) / sqrt(3)

            // Alternative amplitude-invariant form (more common):
            alpha = a;
            beta = (a + 2.0 * b) / std::sqrt(3.0);

            // Power-invariant form:
            // double K = std::sqrt(2.0 / 3.0);
            // alpha = K * (a - 0.5 * b - 0.5 * c);
            // beta = K * (std::sqrt(3.0) / 2.0) * (b - c);
        }

        void BldcMotor::parkTransform(double alpha, double beta, double theta, double &d, double &q) const
        {
            // Park transform
            // d =  alpha * cos(theta) + beta * sin(theta)
            // q = -alpha * sin(theta) + beta * cos(theta)

            double cos_theta = std::cos(theta);
            double sin_theta = std::sin(theta);

            d = alpha * cos_theta + beta * sin_theta;
            q = -alpha * sin_theta + beta * cos_theta;
        }

        void BldcMotor::inverseParkTransform(double d, double q, double theta, double &alpha, double &beta) const
        {
            // Inverse Park transform
            // alpha = d * cos(theta) - q * sin(theta)
            // beta  = d * sin(theta) + q * cos(theta)

            double cos_theta = std::cos(theta);
            double sin_theta = std::sin(theta);

            alpha = d * cos_theta - q * sin_theta;
            beta = d * sin_theta + q * cos_theta;
        }

        void BldcMotor::inverseClarkTransform(double alpha, double beta, double &a, double &b, double &c) const
        {
            // Inverse Clarke transform
            // a = alpha
            // b = -0.5 * alpha + sqrt(3)/2 * beta
            // c = -0.5 * alpha - sqrt(3)/2 * beta

            a = alpha;
            b = -0.5 * alpha + (std::sqrt(3.0) / 2.0) * beta;
            c = -0.5 * alpha - (std::sqrt(3.0) / 2.0) * beta;
        }

        double BldcMotor::measureFloatingPhaseVoltage(int phase) const
        {
            // When a phase is floating (high impedance), the current should be ~0
            // and the terminal voltage equals the back-EMF voltage
            //
            // In reality, with star-connected motor:
            // V_terminal = V_neutral + e_backEMF
            //
            // For a balanced 3-phase system, V_neutral = (V_a + V_b + V_c) / 3
            // where V_x are the terminal voltages
            //
            // When phase is floating (I ≈ 0):
            // V_phase_terminal ≈ e_backEMF + V_neutral
            //
            // For six-step commutation, the floating phase voltage relative to neutral
            // is what you measure for zero-crossing detection

            // Calculate neutral voltage (average of three phase terminals)
            // When one phase is floating and two are driven, this gives the virtual neutral
            double v_neutral = (state_.phase_voltages[0] +
                                state_.phase_voltages[1] +
                                state_.phase_voltages[2]) / 3.0;

            // Terminal voltage = back-EMF + voltage drop due to current
            // For floating phase, current should be ~0, so:
            // V_terminal ≈ back_emf + V_neutral

            return state_.back_emf[phase] + v_neutral;
        }

    } // namespace simulator
} // namespace modules
