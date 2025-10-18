#include "modules/simulator/bldc_motor.h"
#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

using namespace modules::simulator;

class BldcMotorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Default motor configuration for testing
        config_.num_pole_pairs = 4;
        config_.phase_resistance = 0.5;    // 0.5 Ohm
        config_.phase_inductance = 0.001;  // 1 mH
        config_.magnet_flux_linkage = 0.01;
        config_.rotor_inertia = 1e-5;
        config_.viscous_friction = 1e-5;

        config_.calculateDerivedParameters();
    }

    BldcMotorConfig config_;
};

// ===== Configuration Tests =====

TEST_F(BldcMotorTest, ConfigurationDerivedParameters)
{
    // Test that ke and kt are calculated correctly
    EXPECT_GT(config_.ke, 0.0);
    EXPECT_GT(config_.kt, 0.0);

    // ke should be flux_linkage * num_pole_pairs
    EXPECT_NEAR(config_.ke, config_.magnet_flux_linkage * config_.num_pole_pairs, 1e-6);

    // kt should be 1.5 * num_pole_pairs * flux_linkage
    EXPECT_NEAR(config_.kt, 1.5 * config_.num_pole_pairs * config_.magnet_flux_linkage, 1e-6);
}

TEST_F(BldcMotorTest, ConfigurationAutoCalculation)
{
    BldcMotorConfig auto_config;
    auto_config.num_pole_pairs = 7;
    auto_config.num_slots = 12;
    auto_config.turns_per_coil = 50;
    auto_config.wire_diameter = 0.5e-3;
    auto_config.stator_inner_radius = 0.02;
    auto_config.active_length = 0.03;
    auto_config.magnet_flux_linkage = 0.005;

    auto_config.calculateDerivedParameters();

    // Should auto-calculate resistance and inductance
    EXPECT_GT(auto_config.phase_resistance, 0.0);
    EXPECT_GT(auto_config.phase_inductance, 0.0);
}

// ===== Initialization Tests =====

TEST_F(BldcMotorTest, MotorInitialization)
{
    BldcMotor motor(config_);

    // Initial state should be zero
    EXPECT_DOUBLE_EQ(motor.position(), 0.0);
    EXPECT_DOUBLE_EQ(motor.velocity(), 0.0);
    EXPECT_DOUBLE_EQ(motor.torque(), 0.0);

    for (int i = 0; i < 3; i++)
    {
        EXPECT_DOUBLE_EQ(motor.current(i), 0.0);
    }
}

TEST_F(BldcMotorTest, MotorReset)
{
    BldcMotor motor(config_);

    // Apply DQ voltage to build up speed (more stable)
    motor.setDQVoltages(0.0, 15.0);
    for (int i = 0; i < 200; i++)
    {
        motor.step(0.0001);
    }

    // Should have non-zero state
    EXPECT_GT(std::abs(motor.velocity()), 0.1);

    // Reset
    motor.reset();

    // Should be back to zero
    EXPECT_DOUBLE_EQ(motor.position(), 0.0);
    EXPECT_DOUBLE_EQ(motor.velocity(), 0.0);
}

TEST_F(BldcMotorTest, MotorResetWithInitialConditions)
{
    BldcMotor motor(config_);

    double initial_angle = PI / 4.0;  // 45 degrees
    double initial_velocity = 10.0;    // 10 rad/s

    motor.reset(initial_angle, initial_velocity);

    EXPECT_NEAR(motor.position(), initial_angle, 1e-6);
    EXPECT_NEAR(motor.velocity(), initial_velocity, 1e-6);
}

// ===== Back-EMF Tests =====

TEST_F(BldcMotorTest, BackEMFGeneration)
{
    BldcMotor motor(config_);

    // Set initial velocity and position where sin(theta) != 0
    motor.reset(PI / (4.0 * config_.num_pole_pairs), 100.0);  // 100 rad/s mechanical

    // Step once to update back-EMF (motor.step calls updateBackEMF internally)
    motor.step(0.00001);

    const auto &state = motor.state();

    // Back-EMF should be non-zero at this position
    EXPECT_NE(state.back_emf[0], 0.0);

    // Back-EMF magnitude should be proportional to velocity
    // e = ke * omega_elec * sin(theta_elec)
    double expected_magnitude = config_.ke * state.omega;  // electrical omega
    EXPECT_LT(std::abs(state.back_emf[0]), expected_magnitude * 1.1);
}

TEST_F(BldcMotorTest, BackEMFSinusoidal)
{
    BldcMotor motor(config_);

    // Set constant velocity
    motor.reset(0.0, 50.0);

    // Check back-EMF at different positions
    std::vector<double> emf_a_values;

    for (double angle = 0.0; angle < 2.0 * PI; angle += PI / 6.0)
    {
        motor.reset(angle / config_.num_pole_pairs, 50.0);  // Mechanical angle
        motor.step(0.0);
        emf_a_values.push_back(motor.state().back_emf[0]);
    }

    // Check that we have both positive and negative values (sinusoidal)
    bool has_positive = false;
    bool has_negative = false;

    for (double emf : emf_a_values)
    {
        if (emf > 0.0) has_positive = true;
        if (emf < 0.0) has_negative = true;
    }

    EXPECT_TRUE(has_positive);
    EXPECT_TRUE(has_negative);
}

// ===== Electrical Dynamics Tests =====

TEST_F(BldcMotorTest, CurrentRiseWithVoltage)
{
    BldcMotor motor(config_);

    // Apply constant voltage
    motor.setPhaseVoltages(10.0, 0.0, 0.0);

    double initial_current = motor.current(0);

    // Step simulation
    motor.step(0.001);

    double final_current = motor.current(0);

    // Current should rise (L * di/dt = V - R*i - e)
    // At zero velocity, e=0, so current should increase
    EXPECT_GT(final_current, initial_current);
}

TEST_F(BldcMotorTest, CurrentDecayWithoutVoltage)
{
    BldcMotor motor(config_);

    // Apply voltage to build up current (at zero speed for simplicity)
    motor.setPhaseVoltages(5.0, -2.5, -2.5);
    for (int i = 0; i < 20; i++)
    {
        motor.step(0.0001);
    }

    double current_with_voltage = std::abs(motor.current(0));
    EXPECT_GT(current_with_voltage, 0.1);  // Should have built up some current

    // Remove voltage
    motor.setPhaseVoltages(0.0, 0.0, 0.0);
    motor.step(0.0001);

    double current_without_voltage = std::abs(motor.current(0));

    // Current magnitude should decay
    EXPECT_LT(current_without_voltage, current_with_voltage);
}

// ===== Torque Generation Tests =====

TEST_F(BldcMotorTest, TorqueFromCurrent)
{
    BldcMotor motor(config_);

    // Apply DQ voltages to generate torque
    motor.setDQVoltages(0.0, 10.0);  // Only q-axis voltage

    // Step to build current
    for (int i = 0; i < 50; i++)
    {
        motor.step(0.0001);
    }

    // Should have positive torque
    EXPECT_GT(motor.torque(), 0.0);

    // Torque should be approximately kt * i_q
    double i_q = motor.state().i_q;
    double expected_torque = config_.kt * i_q;

    EXPECT_NEAR(motor.torque(), expected_torque, std::abs(expected_torque) * 0.1);
}

TEST_F(BldcMotorTest, TorqueProportionalToIq)
{
    BldcMotor motor(config_);

    // Apply different q-axis currents and measure torque
    std::vector<double> iq_values = {1.0, 2.0, 3.0};
    std::vector<double> torque_values;

    for (double i_q_target : iq_values)
    {
        motor.reset();
        motor.setDQVoltages(0.0, i_q_target * 10.0);  // Higher voltage for better control

        // Let current build up to steady state
        for (int i = 0; i < 200; i++)
        {
            motor.step(0.0001);
        }

        torque_values.push_back(std::abs(motor.torque()));  // Use absolute value
    }

    // Torque magnitude should increase with i_q
    EXPECT_GT(torque_values[1], torque_values[0] * 0.9);  // Allow some tolerance
    EXPECT_GT(torque_values[2], torque_values[1] * 0.9);
}

// ===== Mechanical Dynamics Tests =====

TEST_F(BldcMotorTest, MotorAcceleration)
{
    BldcMotor motor(config_);

    // Apply torque
    motor.setDQVoltages(0.0, 20.0);

    double initial_velocity = motor.velocity();

    // Step simulation
    for (int i = 0; i < 100; i++)
    {
        motor.step(0.0001);
    }

    double final_velocity = motor.velocity();

    // Motor should accelerate
    EXPECT_GT(final_velocity, initial_velocity);
}

TEST_F(BldcMotorTest, PositionIntegration)
{
    // Test that position advances when motor is running
    BldcMotor motor(config_);

    double initial_position = motor.position();

    // Apply torque to spin the motor
    motor.setDQVoltages(0.0, 10.0);

    for (int i = 0; i < 100; i++)
    {
        motor.step(0.0001);
    }

    double final_position = motor.position();

    // Position should have changed
    EXPECT_NE(final_position, initial_position);

    // Motor should be spinning
    EXPECT_GT(std::abs(motor.velocity()), 0.0);
}

TEST_F(BldcMotorTest, ViscousFriction)
{
    // Create two motors - one with friction, one without
    config_.viscous_friction = 1e-4;
    BldcMotor motor_with_friction(config_);

    config_.viscous_friction = 0.0;
    BldcMotor motor_without_friction(config_);

    // Spin up both motors identically
    for (int i = 0; i < 100; i++)
    {
        motor_with_friction.setDQVoltages(0.0, 10.0);
        motor_without_friction.setDQVoltages(0.0, 10.0);
        motor_with_friction.step(0.0001);
        motor_without_friction.step(0.0001);
    }

    // Record velocities
    double vel_with_friction = motor_with_friction.velocity();
    double vel_without_friction = motor_without_friction.velocity();

    // Motor with friction should be slower
    EXPECT_LT(std::abs(vel_with_friction), std::abs(vel_without_friction));
}

// ===== Cogging Torque Tests =====

TEST_F(BldcMotorTest, CoggingTorque)
{
    config_.enable_cogging = true;
    config_.cogging_amplitude = 0.01;  // 10 mNm
    config_.num_slots = 12;
    config_.num_pole_pairs = 7;

    BldcMotor motor(config_);

    // Step through ONE full mechanical revolution
    std::vector<double> cogging_values;
    int num_samples = 50;

    for (int i = 0; i < num_samples; i++)
    {
        double angle = (2.0 * PI * i) / num_samples;
        motor.reset(angle, 0.0);
        motor.step(0.00001);  // Small step to update state
        cogging_values.push_back(motor.state().cogging_torque);
    }

    // Cogging torque should vary - find max and min
    double max_cogging = *std::max_element(cogging_values.begin(), cogging_values.end());
    double min_cogging = *std::min_element(cogging_values.begin(), cogging_values.end());

    // Should have variation around zero
    EXPECT_GT(max_cogging, 0.001);
    EXPECT_LT(min_cogging, -0.001);
    EXPECT_NEAR(max_cogging, config_.cogging_amplitude, config_.cogging_amplitude * 0.2);
}

// ===== Coordinate Transform Tests =====

TEST_F(BldcMotorTest, DQVoltageToPhaseVoltage)
{
    BldcMotor motor(config_);

    // Set DQ voltages
    motor.setDQVoltages(5.0, 10.0);

    const auto &state = motor.state();

    // Should update both DQ and phase voltages
    EXPECT_DOUBLE_EQ(state.v_d, 5.0);
    EXPECT_DOUBLE_EQ(state.v_q, 10.0);

    // Phase voltages should be non-zero
    EXPECT_NE(state.phase_voltages[0], 0.0);
}

TEST_F(BldcMotorTest, PhaseVoltageToDQ)
{
    BldcMotor motor(config_);

    // Set phase voltages
    motor.setPhaseVoltages(10.0, -5.0, -5.0);

    const auto &state = motor.state();

    // Phase voltages should be set
    EXPECT_DOUBLE_EQ(state.phase_voltages[0], 10.0);
    EXPECT_DOUBLE_EQ(state.phase_voltages[1], -5.0);
    EXPECT_DOUBLE_EQ(state.phase_voltages[2], -5.0);

    // DQ voltages should be calculated
    // (exact values depend on rotor position)
}

// ===== Floating Phase Voltage Tests =====

TEST_F(BldcMotorTest, FloatingPhaseVoltage)
{
    BldcMotor motor(config_);

    // Set initial velocity for back-EMF
    motor.reset(0.0, 50.0);

    // Set voltages (phase B floating - zero voltage)
    motor.setPhaseVoltages(12.0, 0.0, -12.0);
    motor.step(0.0);

    // Measure floating phase voltage
    double v_floating = motor.measureFloatingPhaseVoltage(1);  // Phase B

    // Should be close to back-EMF
    EXPECT_NE(v_floating, 0.0);
}

// ===== Load Torque Tests =====

TEST_F(BldcMotorTest, LoadTorqueEffect)
{
    BldcMotor motor(config_);

    // Apply driving voltage
    motor.setDQVoltages(0.0, 20.0);

    // Run without load
    for (int i = 0; i < 100; i++)
    {
        motor.step(0.0001);
    }
    double velocity_no_load = motor.velocity();

    // Reset and add load torque
    motor.reset();
    motor.setLoadTorque(0.05);  // 50 mNm load
    motor.setDQVoltages(0.0, 20.0);

    for (int i = 0; i < 100; i++)
    {
        motor.step(0.0001);
    }
    double velocity_with_load = motor.velocity();

    // Velocity should be lower with load
    EXPECT_LT(velocity_with_load, velocity_no_load);
}

// ===== Electrical/Mechanical Angle Relationship =====

TEST_F(BldcMotorTest, ElectricalMechanicalAngleRelation)
{
    BldcMotor motor(config_);

    double theta_mech = PI / 2.0;  // 90 degrees mechanical
    motor.reset(theta_mech, 0.0);
    motor.step(0.0);

    const auto &state = motor.state();

    // Electrical angle = mechanical angle * pole_pairs
    double expected_theta_elec = theta_mech * config_.num_pole_pairs;

    // Normalize to [0, 2π)
    while (expected_theta_elec >= 2.0 * PI) expected_theta_elec -= 2.0 * PI;

    EXPECT_NEAR(state.theta, expected_theta_elec, 1e-6);
}

// ===== Stability Tests =====

TEST_F(BldcMotorTest, NumericalStability)
{
    BldcMotor motor(config_);

    // Run for many steps with realistic voltages
    motor.setDQVoltages(0.0, 10.0);

    bool stable = true;
    for (int i = 0; i < 10000; i++)
    {
        motor.step(0.0001);

        // Check for NaN or infinity
        if (std::isnan(motor.velocity()) || std::isinf(motor.velocity()))
        {
            stable = false;
            break;
        }
    }

    EXPECT_TRUE(stable);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
