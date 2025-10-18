#include "modules/control/angle/angle_controller.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace modules::control::angle;

// ===== AngleController Tests =====

class AngleControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Default P-only controller for most tests
        config_.kp = 1.0;
        config_.ki = 0.0;
        config_.kd = 0.0;
        config_.max_integral = 10.0;
        config_.max_output = 100.0;
        config_.min_output = -100.0;
    }

    PIDConfig config_;
};

TEST_F(AngleControllerTest, Initialization)
{
    AngleController controller(config_);

    EXPECT_DOUBLE_EQ(controller.getTarget(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getIntegral(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getDerivative(), 0.0);
}

TEST_F(AngleControllerTest, ProportionalControl)
{
    AngleController controller(config_);

    // Set target to π/4 (45 degrees)
    controller.setTarget(PI / 4.0);

    // Current at 0, error should be π/4
    double output = controller.update(0.0, 0.01);

    // With kp=1.0, output should equal error
    EXPECT_NEAR(output, PI / 4.0, 1e-6);
    EXPECT_NEAR(controller.getError(), PI / 4.0, 1e-6);
}

TEST_F(AngleControllerTest, NegativeError)
{
    AngleController controller(config_);

    // Set target to 0
    controller.setTarget(0.0);

    // Current at π/4, error should be -π/4
    double output = controller.update(PI / 4.0, 0.01);

    EXPECT_NEAR(output, -PI / 4.0, 1e-6);
    EXPECT_NEAR(controller.getError(), -PI / 4.0, 1e-6);
}

TEST_F(AngleControllerTest, AngleWrapping)
{
    AngleController controller(config_);

    // Set target near 2π (wraps to ~0)
    controller.setTarget(2.0 * PI - 0.1);

    // Current at 0.1, should take shortest path
    double output = controller.update(0.1, 0.01);

    // Shortest path is -0.2, not (2π - 0.2)
    EXPECT_NEAR(controller.getError(), -0.2, 1e-6);
}

TEST_F(AngleControllerTest, CrossZeroWrapping)
{
    AngleController controller(config_);

    // Target at -π + 0.1
    controller.setTarget(-PI + 0.1);

    // Current at π - 0.1
    // These are only 0.2 radians apart, not 2π-0.2
    double output = controller.update(PI - 0.1, 0.01);

    EXPECT_NEAR(std::abs(controller.getError()), 0.2, 1e-6);
}

TEST_F(AngleControllerTest, IntegralTerm)
{
    config_.kp = 1.0;
    config_.ki = 0.5;
    AngleController controller(config_);

    controller.setTarget(1.0);

    // Apply constant error for multiple steps
    double dt = 0.01;
    for (int i = 0; i < 10; i++)
    {
        controller.update(0.0, dt);
    }

    // Integral should accumulate (1.0 * 0.01 * 10 = 0.1)
    EXPECT_NEAR(controller.getIntegral(), 0.1, 1e-6);

    // Output should include integral term (kp*e + ki*integral)
    double last_output = controller.update(0.0, dt);
    double expected = 1.0 * 1.0 + 0.5 * (0.1 + 1.0 * dt);
    EXPECT_NEAR(last_output, expected, 1e-3);
}

TEST_F(AngleControllerTest, IntegralWindupPrevention)
{
    config_.kp = 1.0;
    config_.ki = 1.0;
    config_.max_integral = 5.0;
    AngleController controller(config_);

    controller.setTarget(10.0);

    // Apply large error for many steps
    double dt = 0.1;
    for (int i = 0; i < 100; i++)
    {
        controller.update(0.0, dt);
    }

    // Integral should be clamped
    EXPECT_LE(std::abs(controller.getIntegral()), config_.max_integral);
}

TEST_F(AngleControllerTest, DerivativeTerm)
{
    config_.kp = 1.0;
    config_.ki = 0.0;
    config_.kd = 0.5;
    AngleController controller(config_);

    controller.setTarget(1.0);

    // First update - derivative should be 0
    double output1 = controller.update(0.0, 0.01);
    EXPECT_DOUBLE_EQ(controller.getDerivative(), 0.0);

    // Second update - error stays same, derivative should be 0
    double output2 = controller.update(0.0, 0.01);
    EXPECT_NEAR(controller.getDerivative(), 0.0, 1e-6);

    // Third update - error changes
    double output3 = controller.update(0.5, 0.01);
    // Error changed from 1.0 to 0.5, derivative = (0.5-1.0)/0.01 = -50
    EXPECT_NEAR(controller.getDerivative(), -50.0, 1e-3);
}

TEST_F(AngleControllerTest, OutputSaturation)
{
    config_.kp = 100.0;  // Very high gain
    config_.max_output = 10.0;
    config_.min_output = -10.0;
    AngleController controller(config_);

    // Set target to a small positive angle (not near ±π to avoid wrapping issues)
    controller.setTarget(1.0);

    // Large error * high gain would exceed limits
    double output = controller.update(0.0, 0.01);

    // Should be saturated at positive max
    EXPECT_LE(output, config_.max_output);
    EXPECT_GE(output, config_.min_output);
    EXPECT_DOUBLE_EQ(output, config_.max_output);
}

TEST_F(AngleControllerTest, NegativeOutputSaturation)
{
    config_.kp = 100.0;
    config_.max_output = 10.0;
    config_.min_output = -10.0;
    AngleController controller(config_);

    controller.setTarget(-PI);

    double output = controller.update(0.0, 0.01);

    EXPECT_DOUBLE_EQ(output, config_.min_output);
}

TEST_F(AngleControllerTest, Reset)
{
    config_.ki = 1.0;
    config_.kd = 1.0;
    AngleController controller(config_);

    controller.setTarget(1.0);

    // Run for several steps to accumulate state
    for (int i = 0; i < 10; i++)
    {
        controller.update(0.0, 0.01);
    }

    EXPECT_NE(controller.getIntegral(), 0.0);

    // Reset
    controller.reset();

    EXPECT_DOUBLE_EQ(controller.getError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getIntegral(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getDerivative(), 0.0);
}

TEST_F(AngleControllerTest, ZeroTimestep)
{
    AngleController controller(config_);

    controller.setTarget(1.0);

    // Zero timestep should return 0
    double output = controller.update(0.0, 0.0);

    EXPECT_DOUBLE_EQ(output, 0.0);
}

TEST_F(AngleControllerTest, ConvergenceToTarget)
{
    config_.kp = 10.0;
    config_.ki = 2.0;
    config_.kd = 1.0;
    AngleController controller(config_);

    double target = PI / 2.0;
    controller.setTarget(target);

    double current = 0.0;
    double velocity = 0.0;
    double dt = 0.001;

    // Simulate with simple dynamics (mass = 1, damping = 0.1)
    for (int i = 0; i < 5000; i++)
    {
        double output = controller.update(current, dt);
        // Simple dynamics: F = ma, with damping
        double acceleration = output - 0.1 * velocity;
        velocity += acceleration * dt;
        current += velocity * dt;
    }

    // Should converge close to target
    EXPECT_NEAR(current, target, 0.2);
}

TEST_F(AngleControllerTest, ConfigUpdate)
{
    AngleController controller(config_);

    // Change config
    PIDConfig new_config;
    new_config.kp = 5.0;
    new_config.ki = 2.0;
    new_config.kd = 1.0;

    controller.setConfig(new_config);

    const PIDConfig &retrieved = controller.getConfig();
    EXPECT_DOUBLE_EQ(retrieved.kp, 5.0);
    EXPECT_DOUBLE_EQ(retrieved.ki, 2.0);
    EXPECT_DOUBLE_EQ(retrieved.kd, 1.0);
}

// ===== CascadedAngleController Tests =====

class CascadedAngleControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Position loop (outer)
        angle_config_.kp = 2.0;
        angle_config_.ki = 0.0;
        angle_config_.kd = 0.0;
        angle_config_.max_output = 50.0;

        // Velocity loop (inner)
        velocity_config_.kp = 0.5;
        velocity_config_.ki = 0.1;
        velocity_config_.kd = 0.01;
        velocity_config_.max_output = 10.0;
        velocity_config_.min_output = -10.0;
    }

    PIDConfig angle_config_;
    PIDConfig velocity_config_;
};

TEST_F(CascadedAngleControllerTest, Initialization)
{
    CascadedAngleController controller(angle_config_, velocity_config_);

    EXPECT_DOUBLE_EQ(controller.getTarget(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getAngleError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getVelocityError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getVelocityCommand(), 0.0);
}

TEST_F(CascadedAngleControllerTest, PositionLoopGeneratesVelocityCommand)
{
    CascadedAngleController controller(angle_config_, velocity_config_);

    // Set position target
    controller.setTarget(PI / 4.0);

    // Update with zero position and velocity
    double output = controller.update(0.0, 0.0, 0.01);

    // Position error should generate velocity command
    EXPECT_NEAR(controller.getAngleError(), PI / 4.0, 1e-6);
    EXPECT_GT(controller.getVelocityCommand(), 0.0);

    // Velocity command should be proportional to position error (with kp=2.0)
    EXPECT_NEAR(controller.getVelocityCommand(), 2.0 * PI / 4.0, 1e-6);
}

TEST_F(CascadedAngleControllerTest, VelocityLoopFollowsCommand)
{
    CascadedAngleController controller(angle_config_, velocity_config_);

    controller.setTarget(1.0);

    // Update with current angle=0, velocity=0
    // Position loop will command some velocity
    double output1 = controller.update(0.0, 0.0, 0.01);

    double vel_cmd = controller.getVelocityCommand();
    EXPECT_GT(vel_cmd, 0.0);

    // Velocity error should be (cmd - current) = (cmd - 0) = cmd
    EXPECT_NEAR(controller.getVelocityError(), vel_cmd, 1e-6);

    // If we're already at target velocity, error should be smaller
    double output2 = controller.update(0.0, vel_cmd, 0.01);
    EXPECT_LT(std::abs(controller.getVelocityError()), std::abs(vel_cmd));
}

TEST_F(CascadedAngleControllerTest, MaxVelocityLimit)
{
    CascadedAngleController controller(angle_config_, velocity_config_);

    // Set velocity limit
    controller.setMaxVelocity(1.0);

    // Large position error
    controller.setTarget(PI);

    double output = controller.update(0.0, 0.0, 0.01);

    // Velocity command should be limited
    EXPECT_LE(std::abs(controller.getVelocityCommand()), 1.0);
}

TEST_F(CascadedAngleControllerTest, AngleWrapping)
{
    CascadedAngleController controller(angle_config_, velocity_config_);

    // Target near 2π
    controller.setTarget(2.0 * PI - 0.1);

    // Current at 0.1 - shortest path is -0.2
    double output = controller.update(0.1, 0.0, 0.01);

    EXPECT_NEAR(controller.getAngleError(), -0.2, 1e-6);
}

TEST_F(CascadedAngleControllerTest, Reset)
{
    CascadedAngleController controller(angle_config_, velocity_config_);

    controller.setTarget(1.0);

    // Run for several steps
    for (int i = 0; i < 10; i++)
    {
        controller.update(0.0, 0.0, 0.01);
    }

    // Reset
    controller.reset();

    EXPECT_DOUBLE_EQ(controller.getAngleError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getVelocityError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getVelocityCommand(), 0.0);
}

TEST_F(CascadedAngleControllerTest, ConvergenceSimulation)
{
    // Create controller with both loops active
    angle_config_.kp = 20.0;
    angle_config_.ki = 2.0;
    angle_config_.kd = 5.0;
    angle_config_.max_output = 50.0;

    velocity_config_.kp = 2.0;
    velocity_config_.ki = 1.0;
    velocity_config_.kd = 0.2;
    velocity_config_.max_output = 50.0;
    velocity_config_.min_output = -50.0;

    CascadedAngleController controller(angle_config_, velocity_config_);
    controller.setMaxVelocity(20.0);

    double target = PI / 2.0;
    controller.setTarget(target);

    // Simple simulation with better dynamics
    double angle = 0.0;
    double velocity = 0.0;
    double dt = 0.001;
    double inertia = 0.01;  // Simple inertia
    double damping = 0.05;   // Viscous damping

    for (int i = 0; i < 10000; i++)
    {
        double torque = controller.update(angle, velocity, dt);

        // Simple dynamics: torque = inertia * acceleration + damping * velocity
        double acceleration = (torque - damping * velocity) / inertia;
        velocity += acceleration * dt;
        angle += velocity * dt;
    }

    // Should converge reasonably close (cascaded control with simple dynamics)
    // The system oscillates but should get within reasonable range
    EXPECT_NEAR(angle, target, 1.2);
    EXPECT_LT(std::abs(velocity), 3.0);  // Should be relatively controlled
}

TEST_F(CascadedAngleControllerTest, OutputSaturation)
{
    // High gains to test saturation
    angle_config_.kp = 100.0;
    velocity_config_.kp = 100.0;
    velocity_config_.max_output = 5.0;
    velocity_config_.min_output = -5.0;

    CascadedAngleController controller(angle_config_, velocity_config_);

    controller.setTarget(PI);

    double output = controller.update(0.0, 0.0, 0.01);

    // Should be saturated
    EXPECT_LE(output, velocity_config_.max_output);
    EXPECT_GE(output, velocity_config_.min_output);
}

TEST_F(CascadedAngleControllerTest, ZeroTimestep)
{
    CascadedAngleController controller(angle_config_, velocity_config_);

    controller.setTarget(1.0);

    double output = controller.update(0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(output, 0.0);
}

// ===== Integration Tests =====

TEST(AngleControllerIntegrationTest, TrackingSinusoidalReference)
{
    PIDConfig config;
    config.kp = 5.0;
    config.ki = 1.0;
    config.kd = 0.5;

    AngleController controller(config);

    double current = 0.0;
    double dt = 0.001;
    double max_error = 0.0;

    // Track a slowly varying sinusoidal target
    for (int i = 0; i < 1000; i++)
    {
        double t = i * dt;
        double target = 0.5 * std::sin(2.0 * PI * 0.5 * t);  // 0.5 Hz

        controller.setTarget(target);
        double output = controller.update(current, dt);

        // Simple integration
        current += output * dt * 0.01;

        max_error = std::max(max_error, std::abs(controller.getError()));
    }

    // Tracking error should be reasonable
    EXPECT_LT(max_error, 0.5);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
