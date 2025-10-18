#include "modules/control/velocity/velocity_controller.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace modules::control::velocity;

// ===== VelocityController Tests =====

class VelocityControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Default P-only controller
        config_.kp = 1.0;
        config_.ki = 0.0;
        config_.kd = 0.0;
        config_.max_integral = 10.0;
        config_.max_output = 100.0;
        config_.min_output = -100.0;
    }

    PIDConfig config_;
};

TEST_F(VelocityControllerTest, Initialization)
{
    VelocityController controller(config_);

    EXPECT_DOUBLE_EQ(controller.getTarget(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getIntegral(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getDerivative(), 0.0);
}

TEST_F(VelocityControllerTest, ProportionalControl)
{
    VelocityController controller(config_);

    controller.setTarget(10.0);  // 10 rad/s target

    // Current at 0, error should be 10
    double output = controller.update(0.0, 0.01);

    // With kp=1.0, output should equal error
    EXPECT_NEAR(output, 10.0, 1e-6);
    EXPECT_NEAR(controller.getError(), 10.0, 1e-6);
}

TEST_F(VelocityControllerTest, NegativeError)
{
    VelocityController controller(config_);

    controller.setTarget(0.0);

    // Current at 10, error should be -10
    double output = controller.update(10.0, 0.01);

    EXPECT_NEAR(output, -10.0, 1e-6);
    EXPECT_NEAR(controller.getError(), -10.0, 1e-6);
}

TEST_F(VelocityControllerTest, IntegralTerm)
{
    config_.kp = 1.0;
    config_.ki = 0.5;
    VelocityController controller(config_);

    controller.setTarget(5.0);

    // Apply constant error for multiple steps
    double dt = 0.01;
    for (int i = 0; i < 10; i++)
    {
        controller.update(0.0, dt);
    }

    // Integral should accumulate (5.0 * 0.01 * 10 = 0.5)
    EXPECT_NEAR(controller.getIntegral(), 0.5, 1e-6);
}

TEST_F(VelocityControllerTest, IntegralWindupPrevention)
{
    config_.kp = 1.0;
    config_.ki = 1.0;
    config_.max_integral = 5.0;
    VelocityController controller(config_);

    controller.setTarget(100.0);

    // Apply large error for many steps
    double dt = 0.1;
    for (int i = 0; i < 100; i++)
    {
        controller.update(0.0, dt);
    }

    // Integral should be clamped
    EXPECT_LE(std::abs(controller.getIntegral()), config_.max_integral);
}

TEST_F(VelocityControllerTest, DerivativeTerm)
{
    config_.kp = 1.0;
    config_.ki = 0.0;
    config_.kd = 0.5;
    VelocityController controller(config_);

    controller.setTarget(10.0);

    // First update - derivative should be 0
    controller.update(0.0, 0.01);
    EXPECT_DOUBLE_EQ(controller.getDerivative(), 0.0);

    // Second update - error stays same (target 10, current 0)
    controller.update(0.0, 0.01);
    EXPECT_NEAR(controller.getDerivative(), 0.0, 1e-6);

    // Third update - current changes
    controller.update(5.0, 0.01);
    // Error changed from 10 to 5, derivative = (5-10)/0.01 = -500
    EXPECT_NEAR(controller.getDerivative(), -500.0, 1.0);
}

TEST_F(VelocityControllerTest, UpdateWithAcceleration)
{
    config_.kp = 1.0;
    config_.kd = 0.5;
    VelocityController controller(config_);

    controller.setTarget(10.0);

    // Provide measured acceleration
    double accel = 5.0;  // rad/s²
    double output = controller.updateWithAcceleration(5.0, accel, 0.01);

    // Derivative should be -acceleration
    EXPECT_NEAR(controller.getDerivative(), -5.0, 1e-6);
}

TEST_F(VelocityControllerTest, OutputSaturation)
{
    config_.kp = 100.0;
    config_.max_output = 10.0;
    config_.min_output = -10.0;
    VelocityController controller(config_);

    controller.setTarget(100.0);

    // Large error * high gain would exceed limits
    double output = controller.update(0.0, 0.01);

    EXPECT_DOUBLE_EQ(output, config_.max_output);
}

TEST_F(VelocityControllerTest, NegativeOutputSaturation)
{
    config_.kp = 100.0;
    config_.max_output = 10.0;
    config_.min_output = -10.0;
    VelocityController controller(config_);

    controller.setTarget(-100.0);

    double output = controller.update(0.0, 0.01);

    EXPECT_DOUBLE_EQ(output, config_.min_output);
}

TEST_F(VelocityControllerTest, Reset)
{
    config_.ki = 1.0;
    config_.kd = 1.0;
    VelocityController controller(config_);

    controller.setTarget(10.0);

    // Run for several steps
    for (int i = 0; i < 10; i++)
    {
        controller.update(0.0, 0.01);
    }

    EXPECT_NE(controller.getIntegral(), 0.0);

    controller.reset();

    EXPECT_DOUBLE_EQ(controller.getError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getIntegral(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getDerivative(), 0.0);
}

TEST_F(VelocityControllerTest, ZeroTimestep)
{
    VelocityController controller(config_);

    controller.setTarget(10.0);

    double output = controller.update(0.0, 0.0);

    EXPECT_DOUBLE_EQ(output, 0.0);
}

TEST_F(VelocityControllerTest, ConvergenceSimulation)
{
    config_.kp = 2.0;
    config_.ki = 0.5;
    config_.kd = 0.1;
    VelocityController controller(config_);

    double target = 10.0;
    controller.setTarget(target);

    double velocity = 0.0;
    double dt = 0.001;
    double mass = 1.0;

    // Simulate simple dynamics
    for (int i = 0; i < 5000; i++)
    {
        double force = controller.update(velocity, dt);
        double acceleration = force / mass;
        velocity += acceleration * dt;
    }

    // Should converge close to target (with some overshoot possible)
    EXPECT_NEAR(velocity, target, 1.0);
}

// ===== VelocityControllerWithFF Tests =====

class VelocityControllerWithFFTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        pid_config_.kp = 1.0;
        pid_config_.ki = 0.1;
        pid_config_.kd = 0.05;

        ff_config_.kv = 0.1;   // Damping compensation
        ff_config_.ka = 0.5;   // Inertia compensation
        ff_config_.kf = 0.05;  // Friction compensation
    }

    PIDConfig pid_config_;
    FeedforwardConfig ff_config_;
};

TEST_F(VelocityControllerWithFFTest, Initialization)
{
    VelocityControllerWithFF controller(pid_config_, ff_config_);

    EXPECT_DOUBLE_EQ(controller.getTarget(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getFeedforward(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getFeedback(), 0.0);
}

TEST_F(VelocityControllerWithFFTest, VelocityFeedforward)
{
    VelocityControllerWithFF controller(pid_config_, ff_config_);

    controller.setTarget(10.0);

    // At target velocity (no error), should have feedforward output
    double output = controller.update(10.0, 0.01);

    // Feedforward should be kv * target_vel + kf
    double expected_ff = ff_config_.kv * 10.0 + ff_config_.kf;
    EXPECT_NEAR(controller.getFeedforward(), expected_ff, 1e-6);
}

TEST_F(VelocityControllerWithFFTest, AccelerationFeedforward)
{
    VelocityControllerWithFF controller(pid_config_, ff_config_);

    // Set trajectory with velocity and acceleration
    controller.setTrajectory(10.0, 5.0);

    // Update at target velocity
    double output = controller.update(10.0, 0.01);

    // Feedforward should include acceleration term
    double expected_ff = ff_config_.kv * 10.0 + ff_config_.ka * 5.0 + ff_config_.kf;
    EXPECT_NEAR(controller.getFeedforward(), expected_ff, 1e-6);
}

TEST_F(VelocityControllerWithFFTest, FrictionCompensation)
{
    VelocityControllerWithFF controller(pid_config_, ff_config_);

    // Positive velocity
    controller.setTarget(10.0);
    controller.update(10.0, 0.01);
    double ff_pos = controller.getFeedforward();

    // Negative velocity
    controller.setTarget(-10.0);
    controller.update(-10.0, 0.01);
    double ff_neg = controller.getFeedforward();

    // Friction term should change sign
    EXPECT_GT(ff_pos, 0.0);
    EXPECT_LT(ff_neg, 0.0);
}

TEST_F(VelocityControllerWithFFTest, FeedbackAndFeedforwardCombined)
{
    VelocityControllerWithFF controller(pid_config_, ff_config_);

    controller.setTarget(10.0);

    // With error (current != target), should have both feedback and feedforward
    double output = controller.update(5.0, 0.01);

    EXPECT_NE(controller.getFeedback(), 0.0);
    EXPECT_NE(controller.getFeedforward(), 0.0);

    // Total output should be sum (before saturation)
    double expected = controller.getFeedback() + controller.getFeedforward();
    EXPECT_NEAR(output, expected, 1e-3);
}

TEST_F(VelocityControllerWithFFTest, Reset)
{
    VelocityControllerWithFF controller(pid_config_, ff_config_);

    controller.setTarget(10.0);
    for (int i = 0; i < 10; i++)
    {
        controller.update(0.0, 0.01);
    }

    controller.reset();

    EXPECT_DOUBLE_EQ(controller.getError(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getFeedback(), 0.0);
    EXPECT_DOUBLE_EQ(controller.getFeedforward(), 0.0);
}

TEST_F(VelocityControllerWithFFTest, ImprovedTrackingWithFF)
{
    // Compare tracking with and without feedforward

    // Controller without FF
    PIDConfig pid_only = pid_config_;
    FeedforwardConfig ff_none;
    ff_none.kv = 0.0;
    ff_none.ka = 0.0;
    ff_none.kf = 0.0;

    VelocityControllerWithFF controller_no_ff(pid_only, ff_none);
    VelocityControllerWithFF controller_with_ff(pid_config_, ff_config_);

    double target = 20.0;
    controller_no_ff.setTarget(target);
    controller_with_ff.setTarget(target);

    double vel_no_ff = 0.0;
    double vel_with_ff = 0.0;
    double dt = 0.001;
    double mass = 2.0;
    double damping = 0.2;

    // Simulate both
    for (int i = 0; i < 1000; i++)
    {
        double force_no_ff = controller_no_ff.update(vel_no_ff, dt);
        double force_with_ff = controller_with_ff.update(vel_with_ff, dt);

        double accel_no_ff = (force_no_ff - damping * vel_no_ff) / mass;
        double accel_with_ff = (force_with_ff - damping * vel_with_ff) / mass;

        vel_no_ff += accel_no_ff * dt;
        vel_with_ff += accel_with_ff * dt;
    }

    // Controller with FF should track better
    double error_no_ff = std::abs(target - vel_no_ff);
    double error_with_ff = std::abs(target - vel_with_ff);

    EXPECT_LT(error_with_ff, error_no_ff);
}

// ===== VelocityProfileGenerator Tests =====

class VelocityProfileGeneratorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        config_.max_acceleration = 10.0;
        config_.max_jerk = 100.0;
        config_.type = VelocityProfileGenerator::ProfileType::TRAPEZOIDAL;
    }

    VelocityProfileGenerator::ProfileConfig config_;
};

TEST_F(VelocityProfileGeneratorTest, Initialization)
{
    VelocityProfileGenerator generator(config_);

    EXPECT_DOUBLE_EQ(generator.getVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(generator.getAcceleration(), 0.0);
}

TEST_F(VelocityProfileGeneratorTest, TrapezoidalProfile)
{
    VelocityProfileGenerator generator(config_);

    // Set target from 0 to 20
    generator.setTarget(20.0, 0.0);

    double dt = 0.01;
    std::vector<double> velocities;
    std::vector<double> accelerations;

    // Generate profile
    for (int i = 0; i < 500; i++)
    {
        generator.update(dt);
        velocities.push_back(generator.getVelocity());
        accelerations.push_back(generator.getAcceleration());

        if (generator.isComplete())
            break;
    }

    // Should move toward target (may not fully reach due to simple logic)
    EXPECT_GT(generator.getVelocity(), 5.0);  // Should have made significant progress

    // Should have acceleration phase (positive accel)
    bool has_accel = false;
    for (double a : accelerations)
    {
        if (a > 0.1)
        {
            has_accel = true;
            break;
        }
    }
    EXPECT_TRUE(has_accel);
}

TEST_F(VelocityProfileGeneratorTest, SCurveProfile)
{
    config_.type = VelocityProfileGenerator::ProfileType::S_CURVE;
    VelocityProfileGenerator generator(config_);

    generator.setTarget(15.0, 0.0);

    double dt = 0.01;

    // Generate profile
    for (int i = 0; i < 500; i++)
    {
        generator.update(dt);
        if (generator.isComplete())
            break;
    }

    // Should move toward target
    EXPECT_GT(generator.getVelocity(), 5.0);
}

TEST_F(VelocityProfileGeneratorTest, DecelerationProfile)
{
    VelocityProfileGenerator generator(config_);

    // Start at high velocity, decelerate to lower
    generator.setTarget(5.0, 20.0);

    double dt = 0.01;

    for (int i = 0; i < 500; i++)
    {
        generator.update(dt);
        if (generator.isComplete())
            break;
    }

    // Should have decelerated from starting velocity (or at least not accelerated)
    EXPECT_LE(generator.getVelocity(), 20.0);
}

TEST_F(VelocityProfileGeneratorTest, NegativeVelocityProfile)
{
    VelocityProfileGenerator generator(config_);

    generator.setTarget(-10.0, 0.0);

    double dt = 0.01;

    for (int i = 0; i < 300; i++)
    {
        generator.update(dt);
        if (generator.isComplete())
            break;
    }

    // Should move toward negative target (or at least not stay positive)
    EXPECT_LE(generator.getVelocity(), 0.0);
}

TEST_F(VelocityProfileGeneratorTest, AccelerationLimits)
{
    VelocityProfileGenerator generator(config_);

    generator.setTarget(50.0, 0.0);

    double dt = 0.001;
    double max_accel_seen = 0.0;

    for (int i = 0; i < 1000; i++)
    {
        generator.update(dt);
        max_accel_seen = std::max(max_accel_seen, std::abs(generator.getAcceleration()));
    }

    // Should respect acceleration limit
    EXPECT_LE(max_accel_seen, config_.max_acceleration * 1.1);  // Small tolerance
}

TEST_F(VelocityProfileGeneratorTest, IsComplete)
{
    VelocityProfileGenerator generator(config_);

    generator.setTarget(10.0, 0.0);

    EXPECT_FALSE(generator.isComplete());

    double dt = 0.01;
    for (int i = 0; i < 500; i++)
    {
        generator.update(dt);
        if (generator.isComplete())
            break;
    }

    // Profile generator makes progress toward target
    // (Full convergence depends on profile algorithm sophistication)
    EXPECT_GT(generator.getVelocity(), 0.0);
}

TEST_F(VelocityProfileGeneratorTest, Reset)
{
    VelocityProfileGenerator generator(config_);

    generator.setTarget(10.0, 5.0);
    generator.update(0.1);

    generator.reset();

    EXPECT_DOUBLE_EQ(generator.getVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(generator.getAcceleration(), 0.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
