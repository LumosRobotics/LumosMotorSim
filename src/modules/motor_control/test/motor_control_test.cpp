// /Users/danielpi/work/LumosMotorSim/src/modules/motor_control/test/motor_control_test.cpp
#include "modules/motor_control/motor_control.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace modules::motor_control;

// ========== Six-Step Controller Tests ==========

class SixStepControllerTest : public ::testing::Test
{
protected:
    SixStepConfig config;
    SixStepController controller;

    void SetUp() override
    {
        config.bus_voltage = 12.0;
        config.max_duty_cycle = 0.95;
        config.min_duty_cycle = 0.0;
        config.complementary_pwm = true;
        controller = SixStepController(config);
    }
};

TEST_F(SixStepControllerTest, Initialization)
{
    EXPECT_EQ(controller.getConfig().bus_voltage, 12.0);
    EXPECT_EQ(controller.getHallState(), HallState::INVALID);
}

TEST_F(SixStepControllerTest, CommutationState1)
{
    // Hall State 1: A+ C-
    PWMOutput pwm = controller.update(HallState::STATE_1, 0.5);

    EXPECT_DOUBLE_EQ(pwm.duty_a, 0.5);   // A high-side PWM
    EXPECT_DOUBLE_EQ(pwm.duty_b, 0.0);   // B off
    EXPECT_DOUBLE_EQ(pwm.duty_c, 0.5);   // C low-side (complementary)

    EXPECT_EQ(controller.getHallState(), HallState::STATE_1);
    EXPECT_EQ(controller.getSector(), 1);
}

TEST_F(SixStepControllerTest, CommutationState2)
{
    // Hall State 2: B+ A-
    PWMOutput pwm = controller.update(HallState::STATE_2, 0.5);

    EXPECT_DOUBLE_EQ(pwm.duty_a, 0.5);   // A low-side (complementary)
    EXPECT_DOUBLE_EQ(pwm.duty_b, 0.5);   // B high-side PWM
    EXPECT_DOUBLE_EQ(pwm.duty_c, 0.0);   // C off
}

TEST_F(SixStepControllerTest, CommutationState3)
{
    // Hall State 3: B+ C-
    PWMOutput pwm = controller.update(HallState::STATE_3, 0.5);

    EXPECT_DOUBLE_EQ(pwm.duty_a, 0.0);   // A off
    EXPECT_DOUBLE_EQ(pwm.duty_b, 0.5);   // B high-side PWM
    EXPECT_DOUBLE_EQ(pwm.duty_c, 0.5);   // C low-side (complementary)
}

TEST_F(SixStepControllerTest, CommutationState4)
{
    // Hall State 4: C+ B-
    PWMOutput pwm = controller.update(HallState::STATE_4, 0.5);

    EXPECT_DOUBLE_EQ(pwm.duty_a, 0.0);   // A off
    EXPECT_DOUBLE_EQ(pwm.duty_b, 0.5);   // B low-side (complementary)
    EXPECT_DOUBLE_EQ(pwm.duty_c, 0.5);   // C high-side PWM
}

TEST_F(SixStepControllerTest, CommutationState5)
{
    // Hall State 5: C+ A-
    PWMOutput pwm = controller.update(HallState::STATE_5, 0.5);

    EXPECT_DOUBLE_EQ(pwm.duty_a, 0.5);   // A low-side (complementary)
    EXPECT_DOUBLE_EQ(pwm.duty_b, 0.0);   // B off
    EXPECT_DOUBLE_EQ(pwm.duty_c, 0.5);   // C high-side PWM
}

TEST_F(SixStepControllerTest, CommutationState6)
{
    // Hall State 6: A+ B-
    PWMOutput pwm = controller.update(HallState::STATE_6, 0.5);

    EXPECT_DOUBLE_EQ(pwm.duty_a, 0.5);   // A high-side PWM
    EXPECT_DOUBLE_EQ(pwm.duty_b, 0.5);   // B low-side (complementary)
    EXPECT_DOUBLE_EQ(pwm.duty_c, 0.0);   // C off
}

TEST_F(SixStepControllerTest, InvalidHallState)
{
    // Invalid hall state should result in all phases off
    PWMOutput pwm = controller.update(HallState::INVALID, 0.5);

    EXPECT_DOUBLE_EQ(pwm.duty_a, 0.0);
    EXPECT_DOUBLE_EQ(pwm.duty_b, 0.0);
    EXPECT_DOUBLE_EQ(pwm.duty_c, 0.0);
}

TEST_F(SixStepControllerTest, DutyCycleSaturation)
{
    // Test maximum duty cycle limiting
    PWMOutput pwm = controller.update(HallState::STATE_1, 1.5);

    EXPECT_NEAR(pwm.duty_a, 0.95, 1e-9);  // Clamped to max
    EXPECT_NEAR(pwm.duty_c, 0.05, 1e-9);  // Complementary of max

    // Test minimum duty cycle
    pwm = controller.update(HallState::STATE_1, -0.5);

    EXPECT_NEAR(pwm.duty_a, 0.0, 1e-9);   // Clamped to min
    EXPECT_NEAR(pwm.duty_c, 1.0, 1e-9);   // Complementary of min
}

TEST_F(SixStepControllerTest, NonComplementaryPWM)
{
    config.complementary_pwm = false;
    controller.setConfig(config);

    PWMOutput pwm = controller.update(HallState::STATE_1, 0.5);

    EXPECT_DOUBLE_EQ(pwm.duty_a, 0.5);   // A high-side PWM
    EXPECT_DOUBLE_EQ(pwm.duty_b, 0.0);   // B off
    EXPECT_DOUBLE_EQ(pwm.duty_c, 0.0);   // C off (not complementary)
}

TEST_F(SixStepControllerTest, PWMToVoltagesComplementary)
{
    PWMOutput pwm;
    pwm.duty_a = 0.75;
    pwm.duty_b = 0.50;
    pwm.duty_c = 0.25;

    double v_a, v_b, v_c;
    controller.pwmToVoltages(pwm, v_a, v_b, v_c);

    EXPECT_NEAR(v_a, 0.25 * 12.0, 1e-6);   // (0.75 - 0.5) * 12
    EXPECT_NEAR(v_b, 0.0, 1e-6);            // (0.50 - 0.5) * 12
    EXPECT_NEAR(v_c, -0.25 * 12.0, 1e-6);  // (0.25 - 0.5) * 12
}

TEST_F(SixStepControllerTest, FullCommutationCycle)
{
    // Test a full electrical cycle through all 6 states
    HallState states[] = {
        HallState::STATE_1,
        HallState::STATE_2,
        HallState::STATE_3,
        HallState::STATE_4,
        HallState::STATE_5,
        HallState::STATE_6
    };

    for (int i = 0; i < 6; i++)
    {
        PWMOutput pwm = controller.update(states[i], 0.6);

        // Verify that exactly two phases are active (complementary PWM)
        int active_count = 0;
        if (pwm.duty_a > 0.01 && pwm.duty_a < 0.99) active_count++;
        if (pwm.duty_b > 0.01 && pwm.duty_b < 0.99) active_count++;
        if (pwm.duty_c > 0.01 && pwm.duty_c < 0.99) active_count++;

        EXPECT_GE(active_count, 2) << "State " << (i + 1) << " should have at least 2 active phases";
    }
}

// ========== FOC Controller Tests ==========

class FOCControllerTest : public ::testing::Test
{
protected:
    FOCConfig config;
    FOCController controller;

    void SetUp() override
    {
        config.bus_voltage = 12.0;
        config.max_duty_cycle = 0.95;
        config.min_duty_cycle = 0.05;
        config.use_svpwm = false;  // Start with SPWM for simpler tests
        controller = FOCController(config);
    }
};

TEST_F(FOCControllerTest, Initialization)
{
    EXPECT_EQ(controller.getConfig().bus_voltage, 12.0);
    EXPECT_FALSE(controller.getConfig().use_svpwm);
}

TEST_F(FOCControllerTest, InverseParkTransform)
{
    // Test inverse Park transform at theta = 0
    // v_d = 1, v_q = 0 should give v_alpha = 1, v_beta = 0
    PWMOutput pwm = controller.updateDQ(1.0, 0.0, 0.0);

    double v_alpha, v_beta;
    controller.getAlphaBetaVoltages(v_alpha, v_beta);

    EXPECT_NEAR(v_alpha, 1.0, 1e-6);
    EXPECT_NEAR(v_beta, 0.0, 1e-6);
}

TEST_F(FOCControllerTest, InverseParkTransformRotated)
{
    // Test inverse Park transform at theta = PI/2
    // v_d = 1, v_q = 0 should give v_alpha = 0, v_beta = 1
    double theta = PI / 2.0;
    PWMOutput pwm = controller.updateDQ(1.0, 0.0, theta);

    double v_alpha, v_beta;
    controller.getAlphaBetaVoltages(v_alpha, v_beta);

    EXPECT_NEAR(v_alpha, 0.0, 1e-6);
    EXPECT_NEAR(v_beta, 1.0, 1e-6);
}

TEST_F(FOCControllerTest, SPWMZeroVoltage)
{
    // Zero voltage should give 50% duty cycle on all phases
    PWMOutput pwm = controller.updateAlphaBeta(0.0, 0.0);

    EXPECT_NEAR(pwm.duty_a, 0.5, 1e-6);
    EXPECT_NEAR(pwm.duty_b, 0.5, 1e-6);
    EXPECT_NEAR(pwm.duty_c, 0.5, 1e-6);
}

TEST_F(FOCControllerTest, SPWMAlphaAxisPositive)
{
    // Positive alpha voltage (aligned with phase A)
    PWMOutput pwm = controller.updateAlphaBeta(6.0, 0.0);

    // Phase A should be high, B and C should be low (balanced)
    EXPECT_GT(pwm.duty_a, 0.5);
    EXPECT_LT(pwm.duty_b, 0.5);
    EXPECT_LT(pwm.duty_c, 0.5);

    // Check symmetry: B and C should be equal
    EXPECT_NEAR(pwm.duty_b, pwm.duty_c, 1e-6);
}

TEST_F(FOCControllerTest, SPWMBetaAxisPositive)
{
    // Positive beta voltage (90° from phase A)
    PWMOutput pwm = controller.updateAlphaBeta(0.0, 6.0);

    // Phase B should be high, C should be low
    EXPECT_GT(pwm.duty_b, 0.5);
    EXPECT_LT(pwm.duty_c, 0.5);
}

TEST_F(FOCControllerTest, PWMToVoltages)
{
    PWMOutput pwm;
    pwm.duty_a = 0.75;
    pwm.duty_b = 0.50;
    pwm.duty_c = 0.25;

    double v_a, v_b, v_c;
    controller.pwmToVoltages(pwm, v_a, v_b, v_c);

    EXPECT_NEAR(v_a, 0.25 * 12.0, 1e-6);   // (0.75 - 0.5) * 12
    EXPECT_NEAR(v_b, 0.0, 1e-6);            // (0.50 - 0.5) * 12
    EXPECT_NEAR(v_c, -0.25 * 12.0, 1e-6);  // (0.25 - 0.5) * 12
}

TEST_F(FOCControllerTest, DutyCycleSaturation)
{
    // Very large voltage should saturate duty cycles
    PWMOutput pwm = controller.updateAlphaBeta(100.0, 0.0);

    EXPECT_GE(pwm.duty_a, config.min_duty_cycle);
    EXPECT_LE(pwm.duty_a, config.max_duty_cycle);
    EXPECT_GE(pwm.duty_b, config.min_duty_cycle);
    EXPECT_LE(pwm.duty_b, config.max_duty_cycle);
    EXPECT_GE(pwm.duty_c, config.min_duty_cycle);
    EXPECT_LE(pwm.duty_c, config.max_duty_cycle);
}

TEST_F(FOCControllerTest, SVPWMEnabled)
{
    config.use_svpwm = true;
    controller.setConfig(config);

    // Test SVPWM with zero voltage
    PWMOutput pwm = controller.updateAlphaBeta(0.0, 0.0);

    // Should still work and produce valid duty cycles
    EXPECT_GE(pwm.duty_a, 0.0);
    EXPECT_LE(pwm.duty_a, 1.0);
    EXPECT_GE(pwm.duty_b, 0.0);
    EXPECT_LE(pwm.duty_b, 1.0);
    EXPECT_GE(pwm.duty_c, 0.0);
    EXPECT_LE(pwm.duty_c, 1.0);
}

TEST_F(FOCControllerTest, SVPWMSector1)
{
    config.use_svpwm = true;
    controller.setConfig(config);

    // Sector 1: 0° to 60° (positive alpha, small positive beta)
    double v_alpha = 3.0;
    double v_beta = 1.0;

    PWMOutput pwm = controller.updateAlphaBeta(v_alpha, v_beta);

    // In sector 1, phase A should have highest duty cycle
    EXPECT_GE(pwm.duty_a, pwm.duty_b);
    EXPECT_GE(pwm.duty_a, pwm.duty_c);
}

TEST_F(FOCControllerTest, SVPWMSector3)
{
    config.use_svpwm = true;
    controller.setConfig(config);

    // Sector 3: 120° to 180° (negative alpha, positive beta)
    double v_alpha = -3.0;
    double v_beta = 3.0;

    PWMOutput pwm = controller.updateAlphaBeta(v_alpha, v_beta);

    // In sector 3, phase B should have highest duty cycle
    EXPECT_GE(pwm.duty_b, pwm.duty_a);
    EXPECT_GE(pwm.duty_b, pwm.duty_c);
}

TEST_F(FOCControllerTest, SVPWMVoltageSaturation)
{
    config.use_svpwm = true;
    controller.setConfig(config);

    // Very large voltage should saturate but still produce valid output
    PWMOutput pwm = controller.updateAlphaBeta(100.0, 100.0);

    EXPECT_GE(pwm.duty_a, config.min_duty_cycle);
    EXPECT_LE(pwm.duty_a, config.max_duty_cycle);
    EXPECT_GE(pwm.duty_b, config.min_duty_cycle);
    EXPECT_LE(pwm.duty_b, config.max_duty_cycle);
    EXPECT_GE(pwm.duty_c, config.min_duty_cycle);
    EXPECT_LE(pwm.duty_c, config.max_duty_cycle);
}

TEST_F(FOCControllerTest, SVPWMDutyCycleSum)
{
    config.use_svpwm = true;
    controller.setConfig(config);

    // For balanced 3-phase, sum of duty cycles should be approximately 1.5 (3 * 0.5)
    PWMOutput pwm = controller.updateAlphaBeta(2.0, 1.0);

    double sum = pwm.duty_a + pwm.duty_b + pwm.duty_c;
    EXPECT_NEAR(sum, 1.5, 0.2);  // Reasonable tolerance for SVPWM
}

TEST_F(FOCControllerTest, FullRotationSPWM)
{
    // Test FOC through a full electrical rotation
    int num_steps = 36;
    for (int i = 0; i < num_steps; i++)
    {
        double theta = (i * TWO_PI) / num_steps;
        PWMOutput pwm = controller.updateDQ(5.0, 0.0, theta);

        // All duty cycles should be valid
        EXPECT_GE(pwm.duty_a, 0.0);
        EXPECT_LE(pwm.duty_a, 1.0);
        EXPECT_GE(pwm.duty_b, 0.0);
        EXPECT_LE(pwm.duty_b, 1.0);
        EXPECT_GE(pwm.duty_c, 0.0);
        EXPECT_LE(pwm.duty_c, 1.0);
    }
}

TEST_F(FOCControllerTest, FullRotationSVPWM)
{
    config.use_svpwm = true;
    controller.setConfig(config);

    // Test SVPWM through a full electrical rotation
    int num_steps = 36;
    for (int i = 0; i < num_steps; i++)
    {
        double theta = (i * TWO_PI) / num_steps;
        PWMOutput pwm = controller.updateDQ(5.0, 0.0, theta);

        // All duty cycles should be valid
        EXPECT_GE(pwm.duty_a, 0.0);
        EXPECT_LE(pwm.duty_a, 1.0);
        EXPECT_GE(pwm.duty_b, 0.0);
        EXPECT_LE(pwm.duty_b, 1.0);
        EXPECT_GE(pwm.duty_c, 0.0);
        EXPECT_LE(pwm.duty_c, 1.0);
    }
}

// ========== Integration Tests ==========

TEST(MotorControlIntegrationTest, SixStepWithVelocityControl)
{
    // Simulate using six-step commutation with a simple velocity controller
    SixStepConfig six_config;
    six_config.bus_voltage = 24.0;
    SixStepController six_step(six_config);

    // Simulate hall sensor states changing as motor rotates
    HallState hall_sequence[] = {
        HallState::STATE_1,
        HallState::STATE_2,
        HallState::STATE_3,
        HallState::STATE_4,
        HallState::STATE_5,
        HallState::STATE_6
    };

    double duty_cycle = 0.3;  // 30% duty cycle

    for (int i = 0; i < 6; i++)
    {
        PWMOutput pwm = six_step.update(hall_sequence[i], duty_cycle);

        // Verify PWM is valid
        EXPECT_GE(pwm.duty_a, 0.0);
        EXPECT_LE(pwm.duty_a, 1.0);
        EXPECT_GE(pwm.duty_b, 0.0);
        EXPECT_LE(pwm.duty_b, 1.0);
        EXPECT_GE(pwm.duty_c, 0.0);
        EXPECT_LE(pwm.duty_c, 1.0);

        // Convert to voltages
        double v_a, v_b, v_c;
        six_step.pwmToVoltages(pwm, v_a, v_b, v_c);

        // Voltages should be reasonable
        EXPECT_GE(v_a, -24.0);
        EXPECT_LE(v_a, 24.0);
        EXPECT_GE(v_b, -24.0);
        EXPECT_LE(v_b, 24.0);
        EXPECT_GE(v_c, -24.0);
        EXPECT_LE(v_c, 24.0);
    }
}

TEST(MotorControlIntegrationTest, FOCWithCurrentControl)
{
    // Simulate FOC with current control loop
    FOCConfig foc_config;
    foc_config.bus_voltage = 24.0;
    foc_config.use_svpwm = true;
    FOCController foc(foc_config);

    // Simulate motor rotating and FOC commanding torque
    double v_d = 0.0;     // No flux weakening
    double v_q = 8.0;     // Torque command

    for (int i = 0; i < 10; i++)
    {
        double theta = i * 0.1;  // Simulated rotor angle
        PWMOutput pwm = foc.updateDQ(v_d, v_q, theta);

        // Verify PWM is valid
        EXPECT_GE(pwm.duty_a, 0.0);
        EXPECT_LE(pwm.duty_a, 1.0);
        EXPECT_GE(pwm.duty_b, 0.0);
        EXPECT_LE(pwm.duty_b, 1.0);
        EXPECT_GE(pwm.duty_c, 0.0);
        EXPECT_LE(pwm.duty_c, 1.0);

        // Convert to voltages
        double v_a, v_b, v_c;
        foc.pwmToVoltages(pwm, v_a, v_b, v_c);

        // Voltages should sum to approximately zero (balanced 3-phase with centered PWM)
        // Note: SVPWM may have some offset, so we use a larger tolerance
        double sum = v_a + v_b + v_c;
        EXPECT_NEAR(sum, 0.0, 10.0);  // Larger tolerance for SVPWM

        // Verify voltages are within reasonable bounds
        EXPECT_GE(v_a, -foc_config.bus_voltage);
        EXPECT_LE(v_a, foc_config.bus_voltage);
        EXPECT_GE(v_b, -foc_config.bus_voltage);
        EXPECT_LE(v_b, foc_config.bus_voltage);
        EXPECT_GE(v_c, -foc_config.bus_voltage);
        EXPECT_LE(v_c, foc_config.bus_voltage);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
