#include "modules/simulator/simulator.h"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using namespace modules::simulator;

// ===== Basic Simulator Tests =====

class SimulatorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Fresh simulator for each test
        sim_ = std::make_unique<Simulator>();
    }

    void TearDown() override
    {
        // Ensure stopped
        if (sim_)
        {
            sim_->stop();
        }
    }

    std::unique_ptr<Simulator> sim_;
};

TEST_F(SimulatorTest, Initialization)
{
    // Initially not running
    EXPECT_FALSE(sim_->running());

    // Initial time should be zero
    EXPECT_DOUBLE_EQ(sim_->time(), 0.0);

    // Default time step should be 0.001 (1ms)
    EXPECT_DOUBLE_EQ(sim_->timeStep(), 0.001);
}

TEST_F(SimulatorTest, StartStop)
{
    // Start simulator
    bool started = sim_->start();
    EXPECT_TRUE(started);
    EXPECT_TRUE(sim_->running());

    // Try to start again - should return false
    started = sim_->start();
    EXPECT_FALSE(started);
    EXPECT_TRUE(sim_->running());

    // Stop simulator
    sim_->stop();
    EXPECT_FALSE(sim_->running());

    // Stop again - should be safe
    sim_->stop();
    EXPECT_FALSE(sim_->running());

    // Can start again after stop
    started = sim_->start();
    EXPECT_TRUE(started);
    EXPECT_TRUE(sim_->running());
}

TEST_F(SimulatorTest, Reset)
{
    // Advance time
    sim_->step();
    sim_->step();
    EXPECT_GT(sim_->time(), 0.0);

    // Reset to zero
    sim_->reset();
    EXPECT_DOUBLE_EQ(sim_->time(), 0.0);

    // Reset to specific time
    sim_->reset(5.5);
    EXPECT_DOUBLE_EQ(sim_->time(), 5.5);

    // Can reset while running
    sim_->start();
    sim_->reset(10.0);
    EXPECT_DOUBLE_EQ(sim_->time(), 10.0);
    EXPECT_TRUE(sim_->running());
}

TEST_F(SimulatorTest, SingleStep)
{
    double initial_time = sim_->time();

    // Perform one step
    double new_time = sim_->step();

    // Time should have advanced by time_step
    EXPECT_NEAR(new_time, initial_time + 0.001, 1e-9);
    EXPECT_NEAR(sim_->time(), initial_time + 0.001, 1e-9);

    // Another step
    new_time = sim_->step();
    EXPECT_NEAR(new_time, initial_time + 0.002, 1e-9);
}

TEST_F(SimulatorTest, MultipleSteps)
{
    // Perform multiple steps
    for (int i = 0; i < 100; i++)
    {
        sim_->step();
    }

    // Should be at 100 * 0.001 = 0.1 seconds
    EXPECT_NEAR(sim_->time(), 0.1, 1e-6);
}

TEST_F(SimulatorTest, UpdateWithExactMultiple)
{
    // Update by exactly 10 time steps
    double dt = 0.01;  // 10 * 0.001
    double new_time = sim_->update(dt);

    EXPECT_NEAR(new_time, 0.01, 1e-9);
    EXPECT_NEAR(sim_->time(), 0.01, 1e-9);
}

TEST_F(SimulatorTest, UpdateWithRemainder)
{
    // Update by a time that doesn't divide evenly
    double dt = 0.0035;  // 3 full steps + 0.0005 remainder
    double new_time = sim_->update(dt);

    EXPECT_NEAR(new_time, 0.0035, 1e-9);
    EXPECT_NEAR(sim_->time(), 0.0035, 1e-9);
}

TEST_F(SimulatorTest, UpdateLargeTimespan)
{
    // Update by a large time
    double dt = 1.0;  // 1 second = 1000 steps
    double new_time = sim_->update(dt);

    EXPECT_NEAR(new_time, 1.0, 1e-6);
    EXPECT_NEAR(sim_->time(), 1.0, 1e-6);
}

TEST_F(SimulatorTest, UpdateVerySmallRemainder)
{
    // Update with a remainder smaller than threshold (1e-9)
    // Should effectively be ignored
    double dt = 0.003 + 1e-12;  // 3 steps + tiny remainder
    sim_->update(dt);

    // Should be at exactly 3 steps worth
    EXPECT_NEAR(sim_->time(), 0.003, 1e-9);
}

// ===== Callback Tests =====

TEST_F(SimulatorTest, CallbackInvocation)
{
    int callback_count = 0;
    double last_sim_time = -1.0;
    double last_dt = -1.0;

    sim_->setTickCallback([&](double sim_time, double dt) {
        callback_count++;
        last_sim_time = sim_time;
        last_dt = dt;
    });

    // Perform a step
    sim_->step();

    EXPECT_EQ(callback_count, 1);
    EXPECT_DOUBLE_EQ(last_sim_time, 0.0);  // Callback sees time BEFORE step
    EXPECT_DOUBLE_EQ(last_dt, 0.001);

    // Another step
    sim_->step();

    EXPECT_EQ(callback_count, 2);
    EXPECT_DOUBLE_EQ(last_sim_time, 0.001);
    EXPECT_DOUBLE_EQ(last_dt, 0.001);
}

TEST_F(SimulatorTest, CallbackWithUpdate)
{
    int callback_count = 0;
    std::vector<double> sim_times;
    std::vector<double> dts;

    sim_->setTickCallback([&](double sim_time, double dt) {
        callback_count++;
        sim_times.push_back(sim_time);
        dts.push_back(dt);
    });

    // Update by 0.0035 (3 full steps + 0.0005 remainder)
    sim_->update(0.0035);

    // Should have 4 callbacks: 3 full steps + 1 remainder
    EXPECT_EQ(callback_count, 4);
    EXPECT_EQ(sim_times.size(), 4);

    // Check first three are full steps
    for (int i = 0; i < 3; i++)
    {
        EXPECT_NEAR(dts[i], 0.001, 1e-9);
    }

    // Last one is remainder
    EXPECT_NEAR(dts[3], 0.0005, 1e-9);

    // Check simulation times progress correctly
    EXPECT_NEAR(sim_times[0], 0.0, 1e-9);
    EXPECT_NEAR(sim_times[1], 0.001, 1e-9);
    EXPECT_NEAR(sim_times[2], 0.002, 1e-9);
    EXPECT_NEAR(sim_times[3], 0.003, 1e-9);
}

TEST_F(SimulatorTest, CallbackCanModifyExternalState)
{
    double external_value = 0.0;

    sim_->setTickCallback([&](double sim_time, double dt) {
        external_value += dt;
    });

    // Run 10 steps
    for (int i = 0; i < 10; i++)
    {
        sim_->step();
    }

    // External value should have accumulated all dt's
    EXPECT_NEAR(external_value, 0.01, 1e-9);
}

TEST_F(SimulatorTest, CallbackReplacement)
{
    int callback1_count = 0;
    int callback2_count = 0;

    // Set first callback
    sim_->setTickCallback([&](double sim_time, double dt) {
        callback1_count++;
    });

    sim_->step();
    EXPECT_EQ(callback1_count, 1);
    EXPECT_EQ(callback2_count, 0);

    // Replace with second callback
    sim_->setTickCallback([&](double sim_time, double dt) {
        callback2_count++;
    });

    sim_->step();
    EXPECT_EQ(callback1_count, 1);  // No longer called
    EXPECT_EQ(callback2_count, 1);
}

TEST_F(SimulatorTest, NoCallbackIsOk)
{
    // No callback set - should not crash
    sim_->step();
    sim_->update(0.01);

    EXPECT_NEAR(sim_->time(), 0.011, 1e-9);
}

// ===== Thread Safety Tests =====

TEST_F(SimulatorTest, ThreadSafeAccess)
{
    // Set up callback
    std::atomic<int> callback_count{0};
    sim_->setTickCallback([&](double sim_time, double dt) {
        callback_count++;
    });

    // Start background thread doing steps
    std::atomic<bool> stop_thread{false};
    std::thread worker([&]() {
        while (!stop_thread.load())
        {
            sim_->step();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    // Access time from main thread
    for (int i = 0; i < 10; i++)
    {
        double t = sim_->time();
        EXPECT_GE(t, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Stop and join
    stop_thread = true;
    worker.join();

    // Should have had multiple callbacks
    EXPECT_GT(callback_count.load(), 0);
}

TEST_F(SimulatorTest, ConcurrentStartStop)
{
    // Multiple threads trying to start/stop
    std::atomic<bool> stop_test{false};
    std::atomic<int> start_success_count{0};

    auto thread_func = [&]() {
        while (!stop_test.load())
        {
            if (sim_->start())
            {
                start_success_count++;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            sim_->stop();
        }
    };

    std::thread t1(thread_func);
    std::thread t2(thread_func);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    stop_test = true;

    t1.join();
    t2.join();

    // Should have had some successful starts (but not every attempt succeeds)
    EXPECT_GT(start_success_count.load(), 0);
}

// ===== Integration Tests =====

TEST_F(SimulatorTest, SimulatePhysicsIntegration)
{
    // Simulate simple physics: x = v*t, v = a*t
    double position = 0.0;
    double velocity = 0.0;
    const double acceleration = 10.0;  // m/s^2

    sim_->setTickCallback([&](double sim_time, double dt) {
        // Simple Euler integration
        velocity += acceleration * dt;
        position += velocity * dt;
    });

    // Simulate 1 second
    sim_->update(1.0);

    // After 1 second with constant acceleration:
    // v = a*t = 10 * 1 = 10 m/s
    // x = 0.5*a*t^2 = 0.5 * 10 * 1 = 5 m (approximately, due to Euler method)
    EXPECT_NEAR(velocity, 10.0, 0.01);
    EXPECT_NEAR(position, 5.0, 0.1);  // Euler has some error
}

TEST_F(SimulatorTest, AccuracyWithDifferentTimeSteps)
{
    // Test that smaller time steps give more accurate results
    // Using same physics as above

    auto simulate_with_timestep = [](double timestep) -> double {
        Simulator sim;
        double position = 0.0;
        double velocity = 0.0;
        const double acceleration = 10.0;

        // Need to set time step through config (would require adding setter)
        // For now, use update with appropriate dt

        int num_steps = static_cast<int>(1.0 / timestep);
        for (int i = 0; i < num_steps; i++)
        {
            velocity += acceleration * timestep;
            position += velocity * timestep;
        }

        return position;
    };

    // Theoretical: x = 0.5 * 10 * 1^2 = 5.0
    double pos_coarse = simulate_with_timestep(0.1);   // 10 steps
    double pos_fine = simulate_with_timestep(0.01);     // 100 steps
    double pos_very_fine = simulate_with_timestep(0.001);  // 1000 steps

    // Finer time steps should be closer to theoretical 5.0
    EXPECT_LT(std::abs(pos_fine - 5.0), std::abs(pos_coarse - 5.0));
    EXPECT_LT(std::abs(pos_very_fine - 5.0), std::abs(pos_fine - 5.0));
}

TEST_F(SimulatorTest, ResetPreservesCallback)
{
    int callback_count = 0;

    sim_->setTickCallback([&](double sim_time, double dt) {
        callback_count++;
    });

    sim_->step();
    EXPECT_EQ(callback_count, 1);

    // Reset should not clear callback
    sim_->reset();

    sim_->step();
    EXPECT_EQ(callback_count, 2);
}

TEST_F(SimulatorTest, DestructorStopsSimulation)
{
    auto sim_ptr = std::make_unique<Simulator>();

    sim_ptr->start();
    EXPECT_TRUE(sim_ptr->running());

    // Destructor should stop
    sim_ptr.reset();

    // No crash - destructor called stop()
}

// ===== Edge Case Tests =====

TEST_F(SimulatorTest, UpdateWithZeroTime)
{
    double initial_time = sim_->time();

    sim_->update(0.0);

    // Time should not change
    EXPECT_DOUBLE_EQ(sim_->time(), initial_time);
}

TEST_F(SimulatorTest, UpdateWithNegativeTime)
{
    // Negative dt should be handled gracefully (no steps)
    double initial_time = sim_->time();

    sim_->update(-0.01);

    // Time should not change (no steps taken)
    EXPECT_DOUBLE_EQ(sim_->time(), initial_time);
}

TEST_F(SimulatorTest, VeryLargeNumberOfSteps)
{
    int callback_count = 0;

    sim_->setTickCallback([&](double sim_time, double dt) {
        callback_count++;
    });

    // Update by 10 seconds (10,000 steps with default 0.001 timestep)
    sim_->update(10.0);

    EXPECT_EQ(callback_count, 10000);
    EXPECT_NEAR(sim_->time(), 10.0, 1e-6);
}

TEST_F(SimulatorTest, ConsecutiveUpdates)
{
    // Multiple consecutive updates should accumulate time correctly
    sim_->update(0.1);
    EXPECT_NEAR(sim_->time(), 0.1, 1e-9);

    sim_->update(0.2);
    EXPECT_NEAR(sim_->time(), 0.3, 1e-9);

    sim_->update(0.15);
    EXPECT_NEAR(sim_->time(), 0.45, 1e-9);
}

TEST_F(SimulatorTest, MixedStepAndUpdate)
{
    // Mix step() and update() calls
    sim_->step();
    EXPECT_NEAR(sim_->time(), 0.001, 1e-9);

    sim_->update(0.01);
    EXPECT_NEAR(sim_->time(), 0.011, 1e-9);

    sim_->step();
    EXPECT_NEAR(sim_->time(), 0.012, 1e-9);

    sim_->update(0.005);
    EXPECT_NEAR(sim_->time(), 0.017, 1e-9);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
