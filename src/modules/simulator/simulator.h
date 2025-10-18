// /Users/danielpi/work/LumosMotorSim/src/modules/simulator/simulator.h
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>

namespace modules
{
    namespace simulator
    {

        struct SimulatorConfig
        {
            double time_step = 0.001; // default simulation step in seconds
            bool realtime = false;    // try to run in realtime when true
        };

        class Simulator
        {
        public:
            using TickCallback = std::function<void(double /*sim_time*/, double /*dt*/)>;

            Simulator() = default;
            ~Simulator();

            // non-copyable
            Simulator(const Simulator &) = delete;
            Simulator &operator=(const Simulator &) = delete;

            // Basic control
            bool start(); // returns true if started (was not already running)
            void stop();  // stops the simulation (safe to call when not running)
            void reset(double sim_time = 0.0);

            // Perform a single simulation step using configured time_step
            // Returns the new simulation time
            double step();

            // Advance simulation by dt (can be multiple internal steps)
            // Returns the new simulation time
            double update(double dt);

            // Accessors
            double time() const;
            double timeStep() const;
            bool running() const;

            // Optional callback invoked after each step (from caller thread)
            void setTickCallback(TickCallback cb);

        private:
            mutable std::mutex mutex_;
            std::atomic<bool> running_{false};
            SimulatorConfig config_{};
            double sim_time_{0.0};

            TickCallback tick_cb_{};
        };

    } // namespace simulator
} // namespace modules