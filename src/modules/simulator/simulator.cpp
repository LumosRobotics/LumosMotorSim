#include "simulator.h"
#include <chrono>
#include <thread>

namespace modules
{
    namespace simulator
    {
        Simulator::~Simulator()
        {
            stop();
        }

        bool Simulator::start()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_)
            {
                return false; // Already running
            }
            running_ = true;
            return true;
        }

        void Simulator::stop()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
        }

        void Simulator::reset(double sim_time)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            sim_time_ = sim_time;
        }

        double Simulator::step()
        {
            std::lock_guard<std::mutex> lock(mutex_);

            // Perform simulation step with configured time step
            double dt = config_.time_step;

            // Execute tick callback if registered
            if (tick_cb_)
            {
                tick_cb_(sim_time_, dt);
            }

            // Advance simulation time
            sim_time_ += dt;

            // If realtime mode, try to maintain realtime sync
            if (config_.realtime)
            {
                // Simple realtime delay (more sophisticated timing could be added)
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(dt));
            }

            return sim_time_;
        }

        double Simulator::update(double dt)
        {
            std::lock_guard<std::mutex> lock(mutex_);

            // Calculate how many steps needed
            int num_steps = static_cast<int>(dt / config_.time_step);
            double remainder = dt - num_steps * config_.time_step;

            // Perform full steps
            for (int i = 0; i < num_steps; ++i)
            {
                if (tick_cb_)
                {
                    tick_cb_(sim_time_, config_.time_step);
                }
                sim_time_ += config_.time_step;
            }

            // Handle remainder if significant
            if (remainder > 1e-9)
            {
                if (tick_cb_)
                {
                    tick_cb_(sim_time_, remainder);
                }
                sim_time_ += remainder;
            }

            return sim_time_;
        }

        double Simulator::time() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return sim_time_;
        }

        double Simulator::timeStep() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return config_.time_step;
        }

        bool Simulator::running() const
        {
            return running_;
        }

        void Simulator::setTickCallback(TickCallback cb)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tick_cb_ = cb;
        }

    } // namespace simulator
} // namespace modules
