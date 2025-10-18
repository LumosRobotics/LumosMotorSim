// /Users/danielpi/work/LumosMotorSim/src/modules/simulator/sensors.cpp
#include "sensors.h"
#include <cmath>

namespace modules
{
    namespace simulator
    {
        // ========== Hall Sensor Array Implementation ==========

        HallSensorArray::HallSensorArray(const HallSensorConfig &config)
            : config_(config)
        {
        }

        void HallSensorArray::update(double theta_mech)
        {
            // Convert mechanical angle to electrical angle
            double theta_elec = theta_mech * config_.num_pole_pairs;

            // Normalize to [0, 2π)
            theta_elec = std::fmod(theta_elec, 2.0 * SENSOR_PI);
            if (theta_elec < 0.0)
                theta_elec += 2.0 * SENSOR_PI;

            // Calculate angle for each Hall sensor
            double angle_a = theta_elec + config_.sensor_offset_a;
            double angle_b = angle_a + config_.sensor_spacing;
            double angle_c = angle_b + config_.sensor_spacing;

            // Normalize angles
            angle_a = std::fmod(angle_a, 2.0 * SENSOR_PI);
            angle_b = std::fmod(angle_b, 2.0 * SENSOR_PI);
            angle_c = std::fmod(angle_c, 2.0 * SENSOR_PI);

            // Hall sensor is HIGH when magnet north pole is nearby
            // We'll use a simple model: HIGH when angle is in [0, π), LOW when in [π, 2π)
            // With hysteresis to prevent chattering

            auto updateHallWithHysteresis = [this](double angle, bool &, bool prev) -> bool
            {
                if (!prev && angle < (SENSOR_PI - config_.hysteresis))
                {
                    return true; // Transition to HIGH
                }
                else if (prev && angle > (SENSOR_PI + config_.hysteresis))
                {
                    return false; // Transition to LOW
                }
                return prev; // Keep previous state (hysteresis band)
            };

            hall_a_ = updateHallWithHysteresis(angle_a, hall_a_, prev_hall_a_);
            hall_b_ = updateHallWithHysteresis(angle_b, hall_b_, prev_hall_b_);
            hall_c_ = updateHallWithHysteresis(angle_c, hall_c_, prev_hall_c_);

            // Update previous states
            prev_hall_a_ = hall_a_;
            prev_hall_b_ = hall_b_;
            prev_hall_c_ = hall_c_;
        }

        uint8_t HallSensorArray::getHallState() const
        {
            // Return 3-bit pattern: bit 0 = A, bit 1 = B, bit 2 = C
            return (hall_a_ ? 1 : 0) |
                   (hall_b_ ? 2 : 0) |
                   (hall_c_ ? 4 : 0);
        }

        int HallSensorArray::getSector() const
        {
            // Map Hall state to electrical sector (0-5)
            // Standard Hall sensor sequence for clockwise rotation:
            // 101(5) -> 001(1) -> 011(3) -> 010(2) -> 110(6) -> 100(4) -> 101(5)...

            uint8_t state = getHallState();

            switch (state)
            {
            case 0b101: return 0;  // Hall state 5
            case 0b001: return 1;  // Hall state 1
            case 0b011: return 2;  // Hall state 3
            case 0b010: return 3;  // Hall state 2
            case 0b110: return 4;  // Hall state 6
            case 0b100: return 5;  // Hall state 4
            default:    return -1; // Invalid Hall state
            }
        }

        double HallSensorArray::getElectricalAngleEstimate() const
        {
            // Return the center angle of the current sector
            int sector = getSector();
            if (sector < 0)
                return 0.0; // Invalid state

            // Each sector spans 60° electrical (π/3 radians)
            // Return the center of the sector
            return (sector + 0.5) * (SENSOR_PI / 3.0);
        }

        // ========== Incremental Encoder Implementation ==========

        IncrementalEncoder::IncrementalEncoder(const IncrementalEncoderConfig &config)
            : config_(config)
        {
            reset();
        }

        void IncrementalEncoder::update(double theta_mech)
        {
            // Normalize angle to [0, 2π)
            theta_mech = std::fmod(theta_mech, 2.0 * SENSOR_PI);
            if (theta_mech < 0.0)
                theta_mech += 2.0 * SENSOR_PI;

            // Apply mechanical zero offset
            double encoder_angle = theta_mech - config_.mechanical_zero_offset;
            encoder_angle = std::fmod(encoder_angle, 2.0 * SENSOR_PI);
            if (encoder_angle < 0.0)
                encoder_angle += 2.0 * SENSOR_PI;

            // Calculate the angle per pulse
            double angle_per_pulse = (2.0 * SENSOR_PI) / config_.pulses_per_revolution;

            // Generate channel A square wave
            // A is HIGH for first half of each pulse period, LOW for second half
            double phase_a = encoder_angle / angle_per_pulse;
            bool new_channel_a = (std::fmod(phase_a, 1.0) < 0.5);

            // Generate channel B square wave (shifted by phase_offset)
            double encoder_angle_b = encoder_angle + config_.phase_offset;
            encoder_angle_b = std::fmod(encoder_angle_b, 2.0 * SENSOR_PI);
            if (encoder_angle_b < 0.0)
                encoder_angle_b += 2.0 * SENSOR_PI;

            double phase_b = encoder_angle_b / angle_per_pulse;
            bool new_channel_b = (std::fmod(phase_b, 1.0) < 0.5);

            // Quadrature decoder (4x resolution - count all edges)
            // State machine for quadrature decoding
            //
            // With phase_offset = +π/2, channel B LEADS channel A by 90° for forward rotation:
            // B: ‾‾|__|‾‾|__|‾‾
            // A: __|‾‾|__|‾‾|__
            //
            // For FORWARD rotation with B leading A:
            // When A rises: B should be HIGH (A == B after rise) -> count++
            // When A falls: B should be LOW (A != B after fall) -> count++
            // When B rises: A should be LOW (A != B) -> count++
            // When B falls: A should be HIGH (A == B after B fall) -> count++
            //
            // Simplified rules for B-leads-A configuration:
            // - When A changes: if new_A == new_B -> forward, else backward
            // - When B changes: if new_A != new_B -> forward, else backward
            if (new_channel_a != prev_channel_a_)
            {
                // A changed
                if (new_channel_a == new_channel_b)
                {
                    count_++;  // Forward rotation
                }
                else
                {
                    count_--;  // Reverse rotation
                }
            }

            if (new_channel_b != prev_channel_b_)
            {
                // B changed
                if (new_channel_a != new_channel_b)
                {
                    count_++;  // Forward rotation
                }
                else
                {
                    count_--;  // Reverse rotation
                }
            }

            // Update states for next iteration
            channel_a_ = new_channel_a;
            channel_b_ = new_channel_b;
            prev_channel_a_ = new_channel_a;
            prev_channel_b_ = new_channel_b;

            // Generate index pulse
            if (config_.has_index)
            {
                index_pulse_ = (encoder_angle < config_.index_width) ||
                               (encoder_angle > (2.0 * SENSOR_PI - config_.index_width));
            }
            else
            {
                index_pulse_ = false;
            }

            // Store angle for velocity calculation
            prev_angle_ = encoder_angle;
        }

        double IncrementalEncoder::getVelocity(double dt) const
        {
            if (dt <= 0.0)
                return 0.0;

            // Calculate velocity from count change
            int32_t count_diff = count_ - prev_count_;
            double angle_change = (2.0 * SENSOR_PI * count_diff) / getCountsPerRev();

            return angle_change / dt;
        }

        void IncrementalEncoder::reset()
        {
            count_ = 0;
            prev_count_ = 0;
            channel_a_ = false;
            channel_b_ = false;
            prev_channel_a_ = false;
            prev_channel_b_ = false;
            index_pulse_ = false;
            prev_angle_ = 0.0;
        }

        // ========== Absolute Encoder Implementation ==========

        Encoder::Encoder(const EncoderConfig &config)
            : config_(config)
        {
            // Calculate counts per revolution based on resolution
            counts_per_rev_ = 1u << config_.resolution_bits;  // 2^resolution_bits
            reset();
        }

        void Encoder::update(double theta_mech)
        {
            // Normalize angle to [0, 2π)
            theta_mech = std::fmod(theta_mech, 2.0 * SENSOR_PI);
            if (theta_mech < 0.0)
                theta_mech += 2.0 * SENSOR_PI;

            current_angle_ = theta_mech;

            // Apply mechanical zero offset (encoder mounting offset)
            double encoder_angle = theta_mech - config_.mechanical_zero_offset;

            // Normalize to [0, 2π)
            encoder_angle = std::fmod(encoder_angle, 2.0 * SENSOR_PI);
            if (encoder_angle < 0.0)
                encoder_angle += 2.0 * SENSOR_PI;

            // Convert to counts
            counts_ = angleToCounts(encoder_angle);

            // Generate index pulse
            if (config_.has_index)
            {
                // Index pulse is active when encoder angle is within index_width of zero
                index_pulse_ = (encoder_angle < config_.index_width) ||
                               (encoder_angle > (2.0 * SENSOR_PI - config_.index_width));
            }
            else
            {
                index_pulse_ = false;
            }
        }

        double Encoder::getAngle() const
        {
            // Return angle as measured by encoder (with zero offset applied)
            return countsToAngle(counts_);
        }

        double Encoder::getAngleRaw() const
        {
            // Return actual rotor angle (without zero offset)
            return current_angle_;
        }

        double Encoder::countsToAngle(uint32_t counts) const
        {
            // Convert counts to radians
            return (2.0 * SENSOR_PI * counts) / counts_per_rev_;
        }

        uint32_t Encoder::angleToCounts(double angle) const
        {
            // Normalize angle
            angle = std::fmod(angle, 2.0 * SENSOR_PI);
            if (angle < 0.0)
                angle += 2.0 * SENSOR_PI;

            // Convert to counts and round
            uint32_t counts = static_cast<uint32_t>((angle * counts_per_rev_) / (2.0 * SENSOR_PI));

            // Ensure within valid range
            if (counts >= counts_per_rev_)
                counts = counts_per_rev_ - 1;

            return counts;
        }

        void Encoder::reset()
        {
            counts_ = 0;
            current_angle_ = 0.0;
            index_pulse_ = false;
        }

    } // namespace simulator
} // namespace modules
