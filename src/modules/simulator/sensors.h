// /Users/danielpi/work/LumosMotorSim/src/modules/simulator/sensors.h
#pragma once

#include <cmath>
#include <cstdint>
#include <array>

namespace modules
{
    namespace simulator
    {
        // Use M_PI from cmath if available, otherwise define our own
        #ifndef M_PI
        constexpr double M_PI = 3.14159265358979323846;
        #endif

        constexpr double SENSOR_PI = M_PI;

        /**
         * Configuration for Hall effect sensor array
         */
        struct HallSensorConfig
        {
            // Angular offset of Hall sensor A from rotor zero (electrical radians)
            double sensor_offset_a = 0.0;

            // Spacing between sensors (typically 120° electrical = 2π/3)
            double sensor_spacing = 2.0 * SENSOR_PI / 3.0;

            // Number of pole pairs (needed to convert mechanical to electrical angle)
            int num_pole_pairs = 4;

            // Hysteresis in radians to prevent chattering
            double hysteresis = 0.01;
        };

        /**
         * Hall effect sensor array (3 digital sensors)
         *
         * Simulates three digital Hall effect sensors positioned around the stator,
         * typically at 120° electrical spacing. Each sensor outputs HIGH when a
         * magnetic north pole is nearby, LOW for south pole.
         *
         * The 3-bit Hall state pattern (ABC) cycles through 6 states per electrical
         * revolution: 101 -> 001 -> 011 -> 010 -> 110 -> 100 -> 101...
         */
        class HallSensorArray
        {
        public:
            explicit HallSensorArray(const HallSensorConfig &config = HallSensorConfig());

            /**
             * Update sensor readings based on rotor position
             * @param theta_mech Mechanical rotor angle (radians)
             */
            void update(double theta_mech);

            /**
             * Get individual Hall sensor states
             * @return true = HIGH (north pole), false = LOW (south pole)
             */
            bool getHallA() const { return hall_a_; }
            bool getHallB() const { return hall_b_; }
            bool getHallC() const { return hall_c_; }

            /**
             * Get Hall state as 3-bit pattern (bits: CBA)
             * @return Value 0-7 representing the 3-bit Hall pattern
             */
            uint8_t getHallState() const;

            /**
             * Get estimated electrical sector (0-5) from Hall sensors
             * Useful for six-step commutation
             */
            int getSector() const;

            /**
             * Get coarse electrical angle estimate from Hall sensors
             * Returns the center angle of the current sector
             */
            double getElectricalAngleEstimate() const;

        private:
            HallSensorConfig config_;
            bool hall_a_ = false;
            bool hall_b_ = false;
            bool hall_c_ = false;

            // Previous states for hysteresis
            bool prev_hall_a_ = false;
            bool prev_hall_b_ = false;
            bool prev_hall_c_ = false;
        };

        /**
         * Configuration for absolute encoder
         */
        struct EncoderConfig
        {
            // Resolution in bits (8, 10, 12, 14, etc.)
            int resolution_bits = 12;

            // Mechanical zero offset (radians)
            // This is the angle offset between the encoder's mechanical zero
            // and the motor's rotor reference position
            double mechanical_zero_offset = 0.0;

            // Index pulse configuration
            bool has_index = true;  // Does encoder have an index/Z pulse?
            double index_width = 0.01;  // Width of index pulse (radians)
        };

        /**
         * Configuration for incremental quadrature encoder
         */
        struct IncrementalEncoderConfig
        {
            // Number of pulses per revolution (PPR)
            // Common values: 100, 200, 360, 400, 500, 600, 1000, 1024, 2000, 2048, 2500
            int pulses_per_revolution = 1000;

            // Phase offset between A and B channels (radians)
            // Typical: π/2 (90°) for quadrature encoding
            double phase_offset = SENSOR_PI / 2.0;

            // Mechanical zero offset (radians)
            double mechanical_zero_offset = 0.0;

            // Index pulse configuration
            bool has_index = true;
            double index_width = 0.01;  // Width of index pulse (radians)
        };

        /**
         * Incremental quadrature encoder
         *
         * Simulates a traditional incremental encoder with A/B quadrature outputs.
         * This type of encoder generates pulses as it rotates, with two channels
         * (A and B) phase-shifted by 90° to allow direction detection.
         *
         * Features:
         * - Configurable pulses per revolution (PPR)
         * - Quadrature output (A and B channels, 90° phase shift)
         * - Direction detection via phase relationship
         * - Optional index pulse (Z channel)
         * - By using both edges of A and B, you get 4x resolution (4*PPR counts/rev)
         */
        class IncrementalEncoder
        {
        public:
            explicit IncrementalEncoder(const IncrementalEncoderConfig &config = IncrementalEncoderConfig());

            /**
             * Update encoder based on current rotor position
             * @param theta_mech Mechanical rotor angle (radians)
             */
            void update(double theta_mech);

            /**
             * Get channel A state (binary output)
             */
            bool getChannelA() const { return channel_a_; }

            /**
             * Get channel B state (binary output)
             */
            bool getChannelB() const { return channel_b_; }

            /**
             * Get index/Z pulse state
             */
            bool getIndexPulse() const { return index_pulse_; }

            /**
             * Get current position count (accumulated from pulse edges)
             * This simulates what a quadrature decoder would output.
             * Uses 4x decoding (both edges of both channels).
             */
            int32_t getCount() const { return count_; }

            /**
             * Get pulses per revolution setting
             */
            int getPulsesPerRev() const { return config_.pulses_per_revolution; }

            /**
             * Get counts per revolution (4x PPR for quadrature decoding)
             */
            int getCountsPerRev() const { return config_.pulses_per_revolution * 4; }

            /**
             * Get estimated velocity based on pulse frequency
             * @param dt Time since last update (seconds)
             * @return Angular velocity (rad/s)
             */
            double getVelocity(double dt) const;

            /**
             * Reset the count (simulate controller reset)
             */
            void reset();

        private:
            IncrementalEncoderConfig config_;
            bool channel_a_ = false;
            bool channel_b_ = false;
            bool index_pulse_ = false;

            // Previous states for edge detection
            bool prev_channel_a_ = false;
            bool prev_channel_b_ = false;

            // Accumulated count (quadrature decoding)
            int32_t count_ = 0;
            int32_t prev_count_ = 0;

            // For velocity calculation
            double prev_angle_ = 0.0;
        };

        /**
         * Absolute rotary encoder sensor
         *
         * Simulates an absolute encoder with configurable resolution.
         * Provides:
         * - Absolute position within one revolution (0 to 2^resolution_bits - 1)
         * - Optional index pulse (once per revolution)
         * - Mechanical zero offset to simulate encoder mounting offset
         */
        class Encoder
        {
        public:
            explicit Encoder(const EncoderConfig &config = EncoderConfig());

            /**
             * Update encoder reading based on rotor position
             * @param theta_mech Mechanical rotor angle (radians)
             */
            void update(double theta_mech);

            /**
             * Get raw encoder count (0 to counts_per_rev - 1)
             */
            uint32_t getCounts() const { return counts_; }

            /**
             * Get encoder counts per revolution
             */
            uint32_t getCountsPerRev() const { return counts_per_rev_; }

            /**
             * Get encoder angle in radians (accounting for zero offset)
             * This returns the angle as measured by the encoder, which may differ
             * from the true rotor angle by the mechanical_zero_offset
             */
            double getAngle() const;

            /**
             * Get encoder angle in radians (raw, without zero offset)
             */
            double getAngleRaw() const;

            /**
             * Get index pulse state
             * @return true when index pulse is active
             */
            bool getIndexPulse() const { return index_pulse_; }

            /**
             * Convert encoder counts to mechanical angle (radians)
             */
            double countsToAngle(uint32_t counts) const;

            /**
             * Convert mechanical angle to encoder counts
             */
            uint32_t angleToCounts(double angle) const;

            /**
             * Reset encoder count (simulate power cycle or reset command)
             */
            void reset();

        private:
            EncoderConfig config_;
            uint32_t counts_ = 0;
            uint32_t counts_per_rev_ = 0;
            bool index_pulse_ = false;
            double current_angle_ = 0.0;
        };

        /**
         * Sensor suite combining multiple sensor types
         *
         * This class provides a convenient way to simulate a complete motor
         * sensor system with Hall sensors and/or encoder.
         */
        struct SensorSuite
        {
            HallSensorArray hall_sensors;
            Encoder encoder;

            bool enable_hall = false;
            bool enable_encoder = false;

            SensorSuite() = default;

            /**
             * Configure and enable Hall sensors
             */
            void enableHallSensors(const HallSensorConfig &config)
            {
                hall_sensors = HallSensorArray(config);
                enable_hall = true;
            }

            /**
             * Configure and enable encoder
             */
            void enableEncoder(const EncoderConfig &config)
            {
                encoder = Encoder(config);
                enable_encoder = true;
            }

            /**
             * Update all enabled sensors
             */
            void update(double theta_mech)
            {
                if (enable_hall)
                    hall_sensors.update(theta_mech);
                if (enable_encoder)
                    encoder.update(theta_mech);
            }
        };

    } // namespace simulator
} // namespace modules
