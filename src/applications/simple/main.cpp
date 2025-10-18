#include "modules/simulator/simulator.h"
#include "modules/simulator/bldc_motor.h"
#include "modules/simulator/sensors.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace modules::simulator;

/**
 * Simple example: Open-loop voltage control (six-step commutation approximation)
 */
void example_six_step()
{
    std::cout << "\n=== Six-Step Commutation Example (Open Loop) ===\n" << std::endl;

    // Configure a small hobby motor
    BldcMotorConfig motor_config;
    motor_config.num_pole_pairs = 7;           // 14 poles
    motor_config.num_phases = 3;
    motor_config.num_slots = 12;
    motor_config.turns_per_coil = 30;
    motor_config.wire_diameter = 0.3e-3;       // 0.3mm wire
    motor_config.magnet_flux_linkage = 0.005;  // 5 mWb
    motor_config.rotor_inertia = 5e-6;         // Small rotor
    motor_config.viscous_friction = 5e-6;
    motor_config.enable_cogging = true;
    motor_config.cogging_amplitude = 0.001;    // 1 mNm cogging

    motor_config.calculateDerivedParameters();

    BldcMotor motor(motor_config);

    // Create simulator
    SimulatorConfig sim_config;
    sim_config.time_step = 0.0001;  // 100 μs timestep (10 kHz)
    sim_config.realtime = false;

    Simulator sim;

    // Apply simple trapezoidal commutation pattern
    double supply_voltage = 12.0; // 12V supply

    std::cout << "Motor Parameters:" << std::endl;
    std::cout << "  Pole pairs: " << motor_config.num_pole_pairs << std::endl;
    std::cout << "  Phase resistance: " << motor_config.phase_resistance * 1000.0 << " mOhm" << std::endl;
    std::cout << "  Phase inductance: " << motor_config.phase_inductance * 1e6 << " μH" << std::endl;
    std::cout << "  Torque constant: " << motor_config.kt * 1000.0 << " mNm/A" << std::endl;
    std::cout << "  Back-EMF constant: " << motor_config.ke * 1000.0 << " mV/(rad/s)" << std::endl;
    std::cout << "\nSimulating...\n" << std::endl;

    // Run simulation for 0.1 seconds
    double sim_time = 0.0;
    double sim_duration = 0.1;
    int output_decimation = 100; // Print every 100 steps

    int step_count = 0;

    while (sim_time < sim_duration)
    {
        // Simple six-step commutation based on electrical angle
        double theta = motor.state().theta;
        int sector = static_cast<int>(theta / (PI / 3.0)) % 6;

        double v_a = 0.0, v_b = 0.0, v_c = 0.0;

        // Six-step pattern (simplified)
        switch (sector)
        {
        case 0: v_a = supply_voltage; v_b = 0; v_c = -supply_voltage; break;
        case 1: v_a = supply_voltage; v_b = -supply_voltage; v_c = 0; break;
        case 2: v_a = 0; v_b = supply_voltage; v_c = -supply_voltage; break;
        case 3: v_a = -supply_voltage; v_b = supply_voltage; v_c = 0; break;
        case 4: v_a = -supply_voltage; v_b = 0; v_c = supply_voltage; break;
        case 5: v_a = 0; v_b = -supply_voltage; v_c = supply_voltage; break;
        }

        motor.setPhaseVoltages(v_a, v_b, v_c);
        motor.step(sim_config.time_step);

        if (step_count % output_decimation == 0)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "t=" << sim_time * 1000.0 << " ms  "
                      << "θ=" << motor.position() << " rad  "
                      << "ω=" << motor.velocityRPM() << " RPM  "
                      << "T=" << motor.torque() * 1000.0 << " mNm  "
                      << "I_rms=" << std::sqrt((motor.current(0) * motor.current(0) +
                                                motor.current(1) * motor.current(1) +
                                                motor.current(2) * motor.current(2)) / 3.0)
                      << " A" << std::endl;
        }

        sim_time += sim_config.time_step;
        step_count++;
    }

    std::cout << "\nFinal velocity: " << motor.velocityRPM() << " RPM" << std::endl;
}

/**
 * Six-step commutation with BEMF zero-crossing detection (sensorless)
 */
void example_six_step_sensorless()
{
    std::cout << "\n=== Six-Step Sensorless (BEMF Zero-Crossing) ===\n" << std::endl;

    // Configure motor
    BldcMotorConfig motor_config;
    motor_config.num_pole_pairs = 7;
    motor_config.magnet_flux_linkage = 0.005;
    motor_config.rotor_inertia = 5e-6;
    motor_config.viscous_friction = 5e-6;
    motor_config.phase_resistance = 1.0;
    motor_config.phase_inductance = 0.001;

    motor_config.calculateDerivedParameters();

    BldcMotor motor(motor_config);

    // Simulation config
    SimulatorConfig sim_config;
    sim_config.time_step = 0.00001;  // 10 μs for better zero-crossing detection

    double supply_voltage = 12.0;
    int current_sector = 0;
    bool zero_crossing_detected = false;
    double last_floating_voltage = 0.0;

    std::cout << "Starting with open-loop to build up speed...\n" << std::endl;

    // Initial open-loop startup
    double sim_time = 0.0;
    while (sim_time < 0.02)  // 20ms startup
    {
        // Simple open-loop pattern for startup
        double theta = motor.state().theta;
        current_sector = static_cast<int>(theta / (PI / 3.0)) % 6;

        double v_a = 0.0, v_b = 0.0, v_c = 0.0;
        switch (current_sector)
        {
        case 0: v_a = supply_voltage; v_b = 0; v_c = -supply_voltage; break;
        case 1: v_a = supply_voltage; v_b = -supply_voltage; v_c = 0; break;
        case 2: v_a = 0; v_b = supply_voltage; v_c = -supply_voltage; break;
        case 3: v_a = -supply_voltage; v_b = supply_voltage; v_c = 0; break;
        case 4: v_a = -supply_voltage; v_b = 0; v_c = supply_voltage; break;
        case 5: v_a = 0; v_b = -supply_voltage; v_c = supply_voltage; break;
        }

        motor.setPhaseVoltages(v_a, v_b, v_c);
        motor.step(sim_config.time_step);
        sim_time += sim_config.time_step;
    }

    std::cout << "Switched to sensorless mode (BEMF zero-crossing)\n" << std::endl;
    std::cout << "Sector  Floating  BEMF_Float   Speed" << std::endl;
    std::cout << "------  --------  ----------   -----" << std::endl;

    // Now run with BEMF zero-crossing detection
    int output_counter = 0;
    while (sim_time < 0.05)  // Run for total 50ms
    {
        // Determine which phase is floating based on current sector
        int floating_phase = 0;
        switch (current_sector)
        {
        case 0: floating_phase = 1; break; // Phase B floating
        case 1: floating_phase = 2; break; // Phase C floating
        case 2: floating_phase = 0; break; // Phase A floating
        case 3: floating_phase = 1; break; // Phase B floating
        case 4: floating_phase = 2; break; // Phase C floating
        case 5: floating_phase = 0; break; // Phase A floating
        }

        // Measure floating phase voltage
        double floating_voltage = motor.measureFloatingPhaseVoltage(floating_phase);

        // Detect zero crossing (with some hysteresis to avoid noise)
        const double hysteresis = 0.1; // 100mV hysteresis
        if (!zero_crossing_detected && last_floating_voltage < -hysteresis && floating_voltage > hysteresis)
        {
            zero_crossing_detected = true;
            // Advance to next sector (with 30° offset for optimal commutation)
            current_sector = (current_sector + 1) % 6;

            if (output_counter % 5 == 0)  // Print every 5th commutation
            {
                std::cout << std::fixed << std::setprecision(2)
                          << "  " << current_sector
                          << "       " << (char)('A' + floating_phase)
                          << "        " << floating_voltage << " V    "
                          << motor.velocityRPM() << " RPM" << std::endl;
            }
            output_counter++;
        }
        else if (zero_crossing_detected && floating_voltage < 0)
        {
            zero_crossing_detected = false;
        }

        last_floating_voltage = floating_voltage;

        // Apply commutation pattern
        double v_a = 0.0, v_b = 0.0, v_c = 0.0;
        switch (current_sector)
        {
        case 0: v_a = supply_voltage; v_b = 0; v_c = -supply_voltage; break;
        case 1: v_a = supply_voltage; v_b = -supply_voltage; v_c = 0; break;
        case 2: v_a = 0; v_b = supply_voltage; v_c = -supply_voltage; break;
        case 3: v_a = -supply_voltage; v_b = supply_voltage; v_c = 0; break;
        case 4: v_a = -supply_voltage; v_b = 0; v_c = supply_voltage; break;
        case 5: v_a = 0; v_b = -supply_voltage; v_c = supply_voltage; break;
        }

        motor.setPhaseVoltages(v_a, v_b, v_c);
        motor.step(sim_config.time_step);

        sim_time += sim_config.time_step;
    }

    std::cout << "\nFinal velocity: " << motor.velocityRPM() << " RPM" << std::endl;
}

/**
 * Simple FOC example with constant torque command
 */
void example_foc()
{
    std::cout << "\n=== Field-Oriented Control (FOC) Example ===\n" << std::endl;

    // Configure motor
    BldcMotorConfig motor_config;
    motor_config.num_pole_pairs = 4;
    motor_config.magnet_flux_linkage = 0.008;
    motor_config.rotor_inertia = 1e-5;
    motor_config.viscous_friction = 1e-5;
    motor_config.phase_resistance = 0.5;    // 0.5 Ohm
    motor_config.phase_inductance = 0.001;  // 1 mH

    motor_config.calculateDerivedParameters();

    BldcMotor motor(motor_config);

    // Simulation config
    SimulatorConfig sim_config;
    sim_config.time_step = 0.0001;  // 100 μs

    // Simple PI controller for current (very basic)
    double i_q_target = 2.0;  // 2A torque-producing current
    double i_d_target = 0.0;  // Zero flux-weakening current

    std::cout << "FOC Simulation with i_q = " << i_q_target << " A\n" << std::endl;

    double sim_time = 0.0;
    double sim_duration = 0.05;
    int output_decimation = 100;
    int step_count = 0;

    while (sim_time < sim_duration)
    {
        // Very simple voltage control (proportional only - not a real PI)
        // In practice you'd use proper PI controllers
        double v_d = 10.0 * (i_d_target - motor.state().i_d);
        double v_q = 10.0 * (i_q_target - motor.state().i_q);

        // Clamp voltages
        double v_max = 20.0;
        v_d = std::max(-v_max, std::min(v_max, v_d));
        v_q = std::max(-v_max, std::min(v_max, v_q));

        motor.setDQVoltages(v_d, v_q);
        motor.step(sim_config.time_step);

        if (step_count % output_decimation == 0)
        {
            std::cout << std::fixed << std::setprecision(4)
                      << "t=" << sim_time * 1000.0 << " ms  "
                      << "ω=" << motor.velocityRPM() << " RPM  "
                      << "i_d=" << motor.state().i_d << " A  "
                      << "i_q=" << motor.state().i_q << " A  "
                      << "T=" << motor.torque() * 1000.0 << " mNm"
                      << std::endl;
        }

        sim_time += sim_config.time_step;
        step_count++;
    }

    std::cout << "\nFinal velocity: " << motor.velocityRPM() << " RPM" << std::endl;
}

/**
 * Sensor demonstration - Hall sensors and Encoder
 */
void example_sensors()
{
    std::cout << "\n=== Sensor Demonstration ===\n" << std::endl;

    // Configure motor
    BldcMotorConfig motor_config;
    motor_config.num_pole_pairs = 7;
    motor_config.magnet_flux_linkage = 0.005;
    motor_config.rotor_inertia = 5e-6;
    motor_config.viscous_friction = 5e-6;
    motor_config.phase_resistance = 1.0;
    motor_config.phase_inductance = 0.001;

    motor_config.calculateDerivedParameters();
    BldcMotor motor(motor_config);

    // Configure Hall sensors
    HallSensorConfig hall_config;
    hall_config.num_pole_pairs = motor_config.num_pole_pairs;
    hall_config.sensor_offset_a = 0.0;  // Aligned with rotor
    hall_config.sensor_spacing = 2.0 * PI / 3.0;  // 120° electrical

    HallSensorArray hall_sensors(hall_config);

    // Configure encoder with different resolutions
    std::cout << "Testing different encoder resolutions:\n" << std::endl;

    for (int bits : {8, 10, 12})
    {
        EncoderConfig enc_config;
        enc_config.resolution_bits = bits;
        enc_config.mechanical_zero_offset = PI / 4.0;  // 45° mounting offset
        enc_config.has_index = true;

        Encoder encoder(enc_config);

        std::cout << bits << "-bit encoder (" << encoder.getCountsPerRev()
                  << " counts/rev, zero offset = " << (enc_config.mechanical_zero_offset * 180.0 / PI)
                  << "°)" << std::endl;
        std::cout << "  Angle   Counts  Index  Resolution" << std::endl;

        for (double angle_deg = 0.0; angle_deg <= 360.0; angle_deg += 45.0)
        {
            double angle_rad = angle_deg * PI / 180.0;
            encoder.update(angle_rad);

            double angle_resolution = (2.0 * PI) / encoder.getCountsPerRev();
            double angle_resolution_deg = angle_resolution * 180.0 / PI;

            std::cout << std::fixed << std::setprecision(1)
                      << "  " << std::setw(5) << angle_deg << "°  "
                      << std::setw(5) << encoder.getCounts() << "   "
                      << (encoder.getIndexPulse() ? "YES" : " NO") << "   "
                      << std::setprecision(3) << angle_resolution_deg << "°/count"
                      << std::endl;
        }
        std::cout << std::endl;
    }

    // Now demonstrate Hall sensors with motor spinning
    std::cout << "\nHall Sensor Commutation Example:" << std::endl;
    std::cout << "Running motor with Hall sensor-based six-step commutation\n" << std::endl;

    // Setup encoder for position tracking
    EncoderConfig enc_config;
    enc_config.resolution_bits = 12;
    enc_config.mechanical_zero_offset = 0.1;  // Small offset
    Encoder encoder(enc_config);

    SimulatorConfig sim_config;
    sim_config.time_step = 0.0001;

    double supply_voltage = 12.0;
    double sim_time = 0.0;

    std::cout << "Time(ms)  Hall  Sector  Enc_Angle  Enc_Counts  Speed(RPM)" << std::endl;
    std::cout << "--------  ----  ------  ---------  ----------  ----------" << std::endl;

    int output_counter = 0;
    while (sim_time < 0.05)
    {
        // Update sensors
        hall_sensors.update(motor.position());
        encoder.update(motor.position());

        // Get Hall sensor sector for commutation
        int sector = hall_sensors.getSector();

        // Apply six-step commutation based on Hall sensors
        double v_a = 0.0, v_b = 0.0, v_c = 0.0;
        if (sector >= 0)  // Valid Hall state
        {
            switch (sector)
            {
            case 0: v_a = supply_voltage; v_b = 0; v_c = -supply_voltage; break;
            case 1: v_a = supply_voltage; v_b = -supply_voltage; v_c = 0; break;
            case 2: v_a = 0; v_b = supply_voltage; v_c = -supply_voltage; break;
            case 3: v_a = -supply_voltage; v_b = supply_voltage; v_c = 0; break;
            case 4: v_a = -supply_voltage; v_b = 0; v_c = supply_voltage; break;
            case 5: v_a = 0; v_b = -supply_voltage; v_c = supply_voltage; break;
            }
        }

        motor.setPhaseVoltages(v_a, v_b, v_c);
        motor.step(sim_config.time_step);

        // Print output every 5ms
        if (output_counter % 500 == 0)
        {
            uint8_t hall_state = hall_sensors.getHallState();
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(7) << sim_time * 1000.0 << "  "
                      << ((hall_state & 0x4) ? "1" : "0")
                      << ((hall_state & 0x2) ? "1" : "0")
                      << ((hall_state & 0x1) ? "1" : "0") << "   "
                      << std::setw(4) << sector << "    "
                      << std::setprecision(3) << std::setw(7) << encoder.getAngle() << "    "
                      << std::setw(8) << encoder.getCounts() << "    "
                      << std::setprecision(1) << std::setw(8) << motor.velocityRPM()
                      << std::endl;
        }

        sim_time += sim_config.time_step;
        output_counter++;
    }

    std::cout << "\nFinal state:" << std::endl;
    std::cout << "  Motor speed: " << motor.velocityRPM() << " RPM" << std::endl;
    std::cout << "  Hall state: " << (int)hall_sensors.getHallState()
              << " (sector " << hall_sensors.getSector() << ")" << std::endl;
    std::cout << "  Encoder: " << encoder.getCounts() << " counts, "
              << (encoder.getAngle() * 180.0 / PI) << "°" << std::endl;
}

/**
 * Incremental encoder demonstration
 */
void example_incremental_encoder()
{
    std::cout << "\n=== Incremental Encoder Demonstration ===\n" << std::endl;

    // Test different PPR values
    std::cout << "Testing incremental encoders with different PPR values:\n" << std::endl;

    for (int ppr : {100, 360, 1000, 2048})
    {
        IncrementalEncoderConfig config;
        config.pulses_per_revolution = ppr;
        config.phase_offset = PI / 2.0;  // 90° quadrature
        config.mechanical_zero_offset = 0.0;
        config.has_index = true;

        IncrementalEncoder encoder(config);

        std::cout << "PPR = " << ppr << " (" << encoder.getCountsPerRev()
                  << " counts/rev with 4x decoding)" << std::endl;
        std::cout << "  Angle    Ch_A  Ch_B  Index  Count" << std::endl;

        int prev_count = 0;

        for (double angle_deg = 0.0; angle_deg <= 370.0; angle_deg += 10.0)
        {
            double angle_rad = angle_deg * PI / 180.0;
            encoder.update(angle_rad);

            // Only print when something interesting happens
            if (angle_deg == 0.0 || angle_deg == 90.0 || angle_deg == 180.0 ||
                angle_deg == 270.0 || angle_deg == 360.0 ||
                encoder.getCount() != prev_count)
            {
                std::cout << "  " << std::setw(5) << std::fixed << std::setprecision(1) << angle_deg << "°   "
                          << (encoder.getChannelA() ? "1" : "0") << "     "
                          << (encoder.getChannelB() ? "1" : "0") << "     "
                          << (encoder.getIndexPulse() ? "YES" : " NO") << "   "
                          << std::setw(6) << encoder.getCount()
                          << std::endl;
                prev_count = encoder.getCount();
            }
        }
        std::cout << std::endl;
    }

    // Now demonstrate with a spinning motor
    std::cout << "Incremental Encoder with Spinning Motor:\n" << std::endl;

    BldcMotorConfig motor_config;
    motor_config.num_pole_pairs = 7;
    motor_config.magnet_flux_linkage = 0.005;
    motor_config.rotor_inertia = 5e-6;
    motor_config.viscous_friction = 5e-6;
    motor_config.phase_resistance = 1.0;
    motor_config.phase_inductance = 0.001;
    motor_config.calculateDerivedParameters();

    BldcMotor motor(motor_config);

    IncrementalEncoderConfig inc_config;
    inc_config.pulses_per_revolution = 1000;
    inc_config.phase_offset = PI / 2.0;
    inc_config.mechanical_zero_offset = 0.1;  // 5.7° offset
    IncrementalEncoder inc_encoder(inc_config);

    SimulatorConfig sim_config;
    sim_config.time_step = 0.0001;

    double supply_voltage = 12.0;
    double sim_time = 0.0;

    std::cout << "Time(ms)  Ch_A  Ch_B  Count    Speed(RPM)  Direction" << std::endl;
    std::cout << "--------  ----  ----  -------  ----------  ---------" << std::endl;

    int output_counter = 0;
    int32_t last_count = 0;

    while (sim_time < 0.03)
    {
        // Simple open-loop commutation
        double theta = motor.state().theta;
        int sector = static_cast<int>(theta / (PI / 3.0)) % 6;

        double v_a = 0.0, v_b = 0.0, v_c = 0.0;
        switch (sector)
        {
        case 0: v_a = supply_voltage; v_b = 0; v_c = -supply_voltage; break;
        case 1: v_a = supply_voltage; v_b = -supply_voltage; v_c = 0; break;
        case 2: v_a = 0; v_b = supply_voltage; v_c = -supply_voltage; break;
        case 3: v_a = -supply_voltage; v_b = supply_voltage; v_c = 0; break;
        case 4: v_a = -supply_voltage; v_b = 0; v_c = supply_voltage; break;
        case 5: v_a = 0; v_b = -supply_voltage; v_c = supply_voltage; break;
        }

        motor.setPhaseVoltages(v_a, v_b, v_c);
        motor.step(sim_config.time_step);

        // Update encoder
        inc_encoder.update(motor.position());

        // Print every 3ms
        if (output_counter % 300 == 0)
        {
            int32_t count_diff = inc_encoder.getCount() - last_count;
            std::string direction = (count_diff > 0) ? "CW" : (count_diff < 0) ? "CCW" : "STOP";

            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(7) << sim_time * 1000.0 << "   "
                      << (inc_encoder.getChannelA() ? "1" : "0") << "     "
                      << (inc_encoder.getChannelB() ? "1" : "0") << "    "
                      << std::setw(6) << inc_encoder.getCount() << "   "
                      << std::setprecision(1) << std::setw(9) << motor.velocityRPM() << "   "
                      << direction
                      << std::endl;

            last_count = inc_encoder.getCount();
        }

        sim_time += sim_config.time_step;
        output_counter++;
    }

    std::cout << "\nFinal encoder count: " << inc_encoder.getCount()
              << " (Motor: " << motor.velocityRPM() << " RPM)" << std::endl;
}

int main(int argc, char *argv[])
{
    std::cout << "BLDC Motor Simulation Examples" << std::endl;
    std::cout << "===============================" << std::endl;

    // Run examples
    example_six_step();
    example_six_step_sensorless();
    example_foc();
    example_sensors();
    example_incremental_encoder();

    std::cout << "\nSimulation complete!" << std::endl;

    return 0;
}