#include "communication.hpp"
#include "drivers/encoders.hpp"
#include "drivers/imu.hpp"
#include "drivers/motors.hpp"
#include "drivers/velocity.hpp"
#include "drivers/wheel_pi.hpp"

void setup()
{
    uart_comm::init();
    motors::init();
    encoders::init();
    encoders::reset();
    wheel_velocity::init();
    wheel_pi::init();
    imu::init();
}

void loop()
{
    // 1. Process incoming commands.
    while (uart_config::SerialPort.available())
    {
        uart_comm::process_byte(
            static_cast<uint8_t>(
                uart_config::SerialPort.read()));
    }

    // 2. Enforce the existing motor watchdog.
    uart_comm::check_motor_watchdog();

    // 3. Run the wheel velocity controllers at 50 Hz.
    constexpr uint32_t CONTROL_PERIOD_US = 20000;

    static uint32_t last_control_us = micros();

    static encoders::EncoderData encoder_data{};
    static wheel_velocity::VelocityData velocity_data{};

    const uint32_t now_us = micros();

    if (now_us - last_control_us >= CONTROL_PERIOD_US)
    {
        const uint32_t elapsed_us = now_us - last_control_us;
        last_control_us = now_us;

        const float dt_s = elapsed_us * 1.0e-6f;

        encoder_data = encoders::read();
        velocity_data = wheel_velocity::update(encoder_data);

        float target_left_mps = 0.0f;
        float target_right_mps = 0.0f;

        const bool command_active =
            uart_comm::get_motor_command(
                target_left_mps,
                target_right_mps);

        // Never apply PI output without an active command
        // and a valid encoder-derived velocity measurement.
        if (!command_active || !velocity_data.valid)
        {
            wheel_pi::reset();
            motors::set_speed(0, 0);
        }
        else
        {
            const auto output = wheel_pi::update(
                target_left_mps,
                target_right_mps,
                velocity_data.left_mps,
                velocity_data.right_mps,
                dt_s);

            motors::set_speed(
                output.left_pwm,
                output.right_pwm);
        }
    }

    // 4. Keep encoder and IMU telemetry at 20 Hz.
    static uint32_t last_tx_ms = 0;

    if (millis() - last_tx_ms >= 50)
    {
        EncoderPacket encoder_packet;

        encoder_packet.left_ticks = encoder_data.left_ticks;
        encoder_packet.right_ticks = encoder_data.right_ticks;
        encoder_packet.timestamp_us = encoder_data.timestamp_us;

        uart_comm::send_encoder_packet(encoder_packet);

        auto imu_data = imu::read();

        ImuPacket imu_packet;

        imu_packet.ax = imu_data.ax;
        imu_packet.ay = imu_data.ay;
        imu_packet.az = imu_data.az;

        imu_packet.gx = imu_data.gx;
        imu_packet.gy = imu_data.gy;
        imu_packet.gz = imu_data.gz;

        imu_packet.timestamp_us = imu_data.timestamp_us;

        uart_comm::send_imu_packet(imu_packet);

        // Debug output is limited to the telemetry rate.
        if (velocity_data.valid)
        {
            Serial.printf(
                "Wheel velocity: L=%.3f m/s, R=%.3f m/s\n",
                velocity_data.left_mps,
                velocity_data.right_mps);
        }

        last_tx_ms = millis();
    }
}
