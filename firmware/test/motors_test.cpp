#include <unity.h>

#include "mocks/Arduino.h"
#include "mocks/arduino_mock.hpp"

#include "drivers/motors.hpp"

void setUp() {
    ArduinoMock::reset();
}

void tearDown() {
}

void assert_motor_pwm(int left, int right) {
    TEST_ASSERT_EQUAL(
        left,
        ArduinoMock::state().pwm_values[
            motor_config::LEFT_PWM_CHANNEL
        ]
    );

    TEST_ASSERT_EQUAL(
        right,
        ArduinoMock::state().pwm_values[
            motor_config::RIGHT_PWM_CHANNEL
        ]
    );
}

void test_motor_speed_forward() {
    motors::set_speed(100, 150);
    assert_motor_pwm(100, 150);
}

void test_motor_speed_maximum() {
    motors::set_speed(255, 255);
    assert_motor_pwm(255, 255);
}

void test_motor_speed_minimum() {
    motors::set_speed(-255, -255);
    assert_motor_pwm(255, 255);
}

void test_motor_speed_above_maximum() {
    motors::set_speed(300, 400);
    assert_motor_pwm(255, 255);
}

void test_motor_speed_below_minimum() {
    motors::set_speed(-300, -400);
    assert_motor_pwm(255, 255);
}

void test_motor_speed_left_stopped() {
    motors::set_speed(0, 150);
    assert_motor_pwm(0, 150);
}

void test_motor_speed_right_stopped() {
    motors::set_speed(150, 0);
    assert_motor_pwm(150, 0);
}

void test_motor_speed_both_stopped() {
    motors::set_speed(0, 0);
    assert_motor_pwm(0, 0);
}

void test_motor_speed_mixed_directions() {
    motors::set_speed(-100, 150);
    assert_motor_pwm(100, 150);
}

void assert_motor_direction(
    uint8_t pin_a,
    uint8_t pin_b,
    int effective_speed
) {
    uint8_t expected_a = effective_speed < 0 ? HIGH : LOW;
    uint8_t expected_b = effective_speed > 0 ? HIGH : LOW;

    TEST_ASSERT_EQUAL(expected_a, ArduinoMock::state().digital_values[pin_a]);
    TEST_ASSERT_EQUAL(expected_b, ArduinoMock::state().digital_values[pin_b]);
}

void test_motor_direction_forward_reverse() {
    motors::set_speed(100, -100);

    assert_motor_direction(
        motor_config::LEFT_IN1_PIN,
        motor_config::LEFT_IN2_PIN,
        100 * motor_config::LEFT_MOTOR_DIR
    );

    assert_motor_direction(
        motor_config::RIGHT_IN3_PIN,
        motor_config::RIGHT_IN4_PIN,
        -100 * motor_config::RIGHT_MOTOR_DIR
    );
}

void test_motor_direction_reverse_forward() {
    motors::set_speed(-100, 100);

    assert_motor_direction(
        motor_config::LEFT_IN1_PIN,
        motor_config::LEFT_IN2_PIN,
        -100 * motor_config::LEFT_MOTOR_DIR
    );

    assert_motor_direction(
        motor_config::RIGHT_IN3_PIN,
        motor_config::RIGHT_IN4_PIN,
        100 * motor_config::RIGHT_MOTOR_DIR
    );
}

void test_motor_direction_stop() {
    motors::set_speed(0, 0);

    assert_motor_direction(
        motor_config::LEFT_IN1_PIN,
        motor_config::LEFT_IN2_PIN,
        0
    );

    assert_motor_direction(
        motor_config::RIGHT_IN3_PIN,
        motor_config::RIGHT_IN4_PIN,
        0
    );
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_motor_speed_forward);
    RUN_TEST(test_motor_speed_maximum);
    RUN_TEST(test_motor_speed_minimum);
    RUN_TEST(test_motor_speed_above_maximum);
    RUN_TEST(test_motor_speed_below_minimum);
    RUN_TEST(test_motor_speed_left_stopped);
    RUN_TEST(test_motor_speed_right_stopped);
    RUN_TEST(test_motor_speed_both_stopped);
    RUN_TEST(test_motor_speed_mixed_directions);

    return UNITY_END();
}