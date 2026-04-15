#include "motor_controller.h"
#include <driver/gpio.h>

MotorController::MotorController() {}

void MotorController::Initialize() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << kLeftForwardPin) |
                           (1ULL << kLeftBackwardPin) |
                           (1ULL << kRightForwardPin) |
                           (1ULL << kRightBackwardPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    Stop();
}

void MotorController::SetLeft(bool forward, bool backward) {
    gpio_set_level(kLeftForwardPin, forward ? 1 : 0);
    gpio_set_level(kLeftBackwardPin, backward ? 1 : 0);
}

void MotorController::SetRight(bool forward, bool backward) {
    gpio_set_level(kRightForwardPin, forward ? 1 : 0);
    gpio_set_level(kRightBackwardPin, backward ? 1 : 0);
}

void MotorController::Forward() {
    SetLeft(true, false);
    SetRight(true, false);
}

void MotorController::Backward() {
    SetLeft(false, true);
    SetRight(false, true);
}

void MotorController::Left() {
    SetLeft(false, true);
    SetRight(true, false);
}

void MotorController::Right() {
    SetLeft(true, false);
    SetRight(false, true);
}

void MotorController::Stop() {
    SetLeft(false, false);
    SetRight(false, false);
}
