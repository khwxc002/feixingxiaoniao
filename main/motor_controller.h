#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <driver/gpio.h>

class MotorController {
public:
    MotorController();
    void Initialize();
    void Forward();
    void Backward();
    void Left();
    void Right();
    void Stop();

private:
    static constexpr gpio_num_t kLeftForwardPin = GPIO_NUM_10;
    static constexpr gpio_num_t kLeftBackwardPin = GPIO_NUM_11;
    static constexpr gpio_num_t kRightForwardPin = GPIO_NUM_12;
    static constexpr gpio_num_t kRightBackwardPin = GPIO_NUM_13;

    void SetLeft(bool forward, bool backward);
    void SetRight(bool forward, bool backward);
};

#endif // MOTOR_CONTROLLER_H_
