#ifndef SERVO_CONTROLLER_H_
#define SERVO_CONTROLLER_H_

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <string>
#include <functional>
#include <vector>
#include <chrono>  // 【新增】用于时间戳记录

class ServoController {
public:
    // 舵机配置结构体
    struct ServoConfig {
        uint8_t channel;        // PCA9685通道 (0-15)
        std::string name;       // 舵机名称
        int min_angle;          // 最小角度
        int max_angle;          // 最大角度
        int default_angle;      // 默认角度
        int min_pulse_us;       // 最小脉冲宽度(us)
        int max_pulse_us;       // 最大脉冲宽度(us)
    };

    // 语音命令回调类型
    using VoiceCommandCallback = std::function<void(const std::string& command, int angle)>;

    ServoController();
    ~ServoController();

    // 初始化
    esp_err_t Initialize(i2c_master_bus_handle_t i2c_bus);

    // 添加舵机配置
    void AddServo(const ServoConfig& config);

    // 控制舵机
    esp_err_t SetAngle(uint8_t channel, int angle);
    esp_err_t SetAngleByName(const std::string& name, int angle);

    // 获取舵机状态
    int GetCurrentAngle(uint8_t channel) const;
    int GetCurrentAngleByName(const std::string& name) const;

    // 语音命令处理
    bool ProcessVoiceCommand(const std::string& text);

    // 注册语音命令回调
    void SetVoiceCommandCallback(VoiceCommandCallback callback);

    // 复位所有舵机到默认位置
    void ResetAllToDefault();

    // 检查是否已初始化
    bool IsInitialized() const { return initialized_; }

private:
    static constexpr uint8_t kPca9685Address = 0x40;
    static constexpr uint8_t kMode1 = 0x00;
    static constexpr uint8_t kMode2 = 0x01;
    static constexpr uint8_t kPwmBase = 0x06;
    static constexpr uint8_t kPrescale = 0xFE;
    static constexpr int kOscillatorHz = 25000000;
    static constexpr int kPwmFreqHz = 50;
    
    // 【新增】指令去重控制
    static constexpr int kMinCommandIntervalMs = 1000;  // 最小命令间隔1秒

    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9685_handle_;
    bool initialized_;
    std::vector<ServoConfig> servos_;
    std::vector<int> current_angles_;
    VoiceCommandCallback voice_callback_;
    
    // 【新增】去重相关成员变量
    std::string last_command_;           // 上次执行的命令
    std::chrono::steady_clock::time_point last_command_time_;  // 上次执行时间

    // 私有方法
    esp_err_t WriteRegister(uint8_t reg, uint8_t value);
    esp_err_t WritePwm(uint8_t channel, int on, int off);
    int AngleToPwmValue(const ServoConfig& config, int angle) const;
    int FindServoByName(const std::string& name) const;
    bool ParseVoiceCommand(const std::string& text, std::string& servo_name, int& angle);
};

#endif // SERVO_CONTROLLER_H_
