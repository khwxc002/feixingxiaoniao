#include "servo_controller.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <cmath>
#include <cstring>

static const char* TAG = "ServoController";

ServoController::ServoController()
    : i2c_bus_(nullptr), pca9685_handle_(nullptr), initialized_(false) {}

ServoController::~ServoController() {
    if (pca9685_handle_ != nullptr) {
        i2c_master_bus_rm_device(pca9685_handle_);
    }
}

esp_err_t ServoController::Initialize(i2c_master_bus_handle_t i2c_bus) {
    if (initialized_ || i2c_bus == nullptr) {
        return initialized_ ? ESP_OK : ESP_ERR_INVALID_ARG;
    }

    i2c_bus_ = i2c_bus;

    // 配置PCA9685设备
    i2c_device_config_t pca9685_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = kPca9685Address,
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags =
            {
                .disable_ack_check = 0,
            },
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus_, &pca9685_config, &pca9685_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PCA9685 device: %s", esp_err_to_name(err));
        return err;
    }

    // 初始化PCA9685
    err = WriteRegister(kMode2, 0x04);  // OUTDRV
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 mode2 init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = WriteRegister(kMode1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 mode1 init failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = WriteRegister(kMode1, 0x10);  // sleep
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 enter sleep failed: %s", esp_err_to_name(err));
        return err;
    }

    // 设置PWM频率
    int prescale = static_cast<int>(round((kOscillatorHz / (4096.0 * kPwmFreqHz)) - 1));
    err = WriteRegister(kPrescale, static_cast<uint8_t>(prescale));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 prescale set failed: %s", esp_err_to_name(err));
        return err;
    }

    err = WriteRegister(kMode1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 wake failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = WriteRegister(kMode1, 0x80);  // restart
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 restart failed: %s", esp_err_to_name(err));
        return err;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "ServoController initialized successfully");
    return ESP_OK;
}

void ServoController::AddServo(const ServoConfig& config) {
    if (config.channel >= 16) {
        ESP_LOGE(TAG, "Invalid servo channel: %d", config.channel);
        return;
    }

    // 检查是否已存在
    for (const auto& servo : servos_) {
        if (servo.channel == config.channel) {
            ESP_LOGW(TAG, "Servo channel %d already exists, replacing", config.channel);
            // 移除旧的
            auto it = std::find_if(servos_.begin(), servos_.end(), [config](const ServoConfig& s) {
                return s.channel == config.channel;
            });
            if (it != servos_.end()) {
                servos_.erase(it);
                if (config.channel < current_angles_.size()) {
                    current_angles_[config.channel] = config.default_angle;
                }
            }
            break;
        }
    }

    servos_.push_back(config);

    // 确保current_angles_向量足够大
    if (config.channel >= current_angles_.size()) {
        current_angles_.resize(config.channel + 1, 90);
    }
    current_angles_[config.channel] = config.default_angle;

    // 如果已初始化，设置到默认角度
    if (initialized_) {
        SetAngle(config.channel, config.default_angle);
    }

    ESP_LOGI(TAG, "Added servo: channel=%d, name=%s, default_angle=%d", config.channel,
             config.name.c_str(), config.default_angle);
}

esp_err_t ServoController::SetAngle(uint8_t channel, int angle) {
    if (!initialized_) {
        ESP_LOGE(TAG, "ServoController not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= servos_.size() || channel >= 16) {
        ESP_LOGE(TAG, "Invalid servo channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // 找到对应的舵机配置
    const ServoConfig* config = nullptr;
    for (const auto& servo : servos_) {
        if (servo.channel == channel) {
            config = &servo;
            break;
        }
    }

    if (!config) {
        ESP_LOGE(TAG, "No configuration found for channel %d", channel);
        return ESP_ERR_NOT_FOUND;
    }

    // 限制角度范围
    angle = std::max(config->min_angle, std::min(config->max_angle, angle));

    int off = AngleToPwmValue(*config, angle);
    esp_err_t err = WritePwm(channel, 0, off);
    if (err == ESP_OK) {
        current_angles_[channel] = angle;
        ESP_LOGD(TAG, "Set servo channel %d to angle %d", channel, angle);
    } else {
        ESP_LOGE(TAG, "Failed to set servo channel %d to angle %d: %s", channel, angle,
                 esp_err_to_name(err));
    }

    return err;
}

esp_err_t ServoController::SetAngleByName(const std::string& name, int angle) {
    int index = FindServoByName(name);
    if (index < 0) {
        ESP_LOGE(TAG, "Servo with name '%s' not found", name.c_str());
        return ESP_ERR_NOT_FOUND;
    }

    return SetAngle(servos_[index].channel, angle);
}

int ServoController::GetCurrentAngle(uint8_t channel) const {
    if (channel >= current_angles_.size()) {
        return -1;
    }
    return current_angles_[channel];
}

int ServoController::GetCurrentAngleByName(const std::string& name) const {
    int index = FindServoByName(name);
    if (index < 0) {
        return -1;
    }
    return current_angles_[servos_[index].channel];
}

// bool ServoController::ProcessVoiceCommand(const std::string& text) {
//     std::string servo_name;
//     int angle;

//     // 👇 新增调试日志，必加！
//     ESP_LOGI("SERVO_DEBUG", "收到语音指令：%s", text.c_str());

//     if (!ParseVoiceCommand(text, servo_name, angle)) {
//         // 👇 解析失败时打印，立刻就能看到问题
//         ESP_LOGW("SERVO_DEBUG", "❌ 指令解析失败！servo_name: %s, angle: %d", servo_name.c_str(),
//         angle); return false;
//     }

//     ESP_LOGI("SERVO_DEBUG", "✅ 解析成功！舵机名：%s，目标角度：%d", servo_name.c_str(), angle);

//     // 后面的代码保持不变...
// }

bool ServoController::ProcessVoiceCommand(const std::string& text) {
    std::string servo_name;
    int angle;

    // 👇 新增调试日志，必加！
    ESP_LOGI("SERVO_DEBUG", "收到语音指令：%s", text.c_str());

    if (!ParseVoiceCommand(text, servo_name, angle)) {
        // 👇 解析失败时打印，立刻就能看到问题
        ESP_LOGW("SERVO_DEBUG", "❌ 指令解析失败！servo_name: %s, angle: %d", servo_name.c_str(),
        return false;
    }

    ESP_LOGI("SERVO_DEBUG", "✅ 解析成功！舵机名：%s，目标角度：%d", servo_name.c_str(), angle);
    
    esp_err_t err;
    if (!servo_name.empty()) {
        err = SetAngleByName(servo_name, angle);
    } else {
        // 如果没有指定舵机名称，默认控制第一个舵机
        if (!servos_.empty()) {
            err = SetAngle(servos_[0].channel, angle);
        } else {
            return false;
        }
    }

    if (err == ESP_OK && voice_callback_) {
        voice_callback_(text, angle);
    }

    return err == ESP_OK;
}

void ServoController::SetVoiceCommandCallback(VoiceCommandCallback callback) {
    voice_callback_ = callback;
}

void ServoController::ResetAllToDefault() {
    for (const auto& servo : servos_) {
        SetAngle(servo.channel, servo.default_angle);
    }
}

esp_err_t ServoController::WriteRegister(uint8_t reg, uint8_t value) {
    if (pca9685_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer[2] = {reg, value};
    return i2c_master_transmit(pca9685_handle_, buffer, 2, 100);
}

esp_err_t ServoController::WritePwm(uint8_t channel, int on, int off) {
    if (pca9685_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    int reg = kPwmBase + 4 * channel;
    uint8_t buffer[5] = {static_cast<uint8_t>(reg), static_cast<uint8_t>(on & 0xFF),
                         static_cast<uint8_t>((on >> 8) & 0x0F), static_cast<uint8_t>(off & 0xFF),
                         static_cast<uint8_t>((off >> 8) & 0x0F)};
    return i2c_master_transmit(pca9685_handle_, buffer, 5, 100);
}

int ServoController::AngleToPwmValue(const ServoConfig& config, int angle) const {
    angle = std::max(config.min_angle, std::min(config.max_angle, angle));
    int pulse_us = config.min_pulse_us + (angle - config.min_angle) *
                                             (config.max_pulse_us - config.min_pulse_us) /
                                             (config.max_angle - config.min_angle);
    return (pulse_us * 4096) / (1000000 / kPwmFreqHz);
}

int ServoController::FindServoByName(const std::string& name) const {
    for (size_t i = 0; i < servos_.size(); ++i) {
        if (servos_[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool ServoController::ParseVoiceCommand(const std::string& text, std::string& servo_name,
                                        int& angle) {
    std::string normalized = text;

    // 转换为小写以便匹配
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::tolower);

    // 检查是否包含舵机相关关键词
    if (normalized.find("舵机") == std::string::npos &&
        normalized.find("duoji") == std::string::npos &&
        normalized.find("servo") == std::string::npos) {
        return false;
    }

    // 解析角度
    angle = -1;
    size_t degree_pos = normalized.find("度");
    if (degree_pos != std::string::npos) {
        // 从"度"字前向后查找数字
        size_t start = degree_pos;
        while (start > 0 && !isdigit(normalized[start - 1])) {
            --start;
        }
        while (start > 0 && start < normalized.size() && isdigit(normalized[start - 1])) {
            --start;
        }

        if (start < degree_pos) {
            try {
                angle = std::stoi(normalized.substr(start, degree_pos - start));
            } catch (...) {
                angle = -1;
            }
        }
    }

    // 如果没找到具体角度，检查预设命令
    if (angle < 0) {
        if (normalized.find("归位") != std::string::npos ||
            normalized.find("复位") != std::string::npos ||
            normalized.find("中") != std::string::npos ||
            normalized.find("中间") != std::string::npos) {
            angle = 90;
        } else if (normalized.find("左转") != std::string::npos ||
                   normalized.find("向左") != std::string::npos) {
            angle = 45;
        } else if (normalized.find("右转") != std::string::npos ||
                   normalized.find("向右") != std::string::npos) {
            angle = 135;
        } else if (normalized.find("最大") != std::string::npos ||
                   normalized.find("最左") != std::string::npos) {
            angle = 0;
        } else if (normalized.find("最小") != std::string::npos ||
                   normalized.find("最右") != std::string::npos) {
            angle = 180;
        } else {
            return false;
        }
    }

    // 验证角度范围
    if (angle < 0 || angle > 180) {
        return false;
    }

    // 暂时不支持指定舵机名称，默认使用第一个
    servo_name.clear();

    return true;
}
