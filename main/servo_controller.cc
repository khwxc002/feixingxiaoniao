#include "servo_controller.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <cmath>
#include <cstring>

static const char* TAG = "ServoController";

ServoController::ServoController()
    : i2c_bus_(nullptr), 
      pca9685_handle_(nullptr), 
      initialized_(false),
      last_command_(""),  // 【新增】初始化上次命令为空
      last_command_time_(std::chrono::steady_clock::now()) {}  // 【新增】初始化时间为当前时间

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

    // 【关键】1. 进入睡眠模式以设置频率（与Arduino代码完全一致）
    err = WriteRegister(kMode1, 0x10); // SLEEP=1
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 enter sleep failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "✅ PCA9685进入睡眠模式");
    
    // 2. 设置预分频器 (Prescale) for 50Hz
    int prescale = static_cast<int>(round((kOscillatorHz / (4096.0 * kPwmFreqHz)) - 1));
    ESP_LOGI(TAG, "Setting Prescale to %d for %d Hz", prescale, kPwmFreqHz);
    err = WriteRegister(kPrescale, static_cast<uint8_t>(prescale));
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 3. 唤醒芯片（退出睡眠模式）
    err = WriteRegister(kMode1, 0x00); // SLEEP=0
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "✅ PCA9685唤醒成功");
    
    // 【关键】4. 设置Mode2为推挽输出模式（与Arduino代码完全一致）
    err = WriteRegister(kMode2, 0x04); // OUTDRV=1 (推挽输出)
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 Mode2 set failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "✅ PCA9685 Mode2设置为推挽输出模式 (0x04)");
    
    // 5. 清零所有通道的PWM值
    for (int ch = 0; ch < 16; ++ch) {
        WritePwm(ch, 0, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    initialized_ = true;
    
    // 🔍 I2C通信验证测试
    ESP_LOGI("SERVO_DEBUG", "🔍 PCA9685初始化完成，开始验证...");
    
    // 读取Mode1和Mode2寄存器验证配置
    uint8_t mode1_value = 0, mode2_value = 0;
    
    // 先写入要读取的寄存器地址
    uint8_t reg_addr = kMode1;
    esp_err_t read_err = i2c_master_transmit_receive(pca9685_handle_, &reg_addr, 1, &mode1_value, 1, 100);
    if (read_err == ESP_OK) {
        ESP_LOGI("SERVO_DEBUG", "✅ Mode1=0x%02X (应为0x00)", mode1_value);
    } else {
        ESP_LOGE("SERVO_DEBUG", "❌ 读取Mode1失败: %s", esp_err_to_name(read_err));
    }
    
    reg_addr = kMode2;
    read_err = i2c_master_transmit_receive(pca9685_handle_, &reg_addr, 1, &mode2_value, 1, 100);
    if (read_err == ESP_OK) {
        ESP_LOGI("SERVO_DEBUG", "✅ Mode2=0x%02X (应为0x04推挽输出)", mode2_value);
        if (mode2_value != 0x04) {
            ESP_LOGW("SERVO_DEBUG", "⚠️ Mode2值不正确！这是舵机不动作的主要原因");
        }
    } else {
        ESP_LOGE("SERVO_DEBUG", "❌ 读取Mode2失败: %s", esp_err_to_name(read_err));
    }
    
    ESP_LOGI(TAG, "Servo controller initialized successfully");
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

    // 【修改】不再在初始化时自动设置舵机角度，等待语音控制
    // if (initialized_) {
    //     SetAngle(config.channel, config.default_angle);
    // }

    ESP_LOGI(TAG, "Added servo: channel=%d, name=%s, default_angle=%d (待语音控制)", config.channel,
             config.name.c_str(), config.default_angle);
}

esp_err_t ServoController::SetAngle(uint8_t channel, int angle) {
    if (!initialized_ || channel >= servos_.size()) {
        ESP_LOGE("SERVO_DEBUG", "❌ SetAngle失败: initialized=%d, channel=%d, servos_size=%zu", 
                 initialized_, channel, servos_.size());
        return ESP_ERR_INVALID_STATE;
    }

    const auto& config = servos_[channel];
    int pwm_value = AngleToPwmValue(config, angle);
    
    ESP_LOGI("SERVO_DEBUG", "📊 SetAngle计算结果: channel=%d, angle=%d, pwm_value=%d", 
             channel, angle, pwm_value);

    esp_err_t err = WritePwm(channel, 0, pwm_value);
    if (err == ESP_OK) {
        current_angles_[channel] = angle;
        ESP_LOGI("SERVO_DEBUG", "✅ PWM写入成功: ON=0, OFF=%d", pwm_value);
    } else {
        ESP_LOGE("SERVO_DEBUG", "❌ PWM写入失败! 错误码: %s (0x%X)", esp_err_to_name(err), err);
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

bool ServoController::ProcessVoiceCommand(const std::string& text) {
    std::string servo_name;
    int angle;

    // 👇 新增调试日志，必加！
    ESP_LOGI("SERVO_DEBUG", "收到语音指令：%s", text.c_str());

    // 【新增】指令去重和时间间隔检查
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_command_time_).count();
    
    // 如果命令相同且时间间隔小于阈值，忽略该命令
    if (text == last_command_ && elapsed_ms < kMinCommandIntervalMs) {
        ESP_LOGW("SERVO_DEBUG", "⚠️ 忽略重复命令（%dms内）", static_cast<int>(elapsed_ms));
        return false;
    }

    if (!ParseVoiceCommand(text, servo_name, angle)) {
        // 👇 解析失败时打印，立刻就能看到问题
        ESP_LOGW("SERVO_DEBUG", "❌ 指令解析失败！servo_name: %s, angle: %d", servo_name.c_str(),
                 angle); return false;
    }

    ESP_LOGI("SERVO_DEBUG", "✅ 解析成功！舵机名：%s，目标角度：%d", servo_name.c_str(), angle);

    esp_err_t err;
    
    // 【新增】处理停止命令
    if (angle == -1) {
        ESP_LOGI("SERVO_DEBUG", "🛑 执行停止命令，关闭PWM输出");
        
        uint8_t target_channel;
        if (!servo_name.empty()) {
            int index = FindServoByName(servo_name);
            if (index < 0) {
                ESP_LOGE("SERVO_DEBUG", "❌ 未找到名为 '%s' 的舵机", servo_name.c_str());
                return false;
            }
            target_channel = servos_[index].channel;
        } else {
            if (servos_.empty()) {
                ESP_LOGE("SERVO_DEBUG", "❌ 错误：没有注册任何舵机配置！");
                return false;
            }
            target_channel = servos_[0].channel;
        }
        
        // 将PWM设置为0，停止舵机输出
        err = WritePwm(target_channel, 0, 0);
        if (err != ESP_OK) {
            ESP_LOGE("SERVO_DEBUG", "❌ 停止舵机失败！错误码: %s (0x%X)", esp_err_to_name(err), err);
            return false;
        }
        
        ESP_LOGI("SERVO_DEBUG", "✅ 舵机已停止！通道: %d", target_channel);
        
        if (voice_callback_) {
            voice_callback_(text, 0);
        }
        
        return true;
    }
    
    // 正常的角度控制逻辑
    if (!servo_name.empty()) {
        ESP_LOGI("SERVO_DEBUG", "尝试按名称设置舵机角度: %s -> %d度", servo_name.c_str(), angle);
        err = SetAngleByName(servo_name, angle);
    } else {
        // 如果没有指定舵机名称，默认控制第一个舵机
        if (!servos_.empty()) {
            ESP_LOGI("SERVO_DEBUG", "使用默认舵机通道 %d，设置角度 %d度", servos_[0].channel, angle);
            err = SetAngle(servos_[0].channel, angle);
        } else {
            ESP_LOGE("SERVO_DEBUG", "❌ 错误：没有注册任何舵机配置！");
            return false;
        }
    }

    if (err != ESP_OK) {
        ESP_LOGE("SERVO_DEBUG", "❌ 舵机控制失败！错误码: %s (0x%X)", esp_err_to_name(err), err);
        return false;
    }

    ESP_LOGI("SERVO_DEBUG", "✅ 舵机控制成功！");

    // 【新增】更新去重相关变量
    last_command_ = text;
    last_command_time_ = std::chrono::steady_clock::now();
    ESP_LOGD("SERVO_DEBUG", "📝 已记录命令和时间戳，下次相同命令需在%dms后执行", kMinCommandIntervalMs);

    if (voice_callback_) {
        voice_callback_(text, angle);
    }

    return true;
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
        ESP_LOGE("SERVO_DEBUG", "❌ WritePwm失败: pca9685_handle_为空");
        return ESP_ERR_INVALID_STATE;
    }

    // 【关键修复】按照Arduino方式，分4次单独写入寄存器
    int base_reg = kPwmBase + 4 * channel;
    
    ESP_LOGI("SERVO_DEBUG", "📝 开始写入PWM寄存器: channel=%d, base_reg=0x%02X, ON=%d, OFF=%d", 
             channel, base_reg, on, off);
    
    esp_err_t err;
    
    // 写入ON_L
    err = WriteRegister(base_reg, static_cast<uint8_t>(on & 0xFF));
    if (err != ESP_OK) {
        ESP_LOGE("SERVO_DEBUG", "❌ 写入ON_L失败: reg=0x%02X, err=%s", base_reg, esp_err_to_name(err));
        return err;
    }
    ESP_LOGD("SERVO_DEBUG", "✅ ON_L写入成功: 0x%02X", on & 0xFF);
    
    // 写入ON_H
    err = WriteRegister(base_reg + 1, static_cast<uint8_t>((on >> 8) & 0x0F));
    if (err != ESP_OK) {
        ESP_LOGE("SERVO_DEBUG", "❌ 写入ON_H失败: reg=0x%02X, err=%s", base_reg + 1, esp_err_to_name(err));
        return err;
    }
    ESP_LOGD("SERVO_DEBUG", "✅ ON_H写入成功: 0x%02X", (on >> 8) & 0x0F);
    
    // 写入OFF_L
    err = WriteRegister(base_reg + 2, static_cast<uint8_t>(off & 0xFF));
    if (err != ESP_OK) {
        ESP_LOGE("SERVO_DEBUG", "❌ 写入OFF_L失败: reg=0x%02X, err=%s", base_reg + 2, esp_err_to_name(err));
        return err;
    }
    ESP_LOGD("SERVO_DEBUG", "✅ OFF_L写入成功: 0x%02X", off & 0xFF);
    
    // 写入OFF_H
    err = WriteRegister(base_reg + 3, static_cast<uint8_t>((off >> 8) & 0x0F));
    if (err != ESP_OK) {
        ESP_LOGE("SERVO_DEBUG", "❌ 写入OFF_H失败: reg=0x%02X, err=%s", base_reg + 3, esp_err_to_name(err));
        return err;
    }
    ESP_LOGD("SERVO_DEBUG", "✅ OFF_H写入成功: 0x%02X", (off >> 8) & 0x0F);
    
    ESP_LOGI("SERVO_DEBUG", "✅ PWM寄存器全部写入成功");
    return ESP_OK;
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
        normalized.find("舵积") == std::string::npos &&
        normalized.find("舵汁") == std::string::npos &&
        normalized.find("剁机") == std::string::npos &&
        normalized.find("剁汁") == std::string::npos &&
        normalized.find("多机") == std::string::npos &&
        normalized.find("舵鸡") == std::string::npos &&
        normalized.find("舵记") == std::string::npos &&
        normalized.find("舵极") == std::string::npos &&
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
        // 【新增】检查是否为停止命令
        if (normalized.find("停止") != std::string::npos ||
            normalized.find("停") != std::string::npos ||
            normalized.find("stop") != std::string::npos ||
            normalized.find("halt") != std::string::npos) {
            angle = -1;  // 使用-1作为停止标志
            ESP_LOGI("SERVO_DEBUG", "🛑 检测到停止命令");
        }
        else if (normalized.find("归位") != std::string::npos ||
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
