#ifndef MCT8316_H
#define MCT8316_H
#include <stdint.h>
class MCT8316
{
public:
    enum class HallSensorType
    {
        DIGITAL,
        ANALOG
    };
    enum class HallSensor
    {
        HALL_A,
        HALL_B,
        HALL_C
    };
private:
    struct __attribute__((packed)) ICStatusRegister
    {
        uint8_t fault: 1;
        uint8_t overtemperature: 1;
        uint8_t overvoltage: 1;
        uint8_t npor: 1;
        uint8_t overcurrent: 1;
        uint8_t spi_fault: 1;
        uint8_t buck_fault: 1;
        uint8_t motor_lock: 1;
    };
    struct __attribute__((packed)) StatusRegister1
    {
        uint8_t overcurrent_la: 1;
        uint8_t overcurrent_ha: 1;
        uint8_t overcurrent_lb: 1;
        uint8_t overcurrent_hb: 1;
        uint8_t overcurrent_lc: 1;
        uint8_t overcurrent_hc: 1;
        uint8_t overtemperature_shutdown: 1;
        uint8_t overtemperature_warning: 1;
    };
    struct __attribute__((packed)) StatusRegister2
    {
        uint8_t spi_addr_error: 1;
        uint8_t spi_clock_error: 1;
        uint8_t spi_parity_error: 1;
        uint8_t charge_pump_undervoltage: 1;
        uint8_t buck_undervoltage: 1;
        uint8_t buck_overcurrent: 1;
        uint8_t otp_error: 1;
        uint8_t reserved: 1;
    };
    struct __attribute__((packed)) ControlRegister1
    {
        uint8_t reg_lock: 3;
        uint8_t reserved: 5;
    };
    struct __attribute__((packed)) ControlRegister2A
    {
        uint8_t clear_fault: 1;
        uint8_t pwm_mode: 2;
        uint8_t slew_mode: 2;
        uint8_t sdo_mode: 1;
        uint8_t reserved: 2;
    };
    struct __attribute__((packed)) ControlRegister3
    {
        uint8_t overtemperature_warning_report: 1;
        uint8_t reserved0: 1;
        uint8_t overvoltage_en: 1;
        uint8_t overvoltage_select: 1;
        uint8_t pwm_duty_cycle: 1;
        uint8_t reserved1: 3;
    };
    struct __attribute__((packed)) ControlRegister4
    {
        uint8_t overcurrent_fault_select: 2;
        uint8_t overcurrent_level_select: 1;
        uint8_t overcurrent_retry_select: 1;
        uint8_t overcurrent_deglitch_time: 2;
        uint8_t overcurrent_pwm_cycle_clear_en: 1;
        uint8_t drive_off: 1;
    };
    struct __attribute__((packed)) ControlRegister5
    {
        uint8_t current_sense_gain: 2;
        uint8_t sync_rectification_en: 1;
        uint8_t async_rectification_en: 1;
        uint8_t reserved0: 2;
        uint8_t current_limit_recirculation_mode: 1;
        uint8_t reserved1: 1;
    };
    struct __attribute__((packed)) ControlRegister6
    {
        uint8_t buck_disable: 1;
        uint8_t buck_select: 2;
        uint8_t buck_current_limit: 1;
        uint8_t buck_power_seq_disable: 1;
        uint8_t reserved: 3;
    };
    struct __attribute__((packed)) ControlRegister7
    {
        uint8_t dir: 1;
        uint8_t brake: 1;
        uint8_t coast: 1;
        uint8_t brake_mode: 1;
        uint8_t hall_hys: 1;
        uint8_t reserved: 3;
    };
    struct __attribute__((packed)) ControlRegister8
    {
        uint8_t motor_lock_mode: 2;
        uint8_t motor_lock_detect_time: 2;
        uint8_t motor_lock_retry_time: 1;
        uint8_t reserved: 1;
        uint8_t fgout_select: 2;
    };
    struct __attribute__((packed)) ControlRegister9
    {
        uint8_t phase_advance: 3;
        uint8_t reserved: 5;
    };
    struct __attribute__((packed)) ControlRegister10
    {
        uint8_t delay_target: 3;
        uint8_t delay_compensation_en: 1;
        uint8_t reserved: 4;
    };
    int pwmPin, csPin, brakePin, hallAPin, hallBPin, hallCPin, pwmWrap;
    bool spiInitialized = false, pwmInitialized = false, brakeInitialized = false, hallSensorsInitialized = false,
        firstSensorTick = true;
    double output = 0.0, rate = 0.0;
    HallSensor lastHallSensor;
    int sensorCounts = 0, lastSensorCounts = 0;
    void initializeSPI(int sclkPin, int txPin, int rxPin, int csPin);
    void initializePWM(int pwmPin, int clockFrequency, int pwmFrequency, HallSensorType type);
    void initializeBrake(int brakePin);
    void initializeHallSensors(int hallAPin, int hallBPin, int hallCPin, HallSensorType type);
    void writeRegister(uint8_t offset, const void* reg);
    void readRegister(uint8_t offset, void* reg);
public:
    static MCT8316* instance;
    MCT8316() = default;
    MCT8316(int sclkPin, int txPin, int rxPin, int csPin, int pwmPin, int brakePin, int hallAPin,
        int hallBPin, int hallCPin, int clockFrequency, int pwmFrequency, HallSensorType type);
    void pwmCallback();
    void digitalHallACallback();
    void digitalHallBCallback();
    void digitalHallCCallback();
    void analogHallCallback();
    void rateCallback();
    void setBrake(bool brakeEnabled);
    void setOutput(double output);
    double getRotations();
    double getRate();
    bool hasFault();
};
#endif