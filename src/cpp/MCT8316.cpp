#include <MCT8316.h>
#include <cmath>
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
void MCT8316::initializeSPI(int sclkPin, int txPin, int rxPin, int csPin)
{
    this->csPin = csPin;
    gpio_set_function(sclkPin, GPIO_FUNC_SPI);
    gpio_set_function(txPin, GPIO_FUNC_SPI);
    gpio_set_function(rxPin, GPIO_FUNC_SPI);
    spi_init(spi0, 5e6);
    gpio_init(csPin);
    gpio_set_dir(csPin, GPIO_OUT);
    gpio_put(csPin, true);
    spiInitialized = true;
}
void pwmCallback()
{
    assert(MCT8316::instance != NULL);
    MCT8316::instance->pwmCallback();
}
void MCT8316::initializePWM(int pwmPin, int clockFrequency, int pwmFrequency, HallSensorType type)
{
    assert(spiInitialized);
    assert(pwmFrequency == 20e3 || pwmFrequency == 40e3);
    pwmWrap = clockFrequency / pwmFrequency;
    this->pwmPin = pwmPin;
    ControlRegister2A ctrl2a;
    readRegister(4, &ctrl2a);
    ctrl2a.pwm_mode = type == HallSensorType::ANALOG ? 0 : 1;
    writeRegister(4, &ctrl2a);
    ControlRegister3 ctrl3;
    readRegister(5, &ctrl3);
    ctrl3.pwm_duty_cycle = pwmFrequency == 40e3;
    writeRegister(5, &ctrl3);
    gpio_set_function(pwmPin, GPIO_FUNC_PWM);
    int pwmSlice = pwm_gpio_to_slice_num(pwmPin);
    pwm_clear_irq(pwmSlice);
    pwm_set_irq_enabled(pwmSlice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, ::pwmCallback);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_config motor_pwm_config = pwm_get_default_config();
    pwm_config_set_wrap(&motor_pwm_config, pwmWrap);
    pwm_init(pwmSlice, &motor_pwm_config, true);
    pwmInitialized = true;
}
void MCT8316::initializeBrake(int brakePin)
{
    this->brakePin = brakePin;
    gpio_init(brakePin);
    gpio_set_dir(brakePin, GPIO_OUT);
    gpio_put(brakePin, false);
    brakeInitialized = true;
}
void digitalHallACallback(uint gpio, uint32_t event)
{
    assert(MCT8316::instance != NULL);
    MCT8316::instance->digitalHallACallback();
}
void digitalHallBCallback(uint gpio, uint32_t event)
{
    assert(MCT8316::instance != NULL);
    MCT8316::instance->digitalHallBCallback();
}
void digitalHallCCallback(uint gpio, uint32_t event)
{
    assert(MCT8316::instance != NULL);
    MCT8316::instance->digitalHallCCallback();
}
bool analogHallCallback(repeating_timer_t* timer)
{
    assert(MCT8316::instance != NULL);
    MCT8316::instance->analogHallCallback();
    return true;
}
bool rateCallback(repeating_timer_t* timer)
{
    assert(MCT8316::instance != NULL);
    MCT8316::instance->rateCallback();
    return true;
}
void MCT8316::initializeHallSensors(int hallAPin, int hallBPin, int hallCPin, HallSensorType type)
{
    this->hallAPin = hallAPin;
    this->hallBPin = hallBPin;
    this->hallCPin = hallCPin;
    if (type == HallSensorType::DIGITAL)
    {
        gpio_init(hallAPin);
        gpio_set_dir(hallAPin, GPIO_IN);
        gpio_set_irq_enabled_with_callback(hallAPin, GPIO_IRQ_EDGE_RISE, true, ::digitalHallACallback);
        gpio_init(hallBPin);
        gpio_set_dir(hallBPin, GPIO_IN);
        gpio_set_irq_enabled_with_callback(hallBPin, GPIO_IRQ_EDGE_RISE, true, ::digitalHallBCallback);
        gpio_init(hallCPin);
        gpio_set_dir(hallCPin, GPIO_IN);
        gpio_set_irq_enabled_with_callback(hallCPin, GPIO_IRQ_EDGE_RISE, true, ::digitalHallCCallback);
    }
    else
    {
        adc_gpio_init(hallAPin);
        adc_gpio_init(hallBPin);
        adc_gpio_init(hallCPin);
        repeating_timer_t timer;
        add_repeating_timer_us(10, ::analogHallCallback, NULL, &timer);
    }
    repeating_timer_t timer;
    add_repeating_timer_ms(-1, ::rateCallback, NULL, &timer);
    hallSensorsInitialized = true;
}
int calculateParity(uint16_t dw)
{
    int parity = 0;
    for (size_t i = 0; i < 16; i++, dw >>= 1)
    {
        if (dw & 1) parity = !parity;
    }
    return parity;
}
void MCT8316::writeRegister(uint8_t offset, const void* reg)
{
    uint16_t cmd = ((uint16_t)offset << 9) | (uint16_t)*(uint8_t*)reg;
    cmd |= calculateParity(cmd) << 8;
    gpio_put(csPin, false);
    spi_write_blocking(spi0, (uint8_t*)&cmd, 2);
    gpio_put(csPin, true);
    sleep_us(400);
}
void MCT8316::readRegister(uint8_t offset, void* reg)
{
    uint16_t cmd = ((uint16_t)offset << 9) | 0x8000;
    cmd |= calculateParity(cmd) << 8;
    gpio_put(csPin, false);
    spi_write16_blocking(spi0, &cmd, 2);
    sleep_us(400);
    spi_read16_blocking(spi0, 0, &cmd, 2);
    gpio_put(csPin, true);
    sleep_us(400);
    *(uint8_t*)reg = cmd & 0xFF;
}
MCT8316::MCT8316(int sclkPin, int txPin, int rxPin, int scPin, int pwmPin, int brakePin, int hallAPin,
    int hallBPin, int hallCPin, int clockFrequency, int pwmFrequency, HallSensorType type)
{
    instance = this;
    initializeSPI(sclkPin, txPin, rxPin, csPin);
    initializePWM(pwmPin, clockFrequency, pwmFrequency, type);
    initializeBrake(brakePin);
    initializeHallSensors(hallAPin, hallBPin, hallCPin, type);
}
void MCT8316::pwmCallback()
{
    pwm_set_gpio_level(pwmPin, output * pwmWrap);
}
void MCT8316::digitalHallACallback()
{
    if (firstSensorTick)
    {
        firstSensorTick = false;
    }
    else
    {
        if (lastHallSensor == HallSensor::HALL_B)
        {
            sensorCounts--;
        }
        else if (lastHallSensor == HallSensor::HALL_C)
        {
            sensorCounts++;
        }
    }
    lastHallSensor = HallSensor::HALL_A;
}
void MCT8316::digitalHallBCallback()
{
    if (firstSensorTick)
    {
        firstSensorTick = false;
    }
    else
    {
        if (lastHallSensor == HallSensor::HALL_C)
        {
            sensorCounts--;
        }
        else if (lastHallSensor == HallSensor::HALL_A)
        {
            sensorCounts++;
        }
    }
    lastHallSensor = HallSensor::HALL_B;
}
void MCT8316::digitalHallCCallback()
{
    if (firstSensorTick)
    {
        firstSensorTick = false;
    }
    else
    {
        if (lastHallSensor == HallSensor::HALL_A)
        {
            sensorCounts--;
        }
        else if (lastHallSensor == HallSensor::HALL_B)
        {
            sensorCounts++;
        }
    }
    lastHallSensor = HallSensor::HALL_C;
}
void MCT8316::analogHallCallback()
{
    adc_select_input(hallAPin - 26);
    double hallA = adc_get_selected_input();
    adc_select_input(hallBPin - 26);
    double hallB = adc_get_selected_input();
    adc_select_input(hallCPin - 26);
    double hallC = adc_get_selected_input();
    if (hallA > hallB && hallA > hallC)
    {    
        if (firstSensorTick)
        {
            firstSensorTick = false;
        }
        else
        {
            if (lastHallSensor == HallSensor::HALL_B)
            {
                sensorCounts--;
            }
            else if (lastHallSensor == HallSensor::HALL_C)
            {
                sensorCounts++;
            }
        }
    }
    else if (hallB > hallA && hallB > hallC)
    {    
        if (firstSensorTick)
        {
            firstSensorTick = false;
        }
        else
        {
            if (lastHallSensor == HallSensor::HALL_C)
            {
                sensorCounts--;
            }
            else if (lastHallSensor == HallSensor::HALL_A)
            {
                sensorCounts++;
            }
        }
    }
    else if (hallC > hallB && hallC > hallA)
    {    
        if (firstSensorTick)
        {
            firstSensorTick = false;
        }
        else
        {
            if (lastHallSensor == HallSensor::HALL_A)
            {
                sensorCounts--;
            }
            else if (lastHallSensor == HallSensor::HALL_B)
            {
                sensorCounts++;
            }
        }
    }
}
void MCT8316::rateCallback()
{
    rate = (sensorCounts - lastSensorCounts) * 100;
    lastSensorCounts = sensorCounts;
}
void MCT8316::setBrake(bool brakeEnabled)
{
    gpio_put(brakePin, brakeEnabled);
}
void MCT8316::setOutput(double output)
{
    this->output = output;
}
double MCT8316::getRotations()
{
    return (double)sensorCounts / 3;
}
double MCT8316::getRate()
{
    return rate;
}
bool MCT8316::hasFault()
{
    ICStatusRegister status;
    readRegister(0, &status);
    return status.fault;
}