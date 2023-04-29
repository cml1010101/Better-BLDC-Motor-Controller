#include <MCT8316.h>
#include <TCAN330.h>
#include "pico/stdio.h"
#include "hardware/xosc.h"
#include "hardware/flash.h"
MCT8316 mct;
TCAN330 can;
double setpoint;
enum class State
{
    ESTOP,
    ENABLED,
    DISABLED
} state;
enum class SetpointType
{
    PERCENT,
    VELOCITY,
    POSITION
} setpointType;
void recieveMessage(can2040_msg_t* can)
{
}
int main()
{
    stdio_init_all();
    xosc_init();
    mct = MCT8316(2, 0, 3, 1, 5, 4, 15, 16, 17, 12e6, 20e3, MCT8316::HallSensorType::DIGITAL);
    can = TCAN330(12, 11, 14, 13, recieveMessage, 12e6);
    while (true)
    {
    }
}