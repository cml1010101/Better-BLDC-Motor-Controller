#include <MCT8316.h>
#include "pico/stdio.h"
MCT8316 mct;
int main()
{
    stdio_init_all();
    mct = MCT8316(2, 0, 3, 1, 5, 4, 15, 16, 17, 12e6, 20e3, MCT8316::HallSensorType::DIGITAL);
}