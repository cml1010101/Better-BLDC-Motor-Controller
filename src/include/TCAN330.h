#ifndef TCAN330_H
#define TCAN330_H
#include <can2040.h>
#include <functional>
typedef struct can2040 can2040_t;
typedef struct can2040_msg can2040_msg_t;
class TCAN330
{
private:
    can2040_t canbus;
    std::function<void(can2040_msg_t*)> rxFunc;
public:
    static TCAN330* instance;
    TCAN330() = default;
    TCAN330(int rxPin, int txPin, int sPin, int shdnPin, std::function<void(can2040_msg_t*)> rxFunc, int clockFrequency);
    void rxCallback(can2040_msg_t* msg);
    void sendMessage(can2040_msg_t* msg);
    can2040_t* getCANBus();
};
#endif