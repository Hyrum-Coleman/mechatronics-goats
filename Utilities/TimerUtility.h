// TimerUtility.h
#ifndef TIMER_UTILITY_H
#define TIMER_UTILITY_H

#include <Arduino.h>

typedef void (*TimerCallback)();  // Define a type for the callback function pointer

class Timer {
public:
    Timer(unsigned long interval, TimerCallback callback);

    void check();
    void setInterval(unsigned long newInterval);

private:
    unsigned long interval;
    unsigned long lastTime;
    TimerCallback callback;  // Use function pointer for the callback
};

#endif // TIMER_UTILITY_H
