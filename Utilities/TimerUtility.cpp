#include "TimerUtility.h"

Timer::Timer(unsigned long interval, TimerCallback callback)
    : interval(interval), lastTime(0), callback(callback) {}

void Timer::check() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
        lastTime = currentTime;
        callback(); // Call the function pointer
    }
}

void Timer::setInterval(unsigned long newInterval) {
    interval = newInterval;
}
