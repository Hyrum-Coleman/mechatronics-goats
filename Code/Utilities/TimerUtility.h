/*
 * TimerUtility.h
 * 
 * This header file contains both the declaration and the implementation of the Timer template class.
 * The implementation is included directly in the header file due to the nature of C++ templates.
 * 
 * Reasons for including the implementation in the header file:
 * 
 * 1. Template Instantiation: In C++, templates are instantiated at compile time. This requires 
 *    the full implementation to be visible to the compiler whenever a template is used. If the 
 *    implementation were separated into a .cpp file, the compiler would not be able to instantiate 
 *    the template for types that are only known at the point of use in other translation units.
 * 
 * 2. Arduino Compatibility: The Arduino build process compiles .cpp files separately, and it does 
 *    not automatically instantiate templates for all possible types. This can lead to linker errors 
 *    if the implementation is not included where the template is used.
 * 
 * 3. Flexibility: By including the implementation in the header, we allow users to instantiate the 
 *    Timer class with any callback type without needing to explicitly instantiate the template for 
 *    each type in the .cpp file. This approach enhances usability and flexibility, especially important 
 *    for a variety of Arduino projects with diverse requirements.
 * 
 * As a result, to ensure compatibility and ease of use across different Arduino boards and projects, 
 * the implementation of the Timer class is provided within this header file.
 */

#ifndef TIMER_UTILITY_H
#define TIMER_UTILITY_H

#include <Arduino.h>

template<typename T>
class Timer {
public:
    Timer(unsigned long interval, T callback) : interval(interval), lastTime(0), callback(callback) {}

    void check() {
        unsigned long currentTime = millis();
        if (currentTime - lastTime >= interval) {
            lastTime = currentTime;
            callback(); // Call the callable object
        }
    }

    void setInterval(unsigned long newInterval) {
        interval = newInterval;
    }

private:
    unsigned long interval;
    unsigned long lastTime;
    T callback;  // Template type for the callback
};

#endif // TIMER_UTILITY_H
