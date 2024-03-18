// IIRFilter.h

#ifndef IIRFilter_h
#define IIRFilter_h

#include "Arduino.h"

class IIRFilter {
public:
  IIRFilter(float alpha) {
    _alpha = alpha;
    _lastOutput = 0.0;
  }

  float process(float input) {
    float output = _alpha * input +
                   (1 - _alpha) * _lastOutput; // Apply the filter equation
    _lastOutput = output;                      // update last output
    return output;
  }

private:
  float _alpha;
  float _lastOutput; // (y[n-1])
};

#endif
