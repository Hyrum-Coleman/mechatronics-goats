#include <Arduino.h>
#include "PID.h"

PID::PID(double p, double i, double d, double h) :
  m_p(p), m_i(i), m_d(d), m_h(h),
  m_useP(p != 0.0), 
  m_useI(i != 0.0), 
  m_useD(d != 0.0),
  m_integ(),
  m_deriv()
{
}

double PID::step(double t, double u)
{
  double result = 0.0;
  if (m_useP) {
    result += m_p * u;
  }
  if (m_useI) {
    result += m_i * m_integ.step(t, u);
  }
  if (m_useD) {
    result += m_d * m_deriv.step(t, u);
  }
  return result;
}

double PID::run(double t, double u, void (*moveMotors)(double error))
{
  double output = PID::step(t, u);
  double error = u - m_h*output;

  moveMotors(error);
}