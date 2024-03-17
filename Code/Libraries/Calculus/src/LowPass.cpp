#include <Arduino.h>
#include "LowPass.h"

LowPass::LowPass(double tau, double yInitial) :
  m_integ(Integrator(yInitial)), m_gain(1.0 / tau)
{
}

double LowPass::step(double t, double u)
{
  m_y = m_integ.step(t, m_gain * (u - m_y));
  return m_y;
}

