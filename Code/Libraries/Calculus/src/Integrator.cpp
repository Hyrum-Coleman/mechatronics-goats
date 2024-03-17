#include <Arduino.h>
#include "Integrator.h"
//#include "TimeBase.h"

Integrator::Integrator(double yInitial) : 
  m_yValue(yInitial), m_timeCache(Cache(3)), m_inputCache(Cache(3))
{
}

double Integrator::step(double t, double y)
{
  // Calculate average timestep
  double* tv = m_timeCache.step(t);
  double tStep1 = *tv - *(tv + 1);
  double tStep2 = *(tv + 1) - *(tv + 2);
  double stepAvg = tStep2 + (tStep1 - tStep2) / 2;  // Difference avg

  // Setup numerical quadrature
  double* yv = m_inputCache.step(y);
  double& y0 = *yv;
  double& ym1 = *(yv + 1);
  double& ym2 = *(yv + 2);
  m_quad.solve(ym2, ym1, y0);
  m_yValue += m_quad.integ() * stepAvg;
  return m_yValue;
}

void Integrator::reset()
{
  m_yValue = 0.0;
  for (byte i = 0; i < m_inputCache.getNumSamples(); ++i) {
    m_inputCache.step(0.0);
  }

    for (byte i = 0; i < m_timeCache.getNumSamples(); ++i) {
    m_timeCache.step(0.0);
  }
}