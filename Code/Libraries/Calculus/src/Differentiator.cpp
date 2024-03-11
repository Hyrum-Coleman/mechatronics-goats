#include <Arduino.h>
#include "Differentiator.h"

Differentiator::Differentiator() : m_timeCache(Cache(3)), m_inputCache(Cache(3))
{
}

double Differentiator::step(double t, double y)
{
  // Calculate average timestep
  double* tv = m_timeCache.step(t);
  double tStep1 = *tv - *(tv + 1);
  double tStep2 = *(tv + 1) - *(tv + 2);
  double stepAvg = tStep2 + (tStep1 - tStep2) / 2;  // Difference avg

  double* yv = m_inputCache.step(y);
  double& y0 = *yv;
  double& ym1 = *(yv + 1);
  double& ym2 = *(yv + 2);
  m_quad.solve(ym2, ym1, y0);
  return m_quad.deriv() / stepAvg;
}
