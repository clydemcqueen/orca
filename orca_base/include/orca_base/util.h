#ifndef ORCA_UTIL_H
#define ORCA_UTIL_H

#include <cstdint>

namespace orca_base {

template<typename T>
constexpr const T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}

template<typename T>
constexpr const T dead_band(const T v, const T d)
{
  return v < d && v > -d ? 0 : v;
}

template<typename A, typename B>
constexpr const B scale(const A a, const A a_min, const A a_max, const B b_min, const B b_max)
{
  return clamp(static_cast<B>(b_min + static_cast<double>(b_max - b_min) / (a_max - a_min) * (a - a_min)), b_min, b_max);
}

constexpr double norm_angle(double a)
{
  // Move an angle to the region [-M_PI, M_PI]
  while (a < -M_PI)
  {
    a += 2 * M_PI;
  }
  while (a > M_PI)
  {
    a -= 2 * M_PI;
  }

  return a;
}

} // namespace orca_base

#endif // ORCA_UTIL_H
