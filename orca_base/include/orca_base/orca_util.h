#ifndef ORCA_UTIL_H
#define ORCA_UTIL_H

template<class T>
constexpr const T dead_band(const T v, const T d)
{
  return v < d && v > -d ? 0 : v;
}

template<class T>
constexpr const T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}

#endif // ORCA_UTIL_H