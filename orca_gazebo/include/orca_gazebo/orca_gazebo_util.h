#ifndef ORCA_GAZEBO_UTIL_H
#define ORCA_GAZEBO_UTIL_H

#include <gazebo/gazebo.hh>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

namespace orca_gazebo {

double gaussianKernel(double mean, double stddev)
{
  // Get 2 random numbers from a uniform distribution
  static unsigned int seed = 0;
  double uniform1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double uniform2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // Use a Box-Muller transform to generate a value from a Gaussian distribution with mean=0 stddev=1
  double gaussian = sqrt(-2.0 * ::log(uniform1)) * cos(2.0*M_PI * uniform2);

  // Scale
  return stddev * gaussian + mean;
}

// Assume x, y and z are independent, e.g., MEMS accelerometers or gyros
void addNoise(const double stddev, ignition::math::Vector3d &v)
{
  v.X() = gaussianKernel(v.X(), stddev);
  v.Y() = gaussianKernel(v.Y(), stddev);
  v.Z() = gaussianKernel(v.Z(), stddev);
}

// Assume r, p and y are independent, e.g., MEMS magnetometers
void addNoise(const double stddev, ignition::math::Quaterniond &q)
{
  ignition::math::Vector3d v = q.Euler();
  addNoise(stddev, v);
  q.Euler(v);
}

void ignition2msg(const ignition::math::Vector3d &i, geometry_msgs::Vector3 &m)
{
  m.x = i.X();
  m.y = i.Y();
  m.z = i.Z();
}

void ignition2msg(const ignition::math::Quaterniond &i, geometry_msgs::Quaternion &m)
{
  m.x = i.X();
  m.y = i.Y();
  m.z = i.Z();
  m.w = i.W();
}

} // namespace orca_gazebo

#endif // ORCA_GAZEBO_UTIL_H
