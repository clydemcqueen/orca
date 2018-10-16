#include "orca_gazebo/orca_gazebo_util.h"

void testGaussianKernel()
{
  // Anything outside of 4 stddev on either side is an outlier, and go in the first and last buckets

  constexpr int NUM_BUCKETS = 10;
  constexpr double MEAN = NUM_BUCKETS / 2;
  constexpr double STDDEV = 1;
  constexpr double BUCKET_WIDTH = STDDEV * 8 / (NUM_BUCKETS - 2);

  std::vector<int>h(NUM_BUCKETS);

  for (int s = 0; s < 10000; ++s)
  {
    double m = orca_gazebo::gaussianKernel(MEAN, STDDEV);
    int i = static_cast<int>(m / BUCKET_WIDTH);
    if (i < 0) i = 0;
    if (i >= NUM_BUCKETS) i = NUM_BUCKETS - 1;
    h[i]++;
  }

  for (int i = 0; i < h.size(); ++i)
  {
    std::cout << "Bucket " << i << ", " << h[i] << std::endl;
  }
}

int main(int argc, char** argv)
{
  testGaussianKernel();
  return 0;
}
