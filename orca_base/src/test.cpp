#include "orca_base/orca_model.h"

void test4to6()
{
  std::array<double, 16> four;
  boost::array<double, 36> six;

  for (int i = 0; i < four.size(); ++i)
  {
    four[i] = i;
  }

  for (int i = 0; i < four.size(); ++i)
  {
    std::cout << "four[" << i << "]: " << four[i] << std::endl;
  }

  orca_base::OrcaOdometry::covar4to6(four, six);

  for (int i = 0; i < six.size(); ++i)
  {
    std::cout << "six[" << i << "]: " << six[i] << std::endl;
  }
}

int main(int argc, char** argv)
{
  test4to6();
  return 0;
}
