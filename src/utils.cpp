#include "pcl_recognizer/utils.h"

#include <chrono>
#include <iostream>

namespace Timer
{
void duration(bool print, std::string name = "")
{
  using hrc = std::chrono::high_resolution_clock;
  static auto last = hrc::now();
  if (print)
    std::cout << name <<
    " duration: " <<
    std::chrono::duration_cast<std::chrono::duration<double>>(hrc::now() - last).count() <<
    std::endl;
  last = hrc::now();
}

void start()
{
  duration(false);
}

void end(std::string name)
{
  duration(true, name);
}
}
