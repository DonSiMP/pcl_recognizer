#include "pcl_recognizer/utils.h"

#include <chrono>
#include <iostream>

void Timer::start()
{
  duration(false);
}

void Timer::end()
{
  duration(true);
}

void Timer::duration(bool print)
{
  using hrc = std::chrono::high_resolution_clock;
  static auto last = hrc::now();
  if (print)
    std::cout <<
        "Timer duration: " <<
        std::chrono::duration_cast<std::chrono::nanoseconds>(hrc::now() - last).count() <<
        std::endl;
  last = hrc::now();
}
