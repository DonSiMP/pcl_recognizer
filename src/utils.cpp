#include "pcl_recognizer/utils.h"

#include <iostream>

namespace Timer
{

void log_duration(const std::string& caller, const tstamp& start, const tstamp& end)
{
  std::cout << caller <<
    " duration: " <<
    std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() <<
    "s" << std::endl;
}

void static_duration(bool print, const std::string& name = "")
{
  static auto last = hrc::now();
  if (print)
    log_duration(name, hrc::now(), last);
  last = hrc::now();
}

void start()
{
  static_duration(false);
}

void end(std::string name)
{
  static_duration(true, name);
}
}

Timer::Scoped::Scoped(std::string name): name_(name)
{
  start_ = hrc::now();
}

Timer::Scoped::~Scoped()
{
  log_duration(name_, start_, hrc::now());
}
