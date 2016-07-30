#ifndef PCL_RECOGNIZE_UTILS_H
#define PCL_RECOGNIZE_UTILS_H

#include <chrono>
#include <string>

namespace Timer
{
using hrc = std::chrono::high_resolution_clock;
using tstamp = decltype(hrc::now());

void start();
void end(std::string name);

class Scoped
{
public:
  Scoped(std::string name = "Timer");
  ~Scoped();
private:
  std::string name_;
  tstamp start_;
};
};


#endif //PCL_RECOGNIZE_UTILS_H
