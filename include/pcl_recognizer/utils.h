#ifndef PCL_RECOGNIZE_UTILS_H
#define PCL_RECOGNIZE_UTILS_H

#include <string>

namespace Timer
{
void start();
void end(std::string name);

class Scoped
{
public:
  Scoped(std::string name = "Timer"): name_(name) { Timer::start(); };
  ~Scoped() { Timer::end(name_); };
private:
  std::string name_;
};
};


#endif //PCL_RECOGNIZE_UTILS_H
