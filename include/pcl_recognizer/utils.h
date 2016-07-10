#ifndef PCL_RECOGNIZE_UTILS_H
#define PCL_RECOGNIZE_UTILS_H

class Timer
{
public:
  static void start();
  static void end();

private:
  static void duration(bool print);
};


#endif //PCL_RECOGNIZE_UTILS_H
