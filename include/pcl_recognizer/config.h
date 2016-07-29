#ifndef PCL_RECOGNIZER_CONFIG_H
#define PCL_RECOGNIZER_CONFIG_H

#include <dynamic_reconfigure/server.h>
#include <pcl_recognizer/RecognizerConfig.h>

class Config
{
public:
  enum StopAt
  {
    Load,
    Denoise,
    Normals,
    Keypoints,
    Descriptors,
    Grouping,
    All
  };

  static pcl_recognizer::RecognizerConfig& get();
  static bool shouldRun(Config::StopAt step);

private:
  pcl_recognizer::RecognizerConfig cfg;
  dynamic_reconfigure::Server<pcl_recognizer::RecognizerConfig> server;

  Config();
  void params_cb(pcl_recognizer::RecognizerConfig &config, uint32_t level);
};

#endif //PCL_RECOGNIZER_CONFIG_H
