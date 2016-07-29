#ifndef PCL_RECOGNIZER_DOWNSAMPLER_H
#define PCL_RECOGNIZER_DOWNSAMPLER_H

#include <dynamic_reconfigure/server.h>
#include <pcl_recognizer/preprocessed_data.h>
#include <pcl_recognizer/KeypointConfig.h>

class Downsampler
{
public:
  Downsampler(std::string name = "Downsampler");

  void computeKeypoints(PreprocessedData& data);

private:
  pcl_recognizer::KeypointConfig keypoint_cfg_;
  dynamic_reconfigure::Server<pcl_recognizer::KeypointConfig> keypoint_srv_;
  void keypoint_cb(pcl_recognizer::KeypointConfig &config, uint32_t level) { keypoint_cfg_ = config; }

  void downsampleUniform(PreprocessedData& data);
  void downsampleISS(PreprocessedData& data);
  void downsampleHarris(PreprocessedData& data);
  void downsampleSIFT(PreprocessedData& data);
};


#endif //PCL_RECOGNIZER_DOWNSAMPLER_H
