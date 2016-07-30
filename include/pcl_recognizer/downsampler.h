#ifndef PCL_RECOGNIZER_DOWNSAMPLER_H
#define PCL_RECOGNIZER_DOWNSAMPLER_H

#include <pcl_recognizer/preprocessed_data.h>
#include <pcl_recognizer/reconfigurable.h>
#include <pcl_recognizer/KeypointConfig.h>

class Downsampler : Reconfigurable<pcl_recognizer::KeypointConfig>
{
public:

  enum class Method
  {
    None,
    Uniform,
    Voxel,
    ISS,
    Sift,
    H3DHarris,
    H3DTomasi,
    H3DNoble,
    H3DLowe,
    H3DCurvature
  };

  Downsampler(std::string name = "Downsampler") : Reconfigurable(name) {};

  void computeKeypoints(PreprocessedData& data);

private:
  void downsampleUniform(PreprocessedData& data);
  void downsampleVoxel(PreprocessedData& data);
  void downsampleISS(PreprocessedData& data);
  void downsampleHarris(PreprocessedData& data);
  void downsampleSIFT(PreprocessedData& data);
};


#endif //PCL_RECOGNIZER_DOWNSAMPLER_H
