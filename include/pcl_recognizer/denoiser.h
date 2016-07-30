#ifndef PCL_RECOGNIZER_DENOISER_H
#define PCL_RECOGNIZER_DENOISER_H

#include <pcl_recognizer/preprocessed_data.h>
#include <pcl_recognizer/reconfigurable.h>
#include <pcl_recognizer/DenoiseConfig.h>

class Denoiser : Reconfigurable<pcl_recognizer::DenoiseConfig>
{
public:
  Denoiser(std::string name = "Denoise") : Reconfigurable(name) {};

  void cleanOnInput(PreprocessedData& data);
  void cleanWithNormals(PreprocessedData& data);

  void passThrough(PreprocessedData& data);
  void refineNormals(PreprocessedData& data);
  void removeEdges(PreprocessedData& data);
  void removeGhostPoints(PreprocessedData& data);
  void removeNaNs(PreprocessedData& data);
  void removeOutliers(PreprocessedData& data);
private:
};


#endif //PCL_RECOGNIZER_DENOISER_H
