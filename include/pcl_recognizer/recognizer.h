#ifndef PCL_RECOGNIZE_RECOGNIZER_H
#define PCL_RECOGNIZE_RECOGNIZER_H

#include "pcl_recognizer/preprocessor.h"

#include <pcl/correspondence.h>

class Recognizer
{
public:
  void setModel(PreprocessedData& model) { model_ = model; }
  int recognize(const PreprocessedData& scene, Pose& pose);
  void reconfigure(pcl_recognizer::ParamsConfig& config);

private:
  PreprocessedData model_, scene_;
  pcl::CorrespondencesPtr model_scene_corrs;
  PoseVector foundInstances;

  //Algorithm params
  bool use_hough_ = false;
  float cg_size_ = 0.01f;
  float cg_thresh_ = 5.0f;

  void findCorrespondences();
  void clusterize();
  void clusterizeWithGC();
};


#endif //PCL_RECOGNIZE_RECOGNIZER_H
