#ifndef PCL_RECOGNIZE_RECOGNIZER_H
#define PCL_RECOGNIZE_RECOGNIZER_H

#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/GroupingConfig.h>

#include <pcl/correspondence.h>

class Recognizer
{
public:
  void setModel(PreprocessedData& model) { model_ = model; }
  int recognize(const PreprocessedData& scene, Pose& pose);

  PreprocessedData getModel() { return model_; }
  PreprocessedData getScene() { return scene_; }
  pcl::CorrespondencesPtr getCorrs() { return model_scene_corrs; };
private:

  PreprocessedData model_, scene_;
  pcl::CorrespondencesPtr model_scene_corrs;
  PoseVector foundInstances;
  //Algorithm params
  bool use_hough_ = false;

  float cg_size_ = 0.01f;
  float cg_thresh_ = 5.0f;

  void cfg_cb(pcl_recognizer::GroupingConfig& config, uint32_t level);

  void findCorrespondences();
  void clusterize();
  void clusterizeWithGC();
};


#endif //PCL_RECOGNIZE_RECOGNIZER_H
