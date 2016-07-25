#ifndef PCL_RECOGNIZE_RECOGNIZER_H
#define PCL_RECOGNIZE_RECOGNIZER_H

#include <dynamic_reconfigure/server.h>
#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/GroupingConfig.h>

#include <pcl/correspondence.h>

class Recognizer
{
public:
  Recognizer();

  void setModel(PreprocessedData& model) { model_ = model; }
  int recognize(const PreprocessedData& scene, Pose& pose);

  PreprocessedData getModel() { return model_; }
  PreprocessedData getScene() { return scene_; }
  pcl::CorrespondencesPtr getCorrs() { return model_scene_corrs; };
  std::vector<pcl::Correspondences>& getClusters() { return correspondence_clusters_; };
private:

  PreprocessedData model_, scene_;
  std::vector<pcl::Correspondences> correspondence_clusters_;
  pcl::CorrespondencesPtr model_scene_corrs;
  PoseVector foundInstances;
  //Algorithm params
  bool use_hough_ = false;
  float cg_size_ = 0.01f;
  float cg_thresh_ = 5.0f;
  float corr_dist_ = 0.25f;

  void grouping_cb(pcl_recognizer::GroupingConfig& config, uint32_t level);
  dynamic_reconfigure::Server<pcl_recognizer::GroupingConfig> grouping_srv_;

  void findCorrespondences();
  void clusterize();
  void clusterizeHough();
  void clusterizeGC();
};


#endif //PCL_RECOGNIZE_RECOGNIZER_H
