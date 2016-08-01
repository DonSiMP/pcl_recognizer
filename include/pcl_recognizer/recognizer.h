#ifndef PCL_RECOGNIZE_RECOGNIZER_H
#define PCL_RECOGNIZE_RECOGNIZER_H

#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/reconfigurable.h>
#include <pcl_recognizer/GroupingConfig.h>

#include <pcl/correspondence.h>

class Recognizer : Reconfigurable<pcl_recognizer::GroupingConfig>
{
public:
  Recognizer() : Reconfigurable("recognizer_grouping") {};

  void setModel(PreprocessedData& model) { model_ = model; }
  int recognize(const PreprocessedData& scene, Pose& pose);
  bool finished() const { return done; };

  PreprocessedData getModel() { return model_; }
  PreprocessedData getScene() { return scene_; }
  pcl::CorrespondencesPtr getCorrs() { return model_scene_corrs_; };
  PoseVector& getPoses() { return foundInstances_; };
  std::vector<pcl::Correspondences>& getClusters() { return correspondence_clusters_; };
private:
  bool done = false;

  PreprocessedData model_, scene_;
  std::vector<pcl::Correspondences> correspondence_clusters_;
  pcl::CorrespondencesPtr model_scene_corrs_;
  PoseVector foundInstances_;

  void findCorrespondences();
  void clusterize();
  void clusterizeHough();
  void clusterizeGC();
  void refineAbsoluteOrientation();
  void refineICP();
  void verifyHypotheses();
};


#endif //PCL_RECOGNIZE_RECOGNIZER_H
