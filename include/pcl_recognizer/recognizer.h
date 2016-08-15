#ifndef PCL_RECOGNIZE_RECOGNIZER_H
#define PCL_RECOGNIZE_RECOGNIZER_H

#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/reconfigurable.h>
#include <pcl_recognizer/GroupingConfig.h>

#include <pcl/correspondence.h>

struct RecognizedData
{
  PreprocessedData model_;
  pcl::CorrespondencesPtr correspondences_ = nullptr;
  std::vector<pcl::Correspondences> clusters_;
  PoseVector poses_;
  std::vector<bool> verification_;
};

class Recognizer : Reconfigurable<pcl_recognizer::GroupingConfig>
{
public:
  Recognizer() : Reconfigurable("recognizer_grouping") {};

  void addModel(const PreprocessedData& model);
  int recognize(const PreprocessedData& full_model,
                const PreprocessedData& scene,
                Pose& pose);
  bool finished() const { return done; };
  void reset();

  std::vector<RecognizedData> getRecognitionResults() { return data_; }
  PreprocessedData getScene() { return scene_; }
  PreprocessedData getFullModel() { return full_model_; }
private:
  bool done = false;

  PreprocessedData full_model_;
  PreprocessedData scene_;
  std::vector<RecognizedData> data_;
  std::vector<Cloud::ConstPtr> registered_instances_;
  std::vector<bool> global_hypotheses_mask_;

  void findCorrespondences(RecognizedData& model_data);
  void clusterize(RecognizedData& model_data);
  void clusterizeHough(RecognizedData& model_data);
  void clusterizeGC(RecognizedData& model_data);
  void refineAbsoluteOrientation(RecognizedData& model_data);
  void refineICP(RecognizedData& model_data);
  void verifyHypotheses();
};


#endif //PCL_RECOGNIZE_RECOGNIZER_H
