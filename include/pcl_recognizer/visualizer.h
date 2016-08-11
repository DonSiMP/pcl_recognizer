#ifndef PCL_RECOGNIZER_VISUALIZER_H
#define PCL_RECOGNIZER_VISUALIZER_H

#include <pcl_recognizer/reconfigurable.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/ViewerConfig.h>


class Visualizer : Reconfigurable<pcl_recognizer::ViewerConfig>
{
public:
  Visualizer(std::string title = "Visualizer");

  void update(PreprocessedData& data);
  void renderRecognition(Recognizer& rec);
  void spin();
private:
  PreprocessedData data_;
  pcl::visualization::PCLVisualizer vis_;
  pcl::visualization::PCLPlotter plot_;
  std::vector<pcl::PointXYZ> picked_points;
  bool new_picks = false;
  bool show_recognition = false;

  pcl_recognizer::ViewerConfig init_status_;

  size_t corr_clusters_count_ = 0;
  size_t registered_instances_count_ = 0;

  using ColorHandler = pcl::visualization::PointCloudColorHandler<Point>;
  void renderRGB(bool cfg, const Cloud::Ptr& data, const ColorHandler& rgb, const std::string& name, bool& status);
  void renderInput();
  void renderKeypoints();
  void renderNormals();
  void renderDescriptors();
  void renderFullModelRecognition(Recognizer& rec);

  void point_pick_cb(const pcl::visualization::PointPickingEvent& event, void* viewer_void);
  void keyboard_cb(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
};


#endif //PCL_RECOGNIZER_VISUALIZER_H
