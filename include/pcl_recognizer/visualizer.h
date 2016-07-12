#ifndef PCL_RECOGNIZER_VISUALIZER_H
#define PCL_RECOGNIZER_VISUALIZER_H

#include <dynamic_reconfigure/server.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/ViewerConfig.h>


class Visualizer
{
public:
  Visualizer(std::string title = "pcl_recognizer");

  void update(PreprocessedData& data);
  void renderRecognition(Recognizer& rec);
  void spin();
private:
  PreprocessedData data_;
  pcl::visualization::PCLVisualizer vis_;

  bool show_recognition = false;

  pcl_recognizer::ViewerConfig cfg_;
  pcl_recognizer::ViewerConfig init_status_;
  dynamic_reconfigure::Server<pcl_recognizer::ViewerConfig> cfg_srv_;

  void params_cb(pcl_recognizer::ViewerConfig &config, uint32_t level);

  using ColorHandler = pcl::visualization::PointCloudColorHandler<Point>;
  void renderRGB(bool cfg, const Cloud::Ptr& data, const ColorHandler& rgb, const std::string& name, bool& status);
  void renderNormals();

  void point_pick_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);
};


#endif //PCL_RECOGNIZER_VISUALIZER_H
