#include "pcl_recognizer/visualizer.h"

Visualizer::Visualizer(std::string title) : vis_(title)
{
  vis_.setBackgroundColor (0, 0, 0);
  vis_.addCoordinateSystem (1.0);
  vis_.initCameraParameters ();
}

void Visualizer::update(PreprocessedData& data, pcl_recognizer::ViewerConfig& cfg)
{
  if(cfg.input) {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(data.input_);
    vis_.addPointCloud<pcl::PointXYZRGB>(data.input_, rgb, "sample cloud");
  }
}

void Visualizer::spin()
{
  vis_.spinOnce(100);
}
