#ifndef PCL_RECOGNIZER_VISUALIZER_H
#define PCL_RECOGNIZER_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/ViewerConfig.h>

class Visualizer
{
public:
  Visualizer(std::string title = "pcl_recognizer");

  void update(PreprocessedData& data, pcl_recognizer::ViewerConfig& cfg);
  void spin();
private:
  pcl::visualization::PCLVisualizer vis_;

};


#endif //PCL_RECOGNIZER_VISUALIZER_H
