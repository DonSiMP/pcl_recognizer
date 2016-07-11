#include "pcl_recognizer/visualizer.h"

Visualizer::Visualizer(std::string title) : vis_(title), cfg_srv_(title)
{
  vis_.setBackgroundColor (0.1, 0.1, 0.1);
  vis_.addCoordinateSystem (0.2);
  vis_.initCameraParameters ();
  vis_.resetCamera();

  init_status_.input = false;
  init_status_.keypoints = false;
  init_status_.normals = false;
  init_status_.descriptors = false;

  dynamic_reconfigure::Server<pcl_recognizer::ViewerConfig>::CallbackType params_server_cb;
  params_server_cb = boost::bind(&Visualizer::params_cb, this, _1, _2);
  cfg_srv_.setCallback(params_server_cb);
}

void Visualizer::update(PreprocessedData& data)
{
  if(!cfg_.update)
    return;
  data_ = data;
}

void Visualizer::renderRGB(bool cfg, const Cloud::Ptr& data, const  ColorHandler& rgb, const std::string& name, bool& status)
{
  if(cfg)
  {
    if(!status)
    {
      vis_.addPointCloud(data, rgb, name);
      status = true;
    }
    else
      vis_.updatePointCloud(data, rgb, name);
  }
  else if (status)
  {
    vis_.removePointCloud(name);
    status = false;
  }
}

void Visualizer::spin()
{
  if(cfg_.update)
  {

    pcl::visualization::PointCloudColorHandlerRGBField<Point> colorInput(data_.input_);
    renderRGB(cfg_.input, data_.input_, colorInput, "input", init_status_.input);
    pcl::visualization::PointCloudColorHandlerCustom<Point> colorKeypoints(data_.input_, 255, 0, 0);
    renderRGB(cfg_.keypoints, data_.getKeypointCloud(), colorKeypoints, "keypoints", init_status_.keypoints);
    vis_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  }
  vis_.spinOnce(1000);
}

void Visualizer::params_cb(pcl_recognizer::ViewerConfig& config, uint32_t level)
{
    cfg_ = config;
}
