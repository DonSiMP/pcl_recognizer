#include <pcl_recognizer/config.h>
#include <pcl_recognizer/recognizer.h>
#include <pcl_recognizer/visualizer.h>
#include <pcl/common/transforms.h>

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

  vis_.registerPointPickingCallback(&Visualizer::point_pick_callback, *this);
}

void Visualizer::update(PreprocessedData& data)
{
  if(!cfg_.update)
    return;
  data_ = data;
  show_recognition = false;
}

void Visualizer::renderRecognition(Recognizer& rec)
{
  show_recognition = true;

  auto model = rec.getModel();
  Cloud::Ptr off_scene_model(new Cloud()), off_scene_model_keypoints(new Cloud());
  pcl::transformPointCloud (*model.input_, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
  pcl::transformPointCloud (*model.keypoints_, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
  {
    vis_.removePointCloud("model_input");
    vis_.removePointCloud("model_keypoints");
    pcl::visualization::PointCloudColorHandlerRGBField<Point> colorInput(off_scene_model);
    vis_.addPointCloud(off_scene_model, colorInput, "model_input");
    pcl::visualization::PointCloudColorHandlerCustom<Point> colorKeypoints(off_scene_model_keypoints, 0, 255, 0);
    vis_.addPointCloud(off_scene_model_keypoints, colorKeypoints, "model_keypoints");
    vis_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "model_keypoints");
  }

  auto scene = rec.getScene();
  {
    vis_.removePointCloud("scene_input");
    vis_.removePointCloud("scene_keypoints");
    pcl::visualization::PointCloudColorHandlerRGBField<Point> colorInput(scene.input_);
    vis_.addPointCloud(scene.input_, colorInput, "scene_input");
    pcl::visualization::PointCloudColorHandlerCustom<Point> colorKeypoints(scene.keypoints_, 255, 0, 0);
    vis_.addPointCloud(scene.keypoints_, colorKeypoints, "scene_keypoints");
    vis_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "scene_keypoints");
  }

  vis_.removeCorrespondences();

  if((Config::get().stop_at >= Config::StopAt::Grouping))
    vis_.addCorrespondences<Point>(off_scene_model_keypoints, scene.keypoints_, *rec.getCorrs());
}

void Visualizer::renderRGB(bool cfg, const Cloud::Ptr& data, const  ColorHandler& rgb, const std::string& name, bool& status)
{
  if(!status)
  {
    vis_.addPointCloud(data, rgb, name);
    status = true;
  }
  else
    vis_.updatePointCloud(data, rgb, name);
}

void Visualizer::renderInput()
{
  if(cfg_.input)
  {
    pcl::visualization::PointCloudColorHandlerRGBField<Point> colorInput(data_.input_);
    renderRGB(cfg_.input, data_.input_, colorInput, "input", init_status_.input);
  }
  else if (init_status_.input)
  {
    vis_.removePointCloud("input");
    init_status_.input = false;
  }
}

void Visualizer::renderNormals()
{
  if (init_status_.normals)
  {
    vis_.removePointCloud("normals");
    init_status_.normals = false;
  }

  if(cfg_.normals && (Config::get().stop_at >= Config::StopAt::Normals))
  {
    vis_.addPointCloudNormals<Point, Normal>(data_.input_, data_.normals_, 10, 0.02, "normals");
    init_status_.normals = true;
  }
}

void Visualizer::renderKeypoints()
{
  if(cfg_.keypoints && (Config::get().stop_at >= Config::StopAt::Keypoints))
  {
    pcl::visualization::PointCloudColorHandlerCustom<Point> colorKeypoints(data_.keypoints_, 255, 0, 0);
    renderRGB(cfg_.keypoints, data_.getKeypointCloud(), colorKeypoints, "keypoints", init_status_.keypoints);
    vis_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  }
  else if (init_status_.keypoints)
  {
    vis_.removePointCloud("keypoints");
    init_status_.keypoints = false;
  }
}

void Visualizer::spin()
{
  if(cfg_.update && !show_recognition)
  {
    renderInput();
    renderNormals();
    renderKeypoints();
  }
  vis_.spinOnce(1000);
}

void Visualizer::params_cb(pcl_recognizer::ViewerConfig& config, uint32_t level)
{
    cfg_ = config;
}

void Visualizer::point_pick_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
  std::cout << "Picking event active" << std::endl;
  if(event.getPointIndex()!=-1)
  {
    float x,y,z;
    event.getPoint(x,y,z);
    std::cout << x<< ";" << y<<";" << z << std::endl;
  }
}