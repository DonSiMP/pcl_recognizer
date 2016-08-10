#include <pcl_recognizer/config.h>
#include <pcl_recognizer/recognizer.h>
#include <pcl_recognizer/visualizer.h>
#include <pcl/common/transforms.h>

Visualizer::Visualizer(std::string title) : Reconfigurable(title), vis_(title), plot_(title.c_str())
{
  vis_.setBackgroundColor (0.1, 0.1, 0.1);
  vis_.addCoordinateSystem (0.2);
  vis_.initCameraParameters ();
  vis_.resetCamera();

  init_status_.input = false;
  init_status_.keypoints = false;
  init_status_.normals = false;
  init_status_.descriptors = false;

  vis_.registerPointPickingCallback(&Visualizer::point_pick_cb, *this);
  vis_.registerKeyboardCallback(&Visualizer::keyboard_cb, *this);
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
  if(!rec.finished())
    return;
  show_recognition = true;

  std::cout << "showing recognition " << std::endl;

  if(Config::get().use_full_model)
    renderFullModelRecognition(rec);

  for(; current_registered_instances >= 0; --current_registered_instances)
    vis_.removePointCloud(std::to_string(current_registered_instances) + "instance");

  if(Config::shouldRun(Config::HypothesisVerification))
  {
    const auto& results = rec.getRecognitionResults();
    for(const auto& result : results)
      for(const auto& pose : result.poses_)
      {
        std::cout << "debug: showing recognition " << current_registered_instances << std::endl;
        using ColorizeCloud = pcl::visualization::PointCloudColorHandlerCustom<Point>;
        pcl::PointCloud<Point>::Ptr rotated_model(new Cloud());
        pcl::transformPointCloud(*result.model_.input_, *rotated_model, pose);
        ColorizeCloud red(rotated_model, 255, 0, 0);
        ColorizeCloud green(rotated_model, 0, 255, 0);
        vis_.addPointCloud(rotated_model,
                           (!result.verification_.empty() && result.verification_.at(current_registered_instances)) ? green : red,
                           std::to_string(current_registered_instances) + "instance");
        current_registered_instances++;
      }
  }
}

void Visualizer::renderFullModelRecognition(Recognizer& rec)
{
  auto results = rec.getRecognitionResults();
  Cloud::Ptr off_scene_model(new Cloud()), off_scene_model_keypoints(new Cloud());
  pcl::transformPointCloud (*results.front().model_.input_,
                            *off_scene_model,
                            Eigen::Vector3f(-.5,0,0),
                            Eigen::Quaternionf(0.5, 0, 0.86603, 0));
  pcl::transformPointCloud (*results.front().model_.keypoints_,
                            *off_scene_model_keypoints,
                            Eigen::Vector3f (-.5,0,0),
                            Eigen::Quaternionf (0.5, 0, 0.86603, 0));
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
  for(; current_corr_clusters > 0; --current_corr_clusters)
    vis_.removeCorrespondences(std::to_string(current_corr_clusters) + "cluster");

  if((Config::get().stop_at >= Config::StopAt::Grouping))
  {
    if (cfg_.view_clusters)
    {
      for (const auto& result : rec.getRecognitionResults())
        for (const auto& corrs : result.clusters_)
        {
          vis_.addCorrespondences<Point>(off_scene_model_keypoints,
                                         scene.keypoints_,
                                         corrs,
                                         std::to_string(current_corr_clusters++) + "cluster");
        }
    }
    else
      for (const auto& result : rec.getRecognitionResults())
        vis_.addCorrespondences<Point>(off_scene_model_keypoints,
                                       scene.keypoints_,
                                       *result.correspondences_,
                                       std::to_string(current_corr_clusters++) + "cluster");
  }
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
    renderDescriptors();
  }
  vis_.spinOnce(1000);
  plot_.spinOnce(100);
}

void Visualizer::point_pick_cb(const pcl::visualization::PointPickingEvent& event,
                               void* viewer_void)
{
  if(event.getPointIndex()!=-1)
  {
    float x,y,z;
    event.getPoint(x,y,z);
    picked_points.emplace_back(x,y,z);

    if(!data_.descriptors_->empty())
      new_picks = true;
    else
      std::cout << "Descriptor pick failed. Compute descriptors first." << std::endl;
  }
}

void Visualizer::keyboard_cb(const pcl::visualization::KeyboardEvent &event,
                             void* viewer_void)
{
  if (event.getKeySym() == "q")
  {
    std::cout << "Clearing plots.." << std::endl;
    plot_.clearPlots();
    picked_points.clear();
    std::cout << " --- Picked points : " << picked_points.size() << std::endl;
  }
}

#include <pcl/search/kdtree.h>

void Visualizer::renderDescriptors() {
  if(!new_picks)
    return;

  if(data_.descriptors_->empty())
    return;

  plot_.clearPlots();

  std::vector<int> neigh_indices(1);
  std::vector<float> neigh_sqr_dists(1);
  pcl::search::KdTree<pcl::PointXYZ> tree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr coords_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*data_.keypoints_, *coords_cloud);
  tree.setInputCloud(coords_cloud);

  for(const auto& point : picked_points)
  {
    std::cout << "debug0" << std::endl;
    int found_neighs = tree.nearestKSearch(point, 1, neigh_indices, neigh_sqr_dists);
    std::cout << "debug1" << std::endl;
    if (found_neighs > 0)
    {
      double indexes[352] = {};
      double descriptor[352] = {};
      for (int idx = 0; idx < 352; idx++)
      {
        indexes[idx] = idx;
        descriptor[idx] = data_.descriptors_->at(neigh_indices[0]).descriptor[idx];
      }
      std::cout << "debug2" << std::endl;

      std::stringstream ss;
      ss << "HistogramOf" << point;

      plot_.addPlotData(indexes, descriptor, 352, ss.str().c_str());

      std::cout << "debug3" << std::endl;
    }

  }
  std::cout << "debug4" << std::endl;
  new_picks = false;
}