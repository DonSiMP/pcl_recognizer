#include "pcl_recognizer/recognizer.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl_recognizer/config.h>
#include <pcl_recognizer/utils.h>

int Recognizer::recognize(const PreprocessedData& scene, Pose& pose)
{
  scene_ = scene;
  if(!Config::shouldSkip(Config::Grouping))
  {
    findCorrespondences();
    clusterize();
  }
  return foundInstances.size();
}

void Recognizer::grouping_cb(pcl_recognizer::GroupingConfig& config, uint32_t level)
{
  use_hough_ = config.use_hough;
  cg_size_ = static_cast<float>(config.cluster_size);
  cg_thresh_ = static_cast<float>(config.cluster_thresh);
  corr_dist_ = static_cast<float>(config.corr_distance);
}

void Recognizer::findCorrespondences()
{
  Timer::Scoped timer("Correspondences");

  model_scene_corrs.reset(new pcl::Correspondences ());

  pcl::KdTreeFLANN<Descriptor> match_search;
  match_search.setInputCloud (model_.descriptors_);

  // For each scene keypoint descriptor, find nearest neighbor into the
  // model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene_.descriptors_->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!pcl_isfinite (scene_.descriptors_->at (i).descriptor[0])) //skipping NaNs
    {
      ROS_INFO("Infinite descriptor");
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_.descriptors_->at(i), 1, neigh_indices, neigh_sqr_dists);
    // add match only if the squared descriptor distance is
    // less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    if(found_neighs == 1 && neigh_sqr_dists[0] < corr_dist_)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  ROS_INFO_STREAM(
      "Correspondences found: " <<
      model_scene_corrs->size () <<
      std::endl);
}

void Recognizer::clusterize()
{
  correspondence_clusters_.clear();

  Timer::Scoped timer("Clustering");

  if(use_hough_)
    clusterizeHough();
  else
    clusterizeGC();
}

void Recognizer::clusterizeHough()
{
  pcl::Hough3DGrouping<Point, Point, ReferenceFrame, ReferenceFrame> clusterer;
  clusterer.setHoughBinSize (cg_size_);
  clusterer.setHoughThreshold (cg_thresh_);
  clusterer.setUseInterpolation (true);
  clusterer.setUseDistanceWeight (false);

  clusterer.setInputCloud (model_.keypoints_);
  clusterer.setInputRf (model_.rf_);
  clusterer.setSceneCloud (scene_.keypoints_);
  clusterer.setSceneRf (scene_.rf_);
  clusterer.setModelSceneCorrespondences (model_scene_corrs);

  clusterer.recognize (foundInstances, correspondence_clusters_);
}

void Recognizer::clusterizeGC()
{
  pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
  gc_clusterer.setGCSize (cg_size_);
  gc_clusterer.setGCThreshold (static_cast<int>(cg_thresh_));

  gc_clusterer.setInputCloud (model_.keypoints_);
  gc_clusterer.setSceneCloud (scene_.keypoints_);
  gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

  gc_clusterer.recognize (foundInstances, correspondence_clusters_);
}

Recognizer::Recognizer(): grouping_srv_(std::string("recognizer_grouping"))
{
  dynamic_reconfigure::Server<pcl_recognizer::GroupingConfig>::CallbackType grouping_server_cb;
  grouping_server_cb = boost::bind(&Recognizer::grouping_cb, this, _1, _2);
  grouping_srv_.setCallback(grouping_server_cb);
}
