#include "pcl_recognizer/recognizer.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl_recognizer/config.h>

int Recognizer::recognize(const PreprocessedData& scene, Pose& pose)
{
  scene_ = scene;

  if(Config::get().stop_at < Config::StopAt::Grouping)
    return 0;

  findCorrespondences();
  clusterize();

  return foundInstances.size();
}

void Recognizer::findCorrespondences()
{
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
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
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
  if(!use_hough_)
    clusterizeWithGC();

  std::vector<pcl::Correspondences> clustered_corrs;

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

  clusterer.recognize (foundInstances, clustered_corrs);
}

void Recognizer::cfg_cb(pcl_recognizer::GroupingConfig& config, uint32_t level)
{
  use_hough_ = config.use_hough;
  cg_size_ = static_cast<float>(config.cg_size);
  cg_thresh_ = static_cast<float>(config.cg_thresh);
}

void Recognizer::clusterizeWithGC()
{
  std::vector<pcl::Correspondences> clustered_corrs;

  pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
  gc_clusterer.setGCSize (cg_size_);
  gc_clusterer.setGCThreshold (cg_thresh_);

  gc_clusterer.setInputCloud (model_.keypoints_);
  gc_clusterer.setSceneCloud (scene_.keypoints_);
  gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

  gc_clusterer.recognize (foundInstances, clustered_corrs);

}
