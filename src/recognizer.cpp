#include <pcl_recognizer/recognizer.h>

#include <algorithm>
#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#undef PCL_NO_PRECOMPILE

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl_recognizer/config.h>
#include <pcl_recognizer/utils.h>


int Recognizer::recognize(const PreprocessedData& scene, Pose& pose)
{
  hypotheses_mask_.clear();
  scene_ = scene;
  if(Config::shouldRun(Config::Grouping))
  {
    findCorrespondences();
    clusterize();
  }

  if(Config::shouldRun(Config::AbsoluteOrientation) && cfg_.refine_svd)
    refineAbsoluteOrientation();

  if(Config::shouldRun(Config::IterativeClosestPoint) && cfg_.refine_icp)
    refineICP();

  if(Config::shouldRun(Config::HypothesisVerification))
    verifyHypotheses();

  done = true;
  return static_cast<int>(std::count(std::begin(hypotheses_mask_), std::end(hypotheses_mask_), true));
}

void Recognizer::findCorrespondences()
{
  Timer::Scoped timer("Correspondences");

  model_scene_corrs_.reset(new pcl::Correspondences ());

  pcl::KdTreeFLANN<Descriptor, ::flann::L1<float>> match_search;
  match_search.setInputCloud (model_.descriptors_);

  // For each scene keypoint descriptor, find nearest neighbor into the
  // model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t descr_idx = 0; descr_idx < scene_.descriptors_->size (); ++descr_idx)
  {
    std::vector<int> neigh_indices (5);
    std::vector<float> neigh_sqr_dists (5);
    if (!pcl_isfinite (scene_.descriptors_->at (descr_idx).descriptor[0])) //skipping NaNs
    {
      ROS_INFO("Infinite descriptor");
      continue;
    }
    // TODO: change norm to L1
    int found_neighs = match_search.radiusSearch(scene_.descriptors_->at(descr_idx), cfg_.corr_distance, neigh_indices, neigh_sqr_dists, 5);
    // add match only if the squared descriptor distance is
    // less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    for(int corr_idx = 0; corr_idx < found_neighs; corr_idx++)
    {
      pcl::Correspondence corr (neigh_indices[corr_idx], static_cast<int>(descr_idx), neigh_sqr_dists[corr_idx]);
      model_scene_corrs_->push_back (corr);
    }
  }
  ROS_INFO_STREAM(
      "Correspondences found: " <<
      model_scene_corrs_->size () <<
      std::endl);
}

void Recognizer::clusterize()
{
  correspondence_clusters_.clear();

  Timer::Scoped timer("Clustering");

  if(cfg_.use_hough)
    clusterizeHough();
  else
    clusterizeGC();
}

void Recognizer::clusterizeHough()
{
  pcl::Hough3DGrouping<Point, Point, ReferenceFrame, ReferenceFrame> clusterer;
  clusterer.setHoughBinSize (cfg_.cluster_size);
  clusterer.setHoughThreshold (cfg_.cluster_thresh);
  clusterer.setUseInterpolation (true);
  clusterer.setUseDistanceWeight (false);

  clusterer.setInputCloud (model_.keypoints_);
  clusterer.setInputRf (model_.rf_);
  clusterer.setSceneCloud (scene_.keypoints_);
  clusterer.setSceneRf (scene_.rf_);
  clusterer.setModelSceneCorrespondences (model_scene_corrs_);

  clusterer.recognize (found_poses_, correspondence_clusters_);
}

void Recognizer::clusterizeGC()
{
  pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
  gc_clusterer.setGCSize (cfg_.cluster_size);
  gc_clusterer.setGCThreshold (static_cast<int>(cfg_.cluster_thresh));

  gc_clusterer.setInputCloud (model_.keypoints_);
  gc_clusterer.setSceneCloud (scene_.keypoints_);
  gc_clusterer.setModelSceneCorrespondences (model_scene_corrs_);

  gc_clusterer.recognize (found_poses_, correspondence_clusters_);
}

void Recognizer::refineAbsoluteOrientation()
{
  Timer::Scoped timer("AbsoluteOrientation");

  pcl::registration::TransformationEstimationSVD<Point, Point> trans_est;
  pcl::registration::CorrespondenceRejectorSampleConsensus<Point> rejector;
  rejector.setInputSource(model_.getKeypointCloud());
  rejector.setInputTarget(scene_.getKeypointCloud());
  for(auto idx = 0u; idx < found_poses_.size(); ++idx)
  {
    rejector.getRemainingCorrespondences(correspondence_clusters_.at(idx),correspondence_clusters_.at(idx));
    trans_est.estimateRigidTransformation(*model_.getKeypointCloud(),
                                          *scene_.getKeypointCloud(),
                                          correspondence_clusters_.at(idx),
                                          found_poses_.at(idx));
  }
}

void Recognizer::refineICP()
{
  Timer::Scoped timer("IterativeClosestPoint");

  pcl::IterativeClosestPoint<Point, Point> icp;
  icp.setInputSource(model_.input_);
  icp.setInputTarget(scene_.input_);
// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(cfg_.icp_corr_dist);
// Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (cfg_.icp_max_iter);
// Set the transformation epsilon (criterion 2)
//  icp.setTransformationEpsilon (1e-8);
// Set the euclidean distance difference epsilon (criterion 3)
//  icp.setEuclideanFitnessEpsilon (1);
  registered_instances_.clear();
  for(auto idx = 0u; idx < found_poses_.size(); ++idx)
  {
    pcl::PointCloud<Point>::Ptr model_registered(new pcl::PointCloud<Point>);
    icp.align(*model_registered, found_poses_.at(idx));
    if (icp.hasConverged())
    {
      registered_instances_.push_back(model_registered);
      std::cout << (found_poses_.at(idx) = icp.getFinalTransformation()) << std::endl;
    }
  }
}

void Recognizer::verifyHypotheses()
{
  pcl::GlobalHypothesesVerification<Point, Point> hv;

  hv.setSceneCloud(scene_.input_);
  hv.addModels(registered_instances_, true);

  hv.setInlierThreshold(cfg_.hv_inlier_th);
  hv.setOcclusionThreshold (cfg_.hv_occlusion_th);
  hv.setRegularizer(cfg_.hv_regularizer);
  hv.setRadiusClutter(cfg_.hv_rad_clutter);
  hv.setClutterRegularizer(cfg_.hv_clutter_reg);
  hv.setDetectClutter(cfg_.hv_detect_clutter);
  hv.setRadiusNormals(cfg_.hv_rad_normals);

  hv.verify();
  hv.getMask(hypotheses_mask_);  // i-element TRUE if hvModels[i] verifies hypotheses
  for (int i = 0; i < hypotheses_mask_.size (); i++)
  {
    if (hypotheses_mask_[i])
    {
      std::cout << "Instance " << i << " is GOOD! <---" << std::endl;
    }
    else
    {
      std::cout << "Instance " << i << " is bad!" << std::endl;
    }
  }
}
