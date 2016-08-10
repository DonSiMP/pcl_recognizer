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


void Recognizer::addModel(const PreprocessedData& model)
{
  data_.push_back({model});
  std::cout << "<<<<<<<<< input: " << data_.back().model_.input_->size() << std::endl;
  std::cout << "<<<<<<<<< descrs: " << data_.back().model_.descriptors_->size() << std::endl;
}

int Recognizer::recognize(const PreprocessedData& scene, Pose& pose)
{
  global_hypotheses_mask_.clear();
  registered_instances_.clear();
  scene_ = scene;

  for(auto& model_data : data_)
  {
    std::cout << "<<<<<<<<< input: " << model_data.model_.input_->size() << std::endl;
    std::cout << "<<<<<<<<< descrs: " << model_data.model_.descriptors_->size() << std::endl;
    if (Config::shouldRun(Config::Grouping))
    {
      findCorrespondences(model_data);
      std::cout << "Correspondences found: " << model_data.correspondences_->size() << std::endl;
      clusterize(model_data);
      std::cout << "Clusters found " << model_data.clusters_.size() << std::endl;
    }

    if (Config::shouldRun(Config::AbsoluteOrientation) && cfg_.refine_svd)
      refineAbsoluteOrientation(model_data);

    if (Config::shouldRun(Config::IterativeClosestPoint) && cfg_.refine_icp)
      refineICP(model_data);
  }

  if(Config::shouldRun(Config::HypothesisVerification))
    verifyHypotheses();

  done = true;
  return static_cast<int>(
      std::count(std::begin(global_hypotheses_mask_),
                 std::end(global_hypotheses_mask_),
                 true)
  );
}

void Recognizer::findCorrespondences(RecognizedData& model_data)
{
  //TODO: something is not cleaned up properly
  Timer::Scoped timer("Correspondences");

  model_data.correspondences_.reset(new pcl::Correspondences());

  pcl::KdTreeFLANN<Descriptor, ::flann::L1<float>> match_search;
  match_search.setInputCloud(model_data.model_.descriptors_);

  // For each scene keypoint descriptor, find nearest neighbor into the
  // model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t descr_idx = 0; descr_idx < scene_.descriptors_->size(); ++descr_idx)
  {
    std::vector<int> neigh_indices(5);
    std::vector<float> neigh_sqr_dists(5);
    if (!pcl_isfinite (scene_.descriptors_->at(descr_idx).descriptor[0])) //skipping NaNs
    {
      std::cerr << "Infinite descriptor" << std::endl;
      continue;
    }

    int found_neighs = match_search.radiusSearch(
        scene_.descriptors_->at(descr_idx),
        cfg_.corr_distance,
        neigh_indices,
        neigh_sqr_dists,
        5
    );
    // add match only if the squared descriptor distance is
    // less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    for(int corr_idx = 0; corr_idx < found_neighs; corr_idx++)
    {
      pcl::Correspondence corr(neigh_indices[corr_idx],
                               static_cast<int>(descr_idx),
                               neigh_sqr_dists[corr_idx]);
      model_data.correspondences_->push_back(corr);
    }
  }
}

void Recognizer::clusterize(RecognizedData& model_data)
{
  model_data.clusters_.clear();

  Timer::Scoped timer("Clustering");

  if(cfg_.use_hough)
    clusterizeHough(model_data);
  else
    clusterizeGC(model_data);
}

void Recognizer::clusterizeHough(RecognizedData& model_data)
{
  pcl::Hough3DGrouping<Point, Point, ReferenceFrame, ReferenceFrame> clusterer;
  clusterer.setHoughBinSize (cfg_.cluster_size);
  clusterer.setHoughThreshold (cfg_.cluster_thresh);
  clusterer.setUseInterpolation (true);
  clusterer.setUseDistanceWeight (false);

  clusterer.setInputCloud (model_data.model_.keypoints_);
  clusterer.setInputRf (model_data.model_.rf_);
  clusterer.setSceneCloud (scene_.keypoints_);
  clusterer.setSceneRf (scene_.rf_);
  clusterer.setModelSceneCorrespondences (model_data.correspondences_);

  clusterer.recognize (model_data.poses_, model_data.clusters_);
}

void Recognizer::clusterizeGC(RecognizedData& model_data)
{
  pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
  gc_clusterer.setGCSize(cfg_.cluster_size);
  gc_clusterer.setGCThreshold(static_cast<int>(cfg_.cluster_thresh));

  gc_clusterer.setInputCloud(model_data.model_.keypoints_);
  gc_clusterer.setSceneCloud(scene_.keypoints_);
  gc_clusterer.setModelSceneCorrespondences(model_data.correspondences_);

  gc_clusterer.recognize(model_data.poses_, model_data.clusters_);
}

void Recognizer::refineAbsoluteOrientation(RecognizedData& model_data)
{
  Timer::Scoped timer("AbsoluteOrientation");

  pcl::registration::TransformationEstimationSVD<Point, Point> trans_est;
  pcl::registration::CorrespondenceRejectorSampleConsensus<Point> rejector;
  rejector.setInputSource(model_data.model_.getKeypointCloud());
  rejector.setInputTarget(scene_.getKeypointCloud());
  for(auto idx = 0u; idx < model_data.poses_.size(); ++idx)
  {
    rejector.getRemainingCorrespondences(model_data.clusters_.at(idx),
                                         model_data.clusters_.at(idx));
    trans_est.estimateRigidTransformation(*model_data.model_.getKeypointCloud(),
                                          *scene_.getKeypointCloud(),
                                          model_data.clusters_.at(idx),
                                          model_data.poses_.at(idx));
  }
}

void Recognizer::refineICP(RecognizedData& model_data)
{
  Timer::Scoped timer("IterativeClosestPoint");

  pcl::IterativeClosestPoint<Point, Point> icp;
  icp.setInputSource(model_data.model_.input_);
  icp.setInputTarget(scene_.input_);
// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(cfg_.icp_corr_dist);
// Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (cfg_.icp_max_iter);
// Set the transformation epsilon (criterion 2)
//  icp.setTransformationEpsilon (1e-8);
// Set the euclidean distance difference epsilon (criterion 3)
//  icp.setEuclideanFitnessEpsilon (1);
  for(auto idx = 0u; idx < model_data.poses_.size(); ++idx)
  {
    pcl::PointCloud<Point>::Ptr model_registered(new pcl::PointCloud<Point>);
    icp.align(*model_registered, model_data.poses_.at(idx));
    if (icp.hasConverged())
    {
      registered_instances_.push_back(model_registered);
      std::cout << (model_data.poses_.at(idx) = icp.getFinalTransformation()) << std::endl;
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
  hv.getMask(global_hypotheses_mask_);  // i-element TRUE if hvModels[i] verifies hypotheses
  for (int i = 0; i < global_hypotheses_mask_.size(); i++)
  {
    if (global_hypotheses_mask_[i])
      std::cout << "Instance " << i << " is GOOD! <---" << std::endl;
    else
      std::cout << "Instance " << i << " is bad!" << std::endl;
  }

  //TODO: fill verification for each result
}

void Recognizer::reset()
{
  data_.clear();
  global_hypotheses_mask_.clear();
  registered_instances_.clear();
}
