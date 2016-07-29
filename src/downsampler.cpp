#include <pcl_recognizer/downsampler.h>
#include <pcl_recognizer/utils.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>

void Downsampler::computeKeypoints(PreprocessedData& data)
{
  Timer::Scoped timer("Keypoints");

  if(cfg_.method == 0)
    downsampleUniform(data);
  else if(cfg_.method == 1)
    downsampleISS(data);
  else if(cfg_.method == 2)
    downsampleSIFT(data);
  else
    downsampleHarris(data);

  std::cout <<
  "Cloud total points: " <<
  data.input_->size () <<
  "; Selected Keypoints: " <<
  data.keypoints_->size () <<
  std::endl;
}

void Downsampler::downsampleUniform(PreprocessedData& data)
{
  pcl::UniformSampling<Point> uniform_sampling;
  uniform_sampling.setInputCloud (data.input_);
  uniform_sampling.setRadiusSearch (cfg_.uniform_radius);
  uniform_sampling.filter (*data.keypoints_);
}

void Downsampler::downsampleISS(PreprocessedData& data)
{
  auto iss_gamma_21_ = 0.975;
  auto iss_gamma_32_ = 0.975;
  auto iss_min_neighbors_ = 5;
  auto iss_threads_ = cfg_.omp_threads;

  auto iss_non_max_radius_ = cfg_.iss_non_max_radius;//4 * resolution_;
  auto iss_border_radius_ = cfg_.iss_border_radius;//1 * resolution_;
  auto iss_salient_radius_ = cfg_.iss_salient_radius;//6 * resolution_;

  if(cfg_.iss_use_resolution)
  {
    iss_non_max_radius_ = 8 * data.input_resolution_;
    iss_border_radius_ = data.input_resolution_;
    iss_salient_radius_ = 6 * data.input_resolution_;
  }

  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
  tree->setInputCloud(data.input_);

//
// Compute keypoints
//
  pcl::ISSKeypoint3D<Point, Point> iss_detector;

  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (iss_salient_radius_);
  iss_detector.setNonMaxRadius (iss_non_max_radius_);
  iss_detector.setBorderRadius (iss_border_radius_);

  iss_detector.setThreshold21 (iss_gamma_21_);
  iss_detector.setThreshold32 (iss_gamma_32_);
  iss_detector.setMinNeighbors (iss_min_neighbors_);
  iss_detector.setNumberOfThreads (iss_threads_);
  iss_detector.setInputCloud (data.input_);
  iss_detector.setNormals (data.normals_);
  iss_detector.compute (*data.keypoints_);

  auto indices = iss_detector.getKeypointsIndices();
  *data.keypoint_idxes_ = indices->indices;
}

void Downsampler::downsampleSIFT(PreprocessedData& data)
{
  pcl::SIFTKeypoint<Point, Point> sift_detector;
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  sift_detector.setSearchMethod(tree);
  sift_detector.setScales (cfg_.sift_min_scale,
                           cfg_.sift_nr_octaves,
                           cfg_.sift_nr_scales_per_octave);
  sift_detector.setMinimumContrast (cfg_.sift_min_contrast);
  sift_detector.setInputCloud(data.input_);
  sift_detector.compute(*data.keypoints_);
}

void Downsampler::downsampleHarris(PreprocessedData& data)
{
  pcl::PointCloud<pcl::PointXYZI> output;

  pcl::HarrisKeypoint3D<Point, pcl::PointXYZI> harris_detector;
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  harris_detector.setSearchMethod(tree);
  harris_detector.setNonMaxSupression(cfg_.harris_use_nonmaxima);
  harris_detector.setRadius(cfg_.harris_radius);
  harris_detector.setRadiusSearch(cfg_.harris_radius_search);
  harris_detector.setNumberOfThreads(4);
  harris_detector.setNormals(data.normals_);
  harris_detector.setInputCloud(data.input_);
  harris_detector.compute(output);

  pcl::copyPointCloud(output, *data.keypoints_);
}
