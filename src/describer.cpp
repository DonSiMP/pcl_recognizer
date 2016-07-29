#include <pcl_recognizer/describer.h>
#include <pcl_recognizer/utils.h>

#include <pcl/features/board.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/surface/mls.h>

void Describer::computeNormals(PreprocessedData& data)
{
  Timer::Scoped timer("Normals");

  if(cfg_.normal_method == 0)
    computeNormalsOMP(data);
  else if(cfg_.normal_method == 1)
    computeNormalsINT(data);
  else
    computeNormalsMLS(data);
}

void Describer::computeNormalsOMP(PreprocessedData& data)
{
  pcl::NormalEstimationOMP<Point, Normal> norm_est;
  norm_est.setNumberOfThreads(cfg_.omp_threads);
  norm_est.setKSearch (cfg_.normal_ksize);
  norm_est.setRadiusSearch(cfg_.normal_rad);
  norm_est.setInputCloud (data.input_);
  norm_est.compute (*data.normals_);
}

void Describer::computeNormalsINT(PreprocessedData& data)
{
  pcl::IntegralImageNormalEstimation<Point, Normal> ne;
  ne.setNormalEstimationMethod(static_cast<decltype(ne.COVARIANCE_MATRIX)>(cfg_.int_method));
  ne.setMaxDepthChangeFactor(cfg_.int_normal_maxdepth);
  ne.setNormalSmoothingSize(cfg_.int_normal_smoothing);
  ne.setInputCloud(data.input_);
  ne.compute(*data.normals_);
}

void Describer::computeNormalsMLS(PreprocessedData& data)
{
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
  pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;
  pcl::MovingLeastSquares<Point, pcl::PointXYZRGBNormal> mls;
  mls.setComputeNormals (true);

  mls.setInputCloud (data.input_);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (cfg_.normal_rad);

  mls.process(mls_points);

  pcl::copyPointCloud(mls_points, *data.input_);
  pcl::copyPointCloud(mls_points, *data.normals_);
}

void Describer::computeReferenceFrames(PreprocessedData& data)
{
  Timer::Scoped timer("Reference frames");

  pcl::BOARDLocalReferenceFrameEstimation<Point, Normal, ReferenceFrame> rf_est;
  rf_est.setFindHoles (true);
  rf_est.setRadiusSearch (cfg_.rf_rad);

  rf_est.setInputCloud (data.keypoints_);
  rf_est.setInputNormals (data.normals_);
  rf_est.setSearchSurface (data.input_);
  rf_est.compute (*data.rf_);
}

void Describer::computeDescriptors(PreprocessedData& data)
{
  Timer::Scoped timer("Descriptors");

  pcl::SHOTColorEstimationOMP<Point, Normal> descr_est;
  descr_est.setRadiusSearch (cfg_.descr_rad);
  descr_est.setNumberOfThreads(4);
  descr_est.setInputCloud (data.keypoints_);
  descr_est.setInputNormals (data.normals_);
  descr_est.setInputReferenceFrames(data.rf_);
  descr_est.setSearchSurface (data.input_);
  descr_est.compute(*data.descriptors_);
}