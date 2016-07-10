#include "pcl_recognizer/preprocessor.h"

#include <pcl/conversions.h>
#include <pcl/features/board.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/io/pcd_io.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/iss_3d.h>

PreprocessedData Preprocessor::load(std::string file_name)
{
  data_.reset();

  pcl::PCLPointCloud2 cloud2;
  if (pcl::io::loadPCDFile(file_name, cloud2) == -1 || cloud2.width == 0)
    throw std::runtime_error("Failed to load model from " + file_name);
  pcl::fromPCLPointCloud2(cloud2, *data_.input_);

  auto fields = cloud2.fields;
  auto has_normals = std::any_of(std::begin(fields), std::end(fields),
                                 [](const pcl::PCLPointField& field)
                                 { return field.name == "normal_x"; });
  if(has_normals)
    pcl::fromPCLPointCloud2(cloud2, *data_.normals_);

  preprocess();

  return data_;
}

void Preprocessor::preprocess()
{
  if(data_.normals_->size() == 0)
    computeNormals();

  if(use_cloud_resolution_)
  {
    computeResolution();
    std::cout << "Cloud resolution " << resolution_ << std::endl;
  }

  downsample();
  computeDescriptors();
  computeReferenceFrames();
}

void Preprocessor::computeNormals()
{
  pcl::NormalEstimationOMP<Point, Normal> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (data_.input_);
  norm_est.compute (*data_.normals_);
}

void Preprocessor::computeDescriptors()
{
  pcl::SHOTEstimationOMP<Point, Normal, Descriptor> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (data_.keypoints_);
  descr_est.setInputNormals (data_.normals_);
  descr_est.setSearchSurface (data_.input_);
  descr_est.compute (*data_.descriptors_);
}

void Preprocessor::computeReferenceFrames()
{
  pcl::BOARDLocalReferenceFrameEstimation<Point, Normal, ReferenceFrame> rf_est;
  rf_est.setFindHoles (true);
  rf_est.setRadiusSearch (rf_rad_);

  rf_est.setInputCloud (data_.keypoints_);
  rf_est.setInputNormals (data_.normals_);
  rf_est.setSearchSurface (data_.input_);
  rf_est.compute (*data_.rf_);
}

void Preprocessor::reconfigure(pcl_recognizer::ParamsConfig& config, ParamContext ctx)
{
  descr_rad_ = static_cast<float>(config.descr_rad);
  rf_rad_ = static_cast<float>(config.rf_rad);
  if(ctx == ParamContext::Model)
    sampling_size_ = static_cast<float>(config.model_ss);
  else
    sampling_size_ = static_cast<float>(config.scene_ss);
}

void Preprocessor::computeResolution()
{
  int n_points = 0;
  std::vector<int> indices(2);
  std::vector<float> sqr_distances(2);

  pcl::search::KdTree<Point> tree;
  tree.setInputCloud(data_.input_);

  for(int i = 0; i < data_.input_->size(); ++i)
  {
    if (!pcl_isfinite((*data_.input_)[i].x))
      continue;

    //Considering the second neighbor since the first is the point itself.
    int nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      resolution_ += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }

  if (n_points != 0)
    resolution_ /= n_points;
}

void Preprocessor::downsample()
{
  //downsampleUniform();
  downsampleISS();
  //downsampleHarris();
  //downsampleSIFT();
}

void Preprocessor::downsampleUniform()
{
  pcl::UniformSampling<Point> uniform_sampling;
  uniform_sampling.setInputCloud (data_.input_);
  uniform_sampling.setRadiusSearch (sampling_size_);
  uniform_sampling.filter (*data_.keypoints_);
  std::cout <<
  "Cloud total points: " <<
  data_.input_->size () <<
  "; Selected Keypoints: " <<
  data_.keypoints_->size () <<
  std::endl;
}

void Preprocessor::downsampleISS()
{
  auto iss_gamma_21_ = 0.975;
  auto iss_gamma_32_ = 0.975;
  auto iss_min_neighbors_ = 5;
  auto  iss_threads_ = 4u;

  auto iss_non_max_radius_ = 4 * resolution_;
  auto iss_normal_radius_ = 4 * resolution_;
  auto iss_border_radius_ = 1 * resolution_;
  auto iss_salient_radius_ = 6 * resolution_;

  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
  tree->setInputCloud(data_.input_);

//
// Compute keypoints
//
  pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;

  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (iss_salient_radius_);
  iss_detector.setNonMaxRadius (iss_non_max_radius_);

  iss_detector.setNormalRadius (iss_normal_radius_);
  iss_detector.setBorderRadius (iss_border_radius_);

  iss_detector.setThreshold21 (iss_gamma_21_);
  iss_detector.setThreshold32 (iss_gamma_32_);
  iss_detector.setMinNeighbors (iss_min_neighbors_);
  iss_detector.setNumberOfThreads (iss_threads_);
  iss_detector.setInputCloud (data_.input_);
  iss_detector.compute (*data_.keypoints_);
}

void Preprocessor::downsampleHarris()
{

}

void Preprocessor::downsampleSIFT()
{

}