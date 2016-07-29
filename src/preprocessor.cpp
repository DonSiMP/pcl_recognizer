#include <pcl_recognizer/preprocessor.h>

#include <pcl/features/board.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl_recognizer/config.h>
#include <pcl_recognizer/utils.h>
#include <pcl_recognizer/KeypointConfig.h>

PreprocessedData Preprocessor::load(std::string file_name)
{
  if(Config::shouldRun(Config::Load))
  {
    data_.reset();

    pcl::PCLPointCloud2 cloud2;
    if (pcl::io::loadPCDFile(file_name, cloud2) == -1 || cloud2.width == 0)
      throw std::runtime_error("Failed to load model from " + file_name);
    pcl::fromPCLPointCloud2(cloud2, *data_.input_);

    auto fields = cloud2.fields;
    normals_loaded = std::any_of(std::begin(fields), std::end(fields),
                                   [](const pcl::PCLPointField& field)
                                   { return field.name == "normal_x"; });
    if(normals_loaded)
      pcl::fromPCLPointCloud2(cloud2, *data_.normals_);
  }
  preprocess();

  return data_;
}

void Preprocessor::preprocess()
{
  if(!normals_loaded && Config::shouldRun(Config::Normals))
    computeNormals();

  if(Config::shouldRun(Config::Keypoints))
  {
    if (keypoint_cfg_.iss_use_resolution)
    {
      computeResolution();
      std::cout << "Cloud resolution " << resolution_ << std::endl;
    }
    downsample();
  }

  if(Config::shouldRun(Config::Descriptors))
  {
    computeReferenceFrames();
    computeDescriptors();
  }
}

void Preprocessor::computeNormals()
{
  Timer::Scoped timer("Normals");

  if(descriptor_cfg_.normal_method == 0)
    computeNormalsOMP();
  else if(descriptor_cfg_.normal_method == 1)
    computeNormalsINT();
  else
    computeNormalsMLS();
}


void Preprocessor::computeNormalsOMP()
{
  pcl::NormalEstimationOMP<Point, Normal> norm_est(10);
  norm_est.setKSearch (descriptor_cfg_.normal_ksize);
  norm_est.setRadiusSearch(descriptor_cfg_.normal_rad);
  norm_est.setInputCloud (data_.input_);
  norm_est.compute (*data_.normals_);
}

void Preprocessor::computeNormalsINT()
{
  pcl::IntegralImageNormalEstimation<Point, Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(descriptor_cfg_.normal_int_maxdepth);
  ne.setNormalSmoothingSize(descriptor_cfg_.normal_int_smoothing);
  ne.setInputCloud(data_.input_);
  ne.compute(*data_.normals_);
}

void Preprocessor::computeNormalsMLS()
{
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
  pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;
  pcl::MovingLeastSquares<Point, pcl::PointXYZRGBNormal> mls;
  mls.setComputeNormals (true);

  mls.setInputCloud (data_.input_);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (descriptor_cfg_.normal_rad);

  mls.process(mls_points);

  pcl::copyPointCloud(mls_points, *data_.input_);
  pcl::copyPointCloud(mls_points, *data_.normals_);
}

void Preprocessor::computeResolution()
{
  Timer::Scoped timer("Resolution");

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
  Timer::Scoped timer("Keypoints");

  if(keypoint_cfg_.method == 0)
    downsampleUniform();
  else if(keypoint_cfg_.method == 1)
    downsampleISS();
  else if(keypoint_cfg_.method == 2)
    downsampleSIFT();
  else
    downsampleHarris();

  std::cout <<
  "Cloud total points: " <<
  data_.input_->size () <<
  "; Selected Keypoints: " <<
  data_.keypoints_->size () <<
  std::endl;
}

void Preprocessor::downsampleUniform()
{
  pcl::UniformSampling<Point> uniform_sampling;
  uniform_sampling.setInputCloud (data_.input_);
  uniform_sampling.setRadiusSearch (keypoint_cfg_.uniform_radius);
  uniform_sampling.filter (*data_.keypoints_);
}

void Preprocessor::downsampleISS()
{
  auto iss_gamma_21_ = 0.975;
  auto iss_gamma_32_ = 0.975;
  auto iss_min_neighbors_ = 5;
  auto iss_threads_ = 4u;

  auto iss_non_max_radius_ = keypoint_cfg_.iss_non_max_radius;//4 * resolution_;
  auto iss_border_radius_ = keypoint_cfg_.iss_border_radius;//1 * resolution_;
  auto iss_salient_radius_ = keypoint_cfg_.iss_salient_radius;//6 * resolution_;

  if(keypoint_cfg_.iss_use_resolution)
  {
    iss_non_max_radius_ = 8 * resolution_;
    iss_border_radius_ = resolution_;
    iss_salient_radius_ = 6 * resolution_;
  }

  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
  tree->setInputCloud(data_.input_);

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
  iss_detector.setInputCloud (data_.input_);
  iss_detector.setNormals (data_.normals_);
  iss_detector.compute (*data_.keypoints_);

  auto indices = iss_detector.getKeypointsIndices();
  *data_.keypoint_idxes_ = indices->indices;
}

void Preprocessor::downsampleHarris()
{
  pcl::PointCloud<pcl::PointXYZI> output;

  pcl::HarrisKeypoint3D<Point, pcl::PointXYZI> harris_detector;
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  harris_detector.setSearchMethod(tree);
  harris_detector.setNonMaxSupression(keypoint_cfg_.harris_use_nonmaxima);
  harris_detector.setRadius(keypoint_cfg_.harris_radius);
  harris_detector.setRadiusSearch(keypoint_cfg_.harris_radius_search);
  harris_detector.setNumberOfThreads(4);
  harris_detector.setNormals(data_.normals_);
  harris_detector.setInputCloud(data_.input_);
  harris_detector.compute(output);

  pcl::copyPointCloud(output, *data_.keypoints_);
}

void Preprocessor::downsampleSIFT()
{
  pcl::SIFTKeypoint<Point, Point> sift_detector;
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  sift_detector.setSearchMethod(tree);
  sift_detector.setScales (keypoint_cfg_.sift_min_scale,
                           keypoint_cfg_.sift_nr_octaves,
                           keypoint_cfg_.sift_nr_scales_per_octave);
  sift_detector.setMinimumContrast (keypoint_cfg_.sift_min_contrast);
  sift_detector.setInputCloud(data_.input_);
  sift_detector.compute(*data_.keypoints_);
}

void Preprocessor::computeReferenceFrames()
{
  Timer::Scoped timer("Reference frames");

  pcl::BOARDLocalReferenceFrameEstimation<Point, Normal, ReferenceFrame> rf_est;
  rf_est.setFindHoles (true);
  rf_est.setRadiusSearch (descriptor_cfg_.rf_rad);

  rf_est.setInputCloud (data_.keypoints_);
  rf_est.setInputNormals (data_.normals_);
  rf_est.setSearchSurface (data_.input_);
  rf_est.compute (*data_.rf_);
}

void Preprocessor::computeDescriptors()
{
  pcl::SHOTColorEstimationOMP<Point, Normal> descr_est;
  descr_est.setRadiusSearch (descriptor_cfg_.descr_rad);
  descr_est.setNumberOfThreads(4);
  descr_est.setInputCloud (data_.keypoints_);
  descr_est.setInputNormals (data_.normals_);
  descr_est.setInputReferenceFrames(data_.rf_);
  descr_est.setSearchSurface (data_.input_);
  descr_est.compute(*data_.descriptors_);
}

Preprocessor::Preprocessor(std::string name) : keypoint_srv_(name + "_keypoints"), descriptor_srv_(name + "_descriptors")
{
  dynamic_reconfigure::Server<pcl_recognizer::KeypointConfig>::CallbackType keypoint_server_cb;
  keypoint_server_cb = boost::bind(&Preprocessor::keypoint_cb, this, _1, _2);
  keypoint_srv_.setCallback(keypoint_server_cb);

  dynamic_reconfigure::Server<pcl_recognizer::DescriptorConfig>::CallbackType descriptor_server_cb;
  descriptor_server_cb = boost::bind(&Preprocessor::descriptor_cb, this, _1, _2);
  descriptor_srv_.setCallback(descriptor_server_cb);
}
