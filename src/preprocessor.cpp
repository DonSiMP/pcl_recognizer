#include <pcl_recognizer/preprocessor.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl_recognizer/config.h>
#include <pcl_recognizer/utils.h>
#include <pcl/search/kdtree.h>

PreprocessedData
Preprocessor::load(const std::string& pcd_path)
{
  if(Config::shouldRun(Config::Load))
  {
    loadPCD(pcd_path);
    computeResolution();
    std::cout << "Cloud resolution " << data_.input_resolution_ << std::endl;
  }
  preprocess();

  std::cout << "Descr " << data_.descriptors_->size() << std::endl;
  return data_;
}

PreprocessedData
Preprocessor::load(const std::string& pcd_path, const std::string& indices_path, const std::string& pose_path)
{
  if(Config::shouldRun(Config::Load))
  {
    loadPCD(pcd_path);
    loadIndices(indices_path);
    loadPose(pose_path);
    computeResolution();
    std::cout << "Cloud resolution " << data_.input_resolution_ << std::endl;
  }
  preprocess();

  return data_;
}

void Preprocessor::loadPCD(const std::string& pcd_path)
{
  data_.reset();

  pcl::PCLPointCloud2 cloud2;
  if (pcl::io::loadPCDFile(pcd_path, cloud2) == -1 || cloud2.width == 0)
    throw std::runtime_error("Failed to load model from " + pcd_path);
  pcl::fromPCLPointCloud2(cloud2, *data_.input_);

  auto fields = cloud2.fields;
  normals_loaded = std::any_of(std::begin(fields), std::end(fields),
                               [](const pcl::PCLPointField& field)
                               { return field.name == "normal_x"; });
  if(normals_loaded)
    pcl::fromPCLPointCloud2(cloud2, *data_.normals_);
}

void Preprocessor::loadIndices(const std::string& indices_path)
{
  std::ifstream file(indices_path);
  pcl::PointIndicesPtr pointIndices(new pcl::PointIndices);
  int index;
  while (file >> index)
    pointIndices->indices.push_back(index);
  pcl::ExtractIndices<Point> extractIndices;
  extractIndices.setInputCloud(data_.input_);
  extractIndices.setIndices(pointIndices);
  extractIndices.filter(*data_.input_);
}

void Preprocessor::loadPose(const std::string& pose_path)
{
}

void Preprocessor::preprocess()
{
  denoiser.cleanOnInput(data_);

  if(!normals_loaded && Config::shouldRun(Config::Normals))
    describer.computeNormals(data_);

  if(!data_.normals_->empty())
    denoiser.cleanWithNormals(data_);

  if(Config::shouldRun(Config::Keypoints))
    downsampler.computeKeypoints(data_);

  if(Config::shouldRun(Config::Descriptors))
  {
    describer.computeReferenceFrames(data_);
    describer.computeDescriptors(data_);
  }
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
      data_.input_resolution_ += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }

  if (n_points != 0)
    data_.input_resolution_ /= n_points;
}