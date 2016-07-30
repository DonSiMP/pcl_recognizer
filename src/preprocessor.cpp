#include <pcl_recognizer/preprocessor.h>

#include <pcl/io/pcd_io.h>
#include <pcl_recognizer/config.h>
#include <pcl_recognizer/utils.h>
#include <pcl/search/kdtree.h>

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

    computeResolution();
    std::cout << "Cloud resolution " << data_.input_resolution_ << std::endl;
  }
  preprocess();
  denoiser.cleanOnInput(data_);

  return data_;
}

void Preprocessor::preprocess()
{
  if(!normals_loaded && Config::shouldRun(Config::Normals))
    describer.computeNormals(data_);

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