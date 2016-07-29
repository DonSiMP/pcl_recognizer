#include <pcl_recognizer/denoiser.h>

#include <pcl/filters/normal_refinement.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/search/kdtree.h>

void Denoiser::removeNaNs(PreprocessedData& data)
{
  auto indices = std::vector<int>();
  pcl::removeNaNFromPointCloud(*data.input_, *data.input_, indices);
}

void Denoiser::removeGhostPoints(PreprocessedData& data)
{
  pcl::ShadowPoints<Point,Normal> sp;
  //OOOPS = NANS
  sp.setKeepOrganized(false);
  sp.setThreshold(cfg_.shadowThreshold);
  sp.setNormals(data.normals_);
  sp.setInputCloud(data.input_);
  sp.filter(*data.input_);
}

void Denoiser::refineNormals(PreprocessedData& data)
{
  std::vector<std::vector<int>> neigh_indices;
  std::vector<std::vector<float>> neigh_sqr_distances;

  pcl::search::KdTree<Point> search;
  search.setInputCloud(data.input_);
  search.radiusSearch(*data.input_,
                        std::vector<int> (),
                        cfg_.refineNormalsRadius,
                        neigh_indices,
                        neigh_sqr_distances);

  pcl::NormalRefinement<Normal> nr(neigh_indices, neigh_sqr_distances);
  nr.setConvergenceThreshold(cfg_.refineNormalsThreshold);
  nr.setInputCloud(data.normals_);
  nr.filter(*data.normals_);
}
