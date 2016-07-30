#include <pcl_recognizer/denoiser.h>

#include <pcl/search/kdtree.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_recognizer/utils.h>

void Denoiser::removeNaNs(PreprocessedData& data)
{
  Timer::Scoped timer("RemoveNaNs");

  auto indices = std::vector<int>();
  pcl::removeNaNFromPointCloud(*data.input_, *data.input_, indices);
}

void Denoiser::removeGhostPoints(PreprocessedData& data)
{
  Timer::Scoped timer("RemoveGhostPoints");

  pcl::ShadowPoints<Point,Normal> sp;
  //OOOPS = NANS
  sp.setKeepOrganized(false);
  sp.setThreshold(cfg_.ghost_threshold);
  sp.setNormals(data.normals_);
  sp.setInputCloud(data.input_);
  sp.filter(*data.input_);
}

void Denoiser::refineNormals(PreprocessedData& data)
{
  Timer::Scoped timer("RefineNormals");

  std::vector<std::vector<int>> neigh_indices;
  std::vector<std::vector<float>> neigh_sqr_distances;

  pcl::search::KdTree<Point> search;
  search.setInputCloud(data.input_);
  search.radiusSearch(*data.input_,
                        std::vector<int> (),
                        cfg_.refine_normals_radius,
                        neigh_indices,
                        neigh_sqr_distances);

  pcl::NormalRefinement<Normal> nr(neigh_indices, neigh_sqr_distances);
  nr.setConvergenceThreshold(cfg_.refine_normals_threshold);
  nr.setInputCloud(data.normals_);
  nr.filter(*data.normals_);
}

void Denoiser::passThrough(PreprocessedData& data)
{
  Timer::Scoped timer("PassThrough");

  pcl::PassThrough<Point> pass;
  pass.setInputCloud(data.input_);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, cfg_.max_distance);
  pass.filter(*data.input_);
}

void Denoiser::removeOutliers(PreprocessedData& data)
{
  Timer::Scoped timer("RemoveOutliers");

  if(cfg_.outliers_method == 0)
  {
    pcl::RadiusOutlierRemoval<Point> ror;
    // build the filter
    ror.setInputCloud(data.input_);
    ror.setRadiusSearch(cfg_.outliers_radius);
    ror.setMinNeighborsInRadius(cfg_.outliers_min_neighs);
    // apply filter
    ror.filter(*data.input_);
  }
  else
  {
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(data.input_);
    sor.setMeanK(cfg_.outliers_mean_k);
    sor.setStddevMulThresh(cfg_.outliers_stddev);
    sor.filter(*data.input_);
  }
}

void Denoiser::voxelize(PreprocessedData& data)
{
  Timer::Scoped timer("Voxelize");

  pcl::VoxelGrid<Point> sor;
  sor.setInputCloud(data.input_);
  auto& leaf = cfg_.voxel_leaf_size;
  sor.setLeafSize(leaf, leaf, leaf);
  sor.filter(*data.input_);
}

void Denoiser::removeEdges(PreprocessedData& data)
{
  Timer::Scoped timer("RemoveEdges");


}

void Denoiser::cleanOnInput(PreprocessedData& data)
{
  if(cfg_.use_pass_through)
    passThrough(data);

  if(cfg_.remove_NaNs)
    removeNaNs(data);

  if(cfg_.remove_edges)
    removeEdges(data);

  if(cfg_.remove_outliers)
    removeOutliers(data);

  if(cfg_.voxelize)
    voxelize(data);
}

void Denoiser::cleanWithNormals(PreprocessedData& data)
{
  if(cfg_.refine_normals)
    refineNormals(data);

  if(cfg_.remove_ghost_points)
    removeGhostPoints(data);
}
