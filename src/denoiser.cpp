#include <pcl_recognizer/denoiser.h>

#include <pcl/search/kdtree.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_recognizer/utils.h>
#include <pcl/common/centroid.h>

void Denoiser::removeNaNs(PreprocessedData& data)
{
  Timer::Scoped timer("RemoveNaNs");

  auto indices = std::vector<int>();
  pcl::removeNaNFromPointCloud(*data.input_, *data.input_, indices);

  std::cout << "Removed NaNs: " << indices.size() << std::endl;
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

  std::cout << "Removed GhostPoints: " <<sp.getRemovedIndices()->size() << std::endl;
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

  size_t removed_count = 0;

  if(cfg_.outliers_method == 0)
  {
    pcl::RadiusOutlierRemoval<Point> ror;
    // build the filter
    ror.setInputCloud(data.input_);
    ror.setRadiusSearch(cfg_.outliers_radius);
    ror.setMinNeighborsInRadius(cfg_.outliers_min_neighs);
    // apply filter
    ror.filter(*data.input_);
    removed_count = ror.getRemovedIndices()->size();
  }
  else
  {
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(data.input_);
    sor.setMeanK(cfg_.outliers_mean_k);
    sor.setStddevMulThresh(cfg_.outliers_stddev);
    sor.filter(*data.input_);
    removed_count = sor.getRemovedIndices()->size();
  }

  std::cout << "Removed Outliers: " << removed_count << std::endl;
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

  IndicesPtr inidices_to_remove(new std::vector<int>);
  const auto& cloud = *data.input_;
  std::vector<std::vector<int>> neigh_indices;
  std::vector<std::vector<float>> neigh_sqr_distances;

  pcl::search::KdTree<Point> search;
  search.setInputCloud(data.input_);
  search.radiusSearch(*data.input_,
                      std::vector<int>(),
                      cfg_.remove_edges_radius,
                      neigh_indices,
                      neigh_sqr_distances);

  Eigen::Matrix3f covariance_matrix;
  Eigen::Vector3f average_nn;
  for(size_t idx = 0; idx < data.input_->size(); idx++)
  {
    const auto& point = data.input_->at(idx);
    const auto& indices = neigh_indices.at(idx);

    unsigned point_count = 0;
    for (size_t i = 0; i < indices.size(); ++i)
    {
      if (!isFinite(cloud.at(indices.at(i))))
        continue;

      Eigen::Vector3f pt;
      pt[0] = cloud[indices[i]].x - point.x;
      pt[1] = cloud[indices[i]].y - point.y;
      pt[2] = cloud[indices[i]].z - point.z;
      average_nn += pt;

      covariance_matrix(1, 1) += pt.y() * pt.y();
      covariance_matrix(1, 2) += pt.y() * pt.z();
      covariance_matrix(2, 2) += pt.z() * pt.z();

      pt *= pt.x();
      covariance_matrix(0, 0) += pt.x();
      covariance_matrix(0, 1) += pt.y();
      covariance_matrix(0, 2) += pt.z();
      ++point_count;
    }
    covariance_matrix(1, 0) = covariance_matrix(0, 1);
    covariance_matrix(2, 0) = covariance_matrix(0, 2);
    covariance_matrix(2, 1) = covariance_matrix(1, 2);

    if(point_count > 0)
    {
      covariance_matrix /= static_cast<float>(point_count);

      average_nn /= static_cast<float>(point_count);

      float eigenval = 0;
      Eigen::Vector3f eigenvec;
      pcl::eigen33(covariance_matrix, eigenval, eigenvec);

      if(eigenval == 0 || std::abs(eigenvec.dot(average_nn)) < cfg_.edge_threshold)
      {
        inidices_to_remove->push_back(idx);
      }
    }
  }

  pcl::ExtractIndices<Point> remove(true); // Initializing with true will allow us to extract the removed indices
  remove.setInputCloud(data.input_);
  remove.setIndices(inidices_to_remove);
  remove.setNegative(false);
  remove.filter(*data.keypoints_);
  std::cout << "Removed Edges: " << inidices_to_remove->size() << std::endl;
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
