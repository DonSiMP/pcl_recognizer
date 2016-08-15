#ifndef PCL_RECOGNIZER_PREPROCESSED_DATA_H
#define PCL_RECOGNIZER_PREPROCESSED_DATA_H

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

using pcl::Normal;
using Point = pcl::PointXYZRGBA;
using Cloud = pcl::PointCloud<Point>;
using NormalCloud = pcl::PointCloud<Normal>;
using Pose = Eigen::Matrix4f;
using PosePtr = boost::shared_ptr<Pose>;
using PoseVector = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
using Descriptor = pcl::SHOT1344;
using DescriptorCloud = pcl::PointCloud<Descriptor>;
using pcl::ReferenceFrame;
using RFCloud = pcl::PointCloud<ReferenceFrame>;
using Indices = std::vector<int>;
using pcl::IndicesPtr;

struct PreprocessedData {
  Cloud::Ptr input_;
  Cloud::Ptr keypoints_;
  IndicesPtr keypoint_idxes_;
  NormalCloud::Ptr normals_;
  DescriptorCloud::Ptr descriptors_;
  RFCloud::Ptr rf_;
  PosePtr pose_;
  double input_resolution_ = .0;

  void reset() {
    input_.reset(new Cloud());
    keypoints_.reset(new Cloud());
    keypoint_idxes_.reset(new std::vector<int>());
    normals_.reset(new NormalCloud());
    descriptors_.reset(new DescriptorCloud());
    rf_.reset(new RFCloud());
    pose_.reset(new Pose(Pose::Identity()));
  }

  Cloud::Ptr getKeypointCloud()
  {
    if(keypoint_idxes_->empty())
      return keypoints_;

    Cloud::Ptr keypoints(new Cloud());
    pcl::ExtractIndices<Point> extractor;

    extractor.setInputCloud(input_);
    extractor.setIndices(keypoint_idxes_);
    extractor.filter(*keypoints);

    return keypoints;
  }

};


#endif //PCL_RECOGNIZER_PREPROCESSED_DATA_H
