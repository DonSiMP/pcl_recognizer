#ifndef PCL_RECOGNIZE_PREPROCESSOR_H
#define PCL_RECOGNIZE_PREPROCESSOR_H

#include <string>

#include <dynamic_reconfigure/server.h>
#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl_recognizer/KeypointConfig.h>
#include <pcl_recognizer/DescriptorConfig.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

using pcl::Normal;
using Point = pcl::PointXYZRGBA;
using Cloud = pcl::PointCloud<Point>;
using NormalCloud = pcl::PointCloud<Normal>;
using Pose = Eigen::Matrix4f;
using PoseVector = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
using Descriptor = pcl::SHOT352;
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

  void reset() {
    input_.reset(new Cloud());
    keypoints_.reset(new Cloud());
    keypoint_idxes_.reset(new std::vector<int>());
    normals_.reset(new NormalCloud());
    descriptors_.reset(new DescriptorCloud());
    rf_.reset(new RFCloud());
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

class Preprocessor {
public:
  Preprocessor(std::string name = "Preprocessor");

  PreprocessedData load(std::string file_name);
private:
  PreprocessedData data_;

  // Resolution
  bool use_cloud_resolution_ = true;
  double resolution_ = .0;

  // Dynamic reconfigure
  pcl_recognizer::KeypointConfig keypoint_cfg_;
  dynamic_reconfigure::Server<pcl_recognizer::KeypointConfig> keypoint_srv_;
  void keypoint_cb(pcl_recognizer::KeypointConfig &config, uint32_t level) { keypoint_cfg_ = config; }
  pcl_recognizer::DescriptorConfig descriptor_cfg_;
  dynamic_reconfigure::Server<pcl_recognizer::DescriptorConfig> descriptor_srv_;
  void descriptor_cb(pcl_recognizer::DescriptorConfig &config, uint32_t level) { descriptor_cfg_ = config; }

  void preprocess();

  void computeNormals();
  void computeNormalsOMP();
  void computeNormalsINT();
  void computeNormalsMLS();

  void downsample();
  void downsampleUniform();
  void downsampleISS();
  void downsampleHarris();
  void downsampleSIFT();

  void computeDescriptors();

  void computeReferenceFrames();

  void computeResolution();
};

#endif // PCL_RECOGNIZE_PREPROCESSOR_H
