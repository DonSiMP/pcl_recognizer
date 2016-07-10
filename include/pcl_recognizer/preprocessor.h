#ifndef PCL_RECOGNIZE_PREPROCESSOR_H
#define PCL_RECOGNIZE_PREPROCESSOR_H

#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_recognizer/ParamsConfig.h>

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

struct PreprocessedData {
  Cloud::Ptr input_;
  Cloud::Ptr keypoints_;
  NormalCloud::Ptr normals_;
  DescriptorCloud::Ptr descriptors_;
  RFCloud::Ptr rf_;

  void reset() {
    input_.reset(new Cloud());
    keypoints_.reset(new Cloud());
    normals_.reset(new NormalCloud());
    descriptors_.reset(new DescriptorCloud());
    rf_.reset(new RFCloud());
  }
};

class Preprocessor {
public:
  enum class ParamContext { Model, Scene };

  PreprocessedData load(std::string file_name);
  void reconfigure(pcl_recognizer::ParamsConfig &config, ParamContext ctx);

private:
  PreprocessedData data_;

  // Algorithm params
  float descr_rad_ = 0.02f;
  float rf_rad_ = 0.015f;
  float sampling_size_ = 0.01f; // 0.03f;

  // Resolution
  bool use_cloud_resolution_ = true;
  double resolution_ = .0;

  void preprocess();
  void computeNormals();
  void computeDescriptors();
  void computeReferenceFrames();
  void computeResolution();

  void downsample();
  void downsampleUniform();
  void downsampleISS();
  void downsampleHarris();
  void downsampleSIFT();
};

#endif // PCL_RECOGNIZE_PREPROCESSOR_H
