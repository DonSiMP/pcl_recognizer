#ifndef PCL_RECOGNIZE_PREPROCESSOR_H
#define PCL_RECOGNIZE_PREPROCESSOR_H

#include <string>

#include <pcl_recognizer/denoiser.h>
#include <pcl_recognizer/describer.h>
#include <pcl_recognizer/downsampler.h>
#include <pcl_recognizer/preprocessed_data.h>

class Preprocessor {
public:
  Preprocessor(std::string name = "Preprocessor") :
      denoiser(name + "_denoise"),
      downsampler(name + "_keypoints"),
      describer(name + "_descriptors") {};

  PreprocessedData load(const std::string& pcd_path);
  PreprocessedData load(const std::string& pcd_path,
                        const std::string& indices_path,
                        const std::string& pose_path);
private:
  PreprocessedData data_;
  Denoiser denoiser;
  Describer describer;
  Downsampler downsampler;
  bool normals_loaded = false;

  void loadPCD(const std::string& pcd_path);
  void loadIndices(const std::string& indices_path);
  void loadPose(const std::string& pose_path);
  void preprocess();
  void computeResolution();
};

#endif // PCL_RECOGNIZE_PREPROCESSOR_H
