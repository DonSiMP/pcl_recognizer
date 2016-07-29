#ifndef PCL_RECOGNIZER_DESCRIBER_H
#define PCL_RECOGNIZER_DESCRIBER_H

#include <dynamic_reconfigure/server.h>
#include <pcl_recognizer/preprocessed_data.h>
#include <pcl_recognizer/DescriptorConfig.h>

class Describer
{
public:
  Describer(std::string name = "Describer");

  void computeNormals(PreprocessedData& data);
  void computeReferenceFrames(PreprocessedData& data);
  void computeDescriptors(PreprocessedData& data);

private:
  pcl_recognizer::DescriptorConfig descriptor_cfg_;
  dynamic_reconfigure::Server<pcl_recognizer::DescriptorConfig> descriptor_srv_;
  void descriptor_cb(pcl_recognizer::DescriptorConfig &config, uint32_t level) { descriptor_cfg_ = config; }

  void computeNormalsOMP(PreprocessedData& data);
  void computeNormalsINT(PreprocessedData& data);
  void computeNormalsMLS(PreprocessedData& data);
};


#endif //PCL_RECOGNIZER_DESCRIBER_H
