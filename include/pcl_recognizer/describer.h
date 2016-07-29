#ifndef PCL_RECOGNIZER_DESCRIBER_H
#define PCL_RECOGNIZER_DESCRIBER_H

#include <pcl_recognizer/preprocessed_data.h>
#include <pcl_recognizer/reconfigurable.h>
#include <pcl_recognizer/DescriptorConfig.h>

class Describer : Reconfigurable<pcl_recognizer::DescriptorConfig>
{
public:
  Describer(std::string name = "Describer") : Reconfigurable(name) {};

  void computeNormals(PreprocessedData& data);
  void computeReferenceFrames(PreprocessedData& data);
  void computeDescriptors(PreprocessedData& data);

private:
  void computeNormalsOMP(PreprocessedData& data);
  void computeNormalsINT(PreprocessedData& data);
  void computeNormalsMLS(PreprocessedData& data);
};


#endif //PCL_RECOGNIZER_DESCRIBER_H
