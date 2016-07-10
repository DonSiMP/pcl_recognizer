#include <ros/ros.h>

#include "pcl_recognizer/preprocessor.h"
#include "pcl_recognizer/recognizer.h"

#include <dynamic_reconfigure/server.h>

static constexpr auto MODEL_PATH = "/home/oles/mgr/datasets/willow/willow_models/object_01/3D_model.pcd";
static constexpr auto SCENE_PATH = "/home/oles/mgr/datasets/willow/willow_models/object_01/views/cloud_00000000.pcd";

pcl_recognizer::ParamsConfig g_config_;

void params_cb(pcl_recognizer::ParamsConfig &config, uint32_t level)
{
  if (config.recalculate)
  {
    ROS_ERROR("Recalculate Request");
    g_config_ = config;
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pcl_recognizer");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  dynamic_reconfigure::Server<pcl_recognizer::ParamsConfig> server;
  dynamic_reconfigure::Server<pcl_recognizer::ParamsConfig>::CallbackType params_server_cb;
  params_server_cb = boost::bind(&params_cb, _1, _2);
  server.setCallback(params_server_cb);

  ros::Rate rate(5);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    if (g_config_.recalculate)
    {
      g_config_.recalculate = false;
      try
      {
        Preprocessor preprocessor;
        preprocessor.reconfigure(g_config_, Preprocessor::ParamContext::Model);
        auto model = preprocessor.load(MODEL_PATH);

        Recognizer recognizer;
        recognizer.setModel(model);
        recognizer.reconfigure(g_config_);

        preprocessor.reconfigure(g_config_, Preprocessor::ParamContext::Scene);
        auto scene = preprocessor.load(SCENE_PATH);
        Pose pose;
        auto result = recognizer.recognize(scene, pose);
        std::cout << "Found " << result << " object instances" << std::endl;
      }
      catch (const std::exception& ex)
      {
        std::cerr << ex.what() << std::endl;
        return -1;
      }
      catch (...)
      {
        std::cerr << "Unknown failure occurred. Possible memory corruption" << std::endl;
        return -1;
      }
    }

    rate.sleep();
  }
  return 0;
}

