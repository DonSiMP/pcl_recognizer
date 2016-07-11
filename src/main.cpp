#include <ros/ros.h>

#include "pcl_recognizer/preprocessor.h"
#include "pcl_recognizer/recognizer.h"
#include "pcl_recognizer/visualizer.h"
#include "pcl_recognizer/RecognizerConfig.h"

static constexpr auto MODEL_PATH = "/home/oles/mgr/datasets/willow/willow_models/object_01/3D_model.pcd";
static constexpr auto SCENE_PATH = "/home/oles/mgr/datasets/willow/willow_models/object_01/views/cloud_00000000.pcd";

pcl_recognizer::RecognizerConfig g_config_;

void params_cb(pcl_recognizer::RecognizerConfig &config, uint32_t level)
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

  dynamic_reconfigure::Server<pcl_recognizer::RecognizerConfig> server;
  dynamic_reconfigure::Server<pcl_recognizer::RecognizerConfig>::CallbackType params_server_cb;
  params_server_cb = boost::bind(&params_cb, _1, _2);
  server.setCallback(params_server_cb);

  Preprocessor model_preprocessor("model");
  Visualizer model_vis("model_visualizer");
  Preprocessor scene_preprocessor("scene");
  Visualizer scene_vis("scene_visualizer");
  Visualizer rec_vis("recognizer_visualizer");

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
        auto model = model_preprocessor.load(MODEL_PATH);

        Recognizer recognizer;
        recognizer.setModel(model);

        auto scene = scene_preprocessor.load(SCENE_PATH);
        Pose pose;
        auto result = recognizer.recognize(scene, pose);
        std::cout << "Found " << result << " object instances" << std::endl;

        model_vis.update(model);
        scene_vis.update(scene);
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

    model_vis.spin();
    scene_vis.spin();
    rate.sleep();
  }
  return 0;
}

