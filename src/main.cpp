#include <ros/ros.h>

#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/recognizer.h>
#include <pcl_recognizer/visualizer.h>
#include <pcl_recognizer/config.h>

static constexpr auto MODEL_PATH = "/home/oles/mgr/datasets/willow/willow_models/object_01/3D_model.pcd";
static constexpr auto SCENE_PATH = "/home/oles/mgr/datasets/willow/willow_models/object_01/views/cloud_00000004.pcd";

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pcl_recognizer");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  Visualizer rec_vis("recognizer_visualizer");
  Preprocessor model_preprocessor("model");
  Visualizer model_vis("model_visualizer");
  Preprocessor scene_preprocessor("scene");
  Visualizer scene_vis("scene_visualizer");

  ros::Rate rate(5);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Recognizer recognizer;
  while(ros::ok())
  {
    if (Config::get().recalculate)
    {
      Config::get().recalculate = false;
      try
      {
        auto model = model_preprocessor.load(MODEL_PATH);
        auto scene = scene_preprocessor.load(SCENE_PATH);
        recognizer.setModel(model);

        Pose pose;
        auto result = recognizer.recognize(scene, pose);
        std::cout << "Found " << result << " object instances" << std::endl;

        model_vis.update(model);
        scene_vis.update(scene);

        rec_vis.renderRecognition(recognizer);
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

