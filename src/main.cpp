#include <ros/ros.h>

#include <iomanip>

#include <pcl_recognizer/preprocessor.h>
#include <pcl_recognizer/recognizer.h>
#include <pcl_recognizer/visualizer.h>
#include <pcl_recognizer/config.h>

static constexpr auto VIEWS_COUNT = 5;//37;
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
        recognizer.reset();
        for(auto view_id = 0; !Config::get().use_full_model && view_id < VIEWS_COUNT; ++view_id)
        {
          std::stringstream ss;
          ss << std::setfill('0') << std::setw(8) << view_id;
          const auto view_id_str = ss.str();
          recognizer.addModel(model_preprocessor.load(
              Config::get().model_path + "/views/cloud_" + view_id_str + ".pcd",
              Config::get().model_path + "/views/object_indices_" + view_id_str + ".txt",
              Config::get().model_path + "/views/pose_" + view_id_str + ".txt")
          );
        }
        auto model = model_preprocessor.load(Config::get().model_path + "3D_model.pcd");
        auto scene = scene_preprocessor.load(Config::get().scene_path);
        if(Config::get().use_full_model)
          recognizer.addModel(model);

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

