#include <boost/filesystem.hpp>
#include <chrono>
#include <H5Cpp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using Point = pcl::PointXYZRGBA;
using Cloud = pcl::PointCloud<Point>;
using CloudVector = std::vector<Cloud>;
using Pose = Eigen::Matrix4f;
using PoseVector = std::vector<Pose, Eigen::aligned_allocator<Pose>>;

static constexpr auto CLOUD_FILE_EXTENSION = ".pcd";
static constexpr auto POSE_FILE_EXTENSION = "_pose.csv";

class Timer
{
public:
  using hrc = std::chrono::high_resolution_clock;

  static void start()
  {
    duration(false);
  }

  static void end()
  {
    duration(true);
  }

private:
  static void duration(bool print)
  {
    static auto last = hrc::now();
    if (print)
      cout << "Timer duration: " << std::chrono::duration_cast<std::chrono::nanoseconds>(hrc::now() - last).count() <<
      endl;
    last = hrc::now();
  }
};

class Model
{
public:
  PoseVector poses;
  CloudVector clouds;

  void loadObj(string obj_fname);

  void loadPartials(string cloud_dir, string pose_dir);
private:
  Cloud loadCloud(string cloud_fname);
  Pose loadPose(string pose_fname);
};

int main(int argc, char *argv[])
{
  if (argc < 3)
    return -1;

  Model model;
  boost::filesystem::path p(argv[1]);
  try
  {
    model.loadObj("");
    //model.loadPartials(argv[1], argv[2]);
  }
  catch (const boost::filesystem::filesystem_error& ex)
  {
    std::cerr << ex.what() << '\n';
  }
  return 0;
}

void Model::loadPartials(string cloud_dir, string pose_dir)
{
  std::vector<std::string> filenames;
  if (boost::filesystem::is_directory(cloud_dir) && boost::filesystem::is_directory(pose_dir))
  {
    using dir_it = boost::filesystem::directory_iterator;
    for (auto dirent = dir_it(cloud_dir); dirent != dir_it(); dirent++)
    {
      auto extension = dirent->path().extension();
      auto filename = dirent->path().stem().string();

      if (extension != CLOUD_FILE_EXTENSION)
        continue;

      auto cloud_path = cloud_dir + '/' + filename + CLOUD_FILE_EXTENSION;
      auto pose_path = pose_dir + '/' + filename + POSE_FILE_EXTENSION;

      if (!boost::filesystem::is_regular_file(pose_path))
        continue;

      clouds.emplace_back(loadCloud(cloud_path));
      cout << "Clouds size: " << clouds.size() << endl;
      poses.emplace_back(loadPose(pose_path));
      cout << "Poses size: " << poses.size() << endl;
    }

    for (auto file : filenames)
    {
      cout << file << endl;
    }
  }
  else
    cerr << "invalid directory\n";
}

Cloud Model::loadCloud(string cloud_fname)
{
  Cloud cloud;

  if (pcl::io::loadPCDFile(cloud_fname, cloud) == -1)
    throw new std::invalid_argument("Couldn't read file " + cloud_fname);

  return cloud;
}

Pose Model::loadPose(string pose_fname)
{
  Pose pose = Pose::Identity();

  std::ifstream infile(pose_fname);
  std::string line;

  infile >> std::ws;
  for (auto row = 0u; std::getline(infile, line); row++, infile >> std::ws)
  {
    std::istringstream iss(line);
    std::string token;
    for (auto col = 0u; std::getline(iss, token, ','); col++, iss >> std::ws)
    {
      if (row < 4)
        pose(row, col) = std::stof(token);
    }
  }
  return pose;
}

void Model::loadObj(string obj_fname)
{
  constexpr auto OBJ_PATH = "/home/oles/mgr/datasets/apc/rutgers/elmers/elmers_washable_no_run_school_glue.obj";

  pcl::TextureMesh mesh7;
  pcl::io::loadOBJFile(OBJ_PATH, mesh7);

  pcl::visualization::PCLVisualizer viewer_pre7("PCL Viewer_pre7");
  viewer_pre7.addTextureMesh(mesh7, "texture", 0);
  viewer_pre7.spin();
}
