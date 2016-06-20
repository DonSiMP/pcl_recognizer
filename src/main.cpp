#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using std::cout;
using std::endl;
using std::string;

using Point = pcl::PointXYZRGBA;
using Cloud = pcl::PointCloud<Point>;
using Pose = Eigen::Matrix4f;
using PoseVector = std::vector<Pose, Eigen::aligned_allocator<Pose>>;

void loadPartials(string cloud_dir, string pose_dir)
{
  std::vector<std::string> filenames;
  if (boost::filesystem::exists(cloud_dir) && boost::filesystem::exists(pose_dir))
  {
    if (boost::filesystem::is_directory(cloud_dir) && boost::filesystem::is_directory(pose_dir))
    {
      using boost::filesystem::directory_iterator;
      for (auto dirent = directory_iterator(cloud_dir); dirent != directory_iterator(); dirent++)
      {
        if(dirent->path().extension() == ".pcd")
          filenames.push_back(dirent->path().stem().string());
      }

      for(auto file : filenames)
        cout << file << endl;
    }
    else
      cout << cloud_dir << " exists, but is not a regular file\n";
  }
  else
    cout << cloud_dir << " does not exist\n";
}

int main(int argc, char *argv[])
{
  if (argc < 3)
    return -1;

  boost::filesystem::path p(argv[1]);
  try
  {
    loadPartials(argv[1], argv[2]);
  }

  catch (const boost::filesystem::filesystem_error &ex)
  {
    std::cerr << ex.what() << '\n';
  }
  return 0;
}