#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iterator>
#include <algorithm>

using Point = pcl::PointXYZRGBA;
using Cloud = pcl::PointCloud<Point>;

v


int main(int argc, char *argv[])
{
  boost::filesystem::path p (argv[1]);
  try
  {
    if (boost::filesystem::exists(p))  // does p actually exist?
    {
      if (boost::filesystem::is_regular_file(p))  // is p a regular file?
        std::cout << p << " size is " << boost::filesystem::file_size(p) << '\n';
      else
        std::cout << p << " exists, but is not a regular file\n";
    }
    else
      std::cout << p << " does not exist\n";
  }

  catch (const boost::filesystem::filesystem_error &ex)
  {
    std::cerr << ex.what() << '\n';
  }
  return 0;
}