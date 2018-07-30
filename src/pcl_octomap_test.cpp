#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <octomap/OcTree.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

int main(int argc, char** argv){

  PointCloud::Ptr cloud (new PointCloud);
  std::cerr << "Loading file " << argv[1] << std::endl;
  pcl::io::loadPCDFile<Point> (argv[1], *cloud);

  octomap::Pointcloud scan;
  for(const Point& pt : cloud->points)
    scan.push_back(pt.x,pt.y,pt.z);

  octomap::OcTree tree(0.05);
  tree.insertPointCloud(scan,octomap::point3d(0,0,0.6));

  std::string data(argv[1]);
  data = data.substr(0,data.find_first_of("."));
  data = data + ".bt";
  tree.writeBinary(data);
  std::cerr << std::endl;

  return 0;
}
