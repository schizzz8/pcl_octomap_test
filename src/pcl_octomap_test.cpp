#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

#include <octomap/OcTree.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

typedef pcl::visualization::PCLVisualizer Visualizer;

void showCubes(double side,
               const PointCloud::Ptr& occ_cloud,
               const PointCloud::Ptr& fre_cloud,
               Visualizer::Ptr& viewer);

int main(int argc, char** argv){

  PointCloud::Ptr cloud (new PointCloud);
  std::cerr << "Loading file " << argv[1] << std::endl;
  pcl::io::loadPCDFile<Point> (argv[1], *cloud);

  octomap::Pointcloud scan;
  for(const Point& pt : cloud->points)
    scan.push_back(pt.x,pt.y,pt.z);

  octomap::OcTree octree(0.05);
  octree.insertPointCloud(scan,octomap::point3d(0,0,0),octomap::pose6d(0,0,0.6,0,0,0));

  Visualizer::Ptr viewer (new Visualizer ("Octree Viewer"));

  bool showAll = true;
  int cnt_occupied_thres, cnt_occupied, cnt_free_thres, cnt_free;
  PointCloud::Ptr occ_voxel_cloud (new PointCloud);
  PointCloud::Ptr fre_voxel_cloud (new PointCloud);
  octomap::point3d p;
  Point pt;
  for(octomap::OcTree::tree_iterator it = octree.begin_tree(octree.getTreeDepth()),end=octree.end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      if (octree.isNodeOccupied(*it)){ // occupied voxels
        if (octree.isNodeAtThreshold(*it))
          ++cnt_occupied_thres;
        else
          ++cnt_occupied;

        p = it.getCoordinate();
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        occ_voxel_cloud->points.push_back(pt);
      }
      else if (showAll) { // freespace voxels
        if (octree.isNodeAtThreshold(*it))
          ++cnt_free_thres;
        else
          ++cnt_free;
        p = it.getCoordinate();
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        fre_voxel_cloud->points.push_back(pt);
      }
    }
  }

  showCubes(0.05,occ_voxel_cloud,fre_voxel_cloud,viewer);
  viewer->addCoordinateSystem (0.5);

  while (!viewer->wasStopped()){
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }


//  std::string data(argv[1]);
//  data = data.substr(0,data.find_first_of("."));
//  data = data + ".bt";
//  octree.writeBinary(data);
//  std::cerr << std::endl;

  return 0;
}

void showCubes(double s,
               const PointCloud::Ptr& occ_cloud,
               const PointCloud::Ptr& fre_cloud,
               Visualizer::Ptr& viewer){

  if(occ_cloud->points.size()){
    // process occ cloud
    vtkSmartPointer<vtkAppendPolyData> occ_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
    for (size_t i = 0; i < occ_cloud->points.size (); i++) {
      double x = occ_cloud->points[i].x;
      double y = occ_cloud->points[i].y;
      double z = occ_cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> occ_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      occ_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      occ_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      occ_append_filter->AddInput (occ_cube_source->GetOutput ());
#else
      occ_append_filter->AddInputData (occ_cube_source->GetOutput ());
#endif
    }

    // Remove duplicate points
    vtkSmartPointer<vtkCleanPolyData> occ_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New ();
    occ_clean_filter->SetInputConnection (occ_append_filter->GetOutputPort ());
    occ_clean_filter->Update ();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> occ_multi_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    occ_multi_mapper->SetInputConnection (occ_clean_filter->GetOutputPort ());
    vtkSmartPointer<vtkActor> occ_multi_actor = vtkSmartPointer<vtkActor>::New ();
    occ_multi_actor->SetMapper (occ_multi_mapper);
    occ_multi_actor->GetProperty ()->SetColor (1.0, 0.0, 0.0);
    occ_multi_actor->GetProperty ()->SetAmbient (1.0);
    occ_multi_actor->GetProperty ()->SetLineWidth (1);
    occ_multi_actor->GetProperty ()->EdgeVisibilityOn ();
    occ_multi_actor->GetProperty ()->SetOpacity (1.0);
    occ_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

    // Add the actor to the scene
    viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (occ_multi_actor);
  }

  if(fre_cloud->points.size()){
    // process fre cloud
    vtkSmartPointer<vtkAppendPolyData> fre_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
    for (size_t i = 0; i < fre_cloud->points.size (); i++) {
      double x = fre_cloud->points[i].x;
      double y = fre_cloud->points[i].y;
      double z = fre_cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> fre_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      fre_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      fre_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      fre_append_filter->AddInput (fre_cube_source->GetOutput ());
#else
      fre_append_filter->AddInputData (fre_cube_source->GetOutput ());
#endif
    }

    // Remove duplicate points
    vtkSmartPointer<vtkCleanPolyData> fre_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New ();
    fre_clean_filter->SetInputConnection (fre_append_filter->GetOutputPort ());
    fre_clean_filter->Update ();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> fre_multi_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    fre_multi_mapper->SetInputConnection (fre_clean_filter->GetOutputPort ());
    vtkSmartPointer<vtkActor> fre_multi_actor = vtkSmartPointer<vtkActor>::New ();
    fre_multi_actor->SetMapper (fre_multi_mapper);
    fre_multi_actor->GetProperty ()->SetColor (0.0, 1.0, 0.0);
    fre_multi_actor->GetProperty ()->SetAmbient (1.0);
    fre_multi_actor->GetProperty ()->SetLineWidth (1);
    fre_multi_actor->GetProperty ()->EdgeVisibilityOn ();
    fre_multi_actor->GetProperty ()->SetOpacity (1.0);
    fre_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

    // Add the actor to the scene
    viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (fre_multi_actor);
  }

  // Render and interact
  viewer->getRenderWindow ()->Render ();
}
