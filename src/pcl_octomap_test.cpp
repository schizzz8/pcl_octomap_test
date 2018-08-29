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

octomap::Pointcloud makeSphere(){
  octomap::Pointcloud scan;
  float theta=0,phi=0;
  int m = 20;
  float theta_stepsize = 180.0 / m * M_PI/180;
  int n = 20;
  float phi_stepsize = 180.0 / n * M_PI/180;
  octomap::point3d temp;
  for(int i = 0; i < m; ++i){
    theta = i*theta_stepsize;
    for(int j = 0; j < n; ++j){
      phi = j*phi_stepsize;
      temp.x() = 0.7*cos(phi)*cos(theta);
      temp.y() = 0.7*cos(phi)*sin(theta);
      temp.z() = 0.7*sin(phi);
      scan.push_back(temp);
    }
  }
  return scan;
}

void showCubes(double side,
               const PointCloud::Ptr& occ_cloud,
               const PointCloud::Ptr& fre_cloud,
               Visualizer::Ptr& viewer);

int main(int argc, char** argv){

  //build octree
  octomap::OcTree octree(0.05);
  octomap::point3d origin(0.0,-0.025,-0.025);
  octomap::Pointcloud scan = makeSphere();
  scan.transform(octomath::Pose6D(octomath::Vector3(1.0,0.0,0.0),octomath::Quaternion(0.0,-M_PI_2,0.0)));
  octree.insertPointCloud(scan,origin);

  //add it to the pcl viewer
  Visualizer::Ptr viewer (new Visualizer ("Octree Viewer"));
  viewer->addCoordinateSystem (0.5);
  int cnt_occupied,cnt_free;
  PointCloud::Ptr occ_voxel_cloud (new PointCloud);
  PointCloud::Ptr fre_voxel_cloud (new PointCloud);
  octomap::point3d p;
  Point pt;
  for(octomap::OcTree::tree_iterator it = octree.begin_tree(octree.getTreeDepth()),end=octree.end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      if (octree.isNodeOccupied(*it)){ // occupied voxels
        ++cnt_occupied;

        p = it.getCoordinate();
        pt.x = p.x();pt.y = p.y();pt.z = p.z();
        occ_voxel_cloud->points.push_back(pt);
      } else { // freespace voxels
        ++cnt_free;

        p = it.getCoordinate();
        pt.x = p.x();pt.y = p.y();pt.z = p.z();
        fre_voxel_cloud->points.push_back(pt);
      }
    }
  }
  showCubes(0.025,occ_voxel_cloud,fre_voxel_cloud,viewer);

  //perform ray casting
  int occ=0,fre=0,unn=0;
  float s=0.025;
  std::vector<octomap::point3d> ray;
  octomap::point3d ray_o(0.5,1.0,0.0);
  octomap::point3d ray_e(0.5,0.0,0.0);
  ray.clear();
  if(octree.computeRay(ray_o,ray_e,ray)){
    std::cerr << "Voxels: " << ray.size() << std::endl;
    for(int i=0; i<ray.size(); ++i){
      octomap::OcTreeNode* n = octree.search(ray[i]);
      bool found = false;
      double value = -1;
      if(n){
        found = true;
        value = n->getOccupancy();

        if(value==0.5)
          unn++;
        if(value>0.5){
          occ++;
          break;
        }
      }
      octomap::point3d v = ray[i];
      char buffer[30];
      sprintf(buffer,"cube_%d",i);
      if(found)
        viewer->addCube(v.x()-s,v.x()+s,v.y()-s,v.y()+s,v.z()-s,v.z()+s,0.0,0.0,1.0,buffer);
      //      else
      //        viewer->addCube(v.x()-s,v.x()+s,v.y()-s,v.y()+s,v.z()-s,v.z()+s,0.0,1.0,0.0,buffer);
      std::cerr << "voxel: " << v << " - value: " << value << std::endl;
    }
  }


  Point o;
  o.x = ray_o.x();
  o.y = ray_o.y();
  o.z = ray_o.z();
  Point e;
  e.x = ray_e.x();
  e.y = ray_e.y();
  e.z = ray_e.z();
  viewer->addLine(o,e,"line");

  std::cerr << std::endl << "occ: " << occ << " - unn: " << unn << std::endl;

  while (!viewer->wasStopped()){
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

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

  if(fre_cloud->points.size() && 0){
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
