add_executable (pcl_octomap_test
  pcl_octomap_test.cpp
)

target_link_libraries (pcl_octomap_test 
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

