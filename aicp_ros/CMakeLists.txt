cmake_minimum_required(VERSION 3.8)

project(aicp_ros)
add_compile_options(-std=c++17)

# # Find system dependencies (buildtime dependencies)
# find_package(catkin REQUIRED COMPONENTS aicp_core
#                                         aicp_srv
#                                         roscpp
#                                         eigen_conversions
#                                         pcl_conversions
#                                         tf_conversions
#                                         std_srvs
#                                         std_msgs
#                                         sensor_msgs
#                                         geometry_msgs
#                                         nav_msgs)


find_package(aicp_core REQUIRED)
find_package(aicp_srv REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(eigen_conversions REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(tf_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(VTK REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)





                        
#find_package(aicp_core)
#message("++++++++++++++++++++++ ${aicp_core_LIBRARIES}") # contains all libraries and dependencies declated in catkin_package of aicp_core                                        

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS aicp_core
#                  aicp_srv
#                  roscpp
#                  eigen_conversions
#                  pcl_conversions
#                  tf_conversions
#                  std_srvs
#                  std_msgs
#                  sensor_msgs
#                  geometry_msgs
#                  nav_msgs)


ament_export_dependencies(${PROJECT_NAME} 
  "aicp_core"
  "aicp_srv"
  "rclcpp"
  "eigen_conversions"
  "pcl_conversions"
  "tf_conversions"
  "std_srvs"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "rmw"
  "tf2_ros"
  "VTK"
  "tf2_eigen"
  "tf2"
)

ament_export_include_directories(${PROJECT_NAME} 
include
)

ament_export_libraries(
  ${PROJECT_NAME} 
)

# Add include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
  ${rosidl_typesupport_cpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${aicp_srv_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${aicp_core_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} SHARED src/app_ros.cpp
                                   src/visualizer_ros.cpp
                                   src/talker_ros.cpp
                                   src/velodyne_accumulator.cpp)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node src/aicp_ros_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}
                                           ${rclcpp_LIBRARIES}
                                           ${aicp_srv_LIBRARIES}
                                           ${aicp_core_LIBRARIES}
                                           ${tf2_ros_LIBRARIES}
                                           ${tf2_LIBRARIES}
                                           ${tf2_eigen_LIBRARIES}
                                           ${rosidl_typesupport_cpp_LIBRARIES}
                                           ${sensor_msgs_LIBRARIES}
                                           ${geometry_msgs_LIBRARIES}
                                           ${nav_msgs_LIBRARIES}
                                           ${tf2_geometry_msgs_LIBRARIES}
                                           ${std_srvs_LIBRARIES}
)          
# ament_export_dependencies(${PROJECT_NAME}_node
# "rclcpp"
# )                                 


#############                                       
# Unit test #
#############
#catkin_add_gtest(aicp_test_node test/aicp_test_node.cpp)
#add_executable(aicp_test_node test/aicp_test_node.cpp)
#target_link_libraries(aicp_test_node ${PROJECT_NAME}
#                                     ${catkin_LIBRARIES}
#                                     yaml-cpp)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME} 
        FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/config
)



ament_package()

