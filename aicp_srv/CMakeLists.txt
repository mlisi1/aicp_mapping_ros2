cmake_minimum_required(VERSION 3.8)

project(aicp_srv)
add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)


# Find system dependencies (buildtime dependencies)
# find_package(catkin REQUIRED COMPONENTS roscpp
#                                         message_generation)

                                        
                                        
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)


# Service files to be built
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ProcessFile.srv"
)


ament_package()
