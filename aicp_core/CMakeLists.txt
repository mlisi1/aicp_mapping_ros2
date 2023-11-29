cmake_minimum_required(VERSION 3.8)

project(aicp_core)
add_compile_options(-std=c++17)
set(AICP_CORE_LIB ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

# Find system dependencies (buildtime dependencies)
find_package(ament_cmake REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(PCL REQUIRED)# COMPONENTS common io visualization)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VTK REQUIRED)


# Add include directories
include_directories(
  include
  ${libpointmatcher_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  /usr/include/vtk-9.1
)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${AICP_CORE_LIB} # libraries which will be visible outside 
#             aicpRegistration
#             aicpOverlap
#             aicpClassification
#             aicpUtils 
#   CATKIN_DEPENDS
#   DEPENDS libpointmatcher
#           PCL
#           OCTOMAP
#           OpenCV) # dependencies which will be taken outside



ament_export_dependencies(
"libpointmatcher"
"PCL"
"OCTOMAP"
"OpenCV"
"VTK"
)

ament_export_include_directories(
include
${aicp_core_INCLUDE_DIRS}
/usr/include/vtk-9.1   
)

add_definitions(${PCL_DEFINITIONS})





ament_export_libraries(
${AICP_CORE_LIB} # libraries which will be visible outside 
aicpRegistration
aicpOverlap
aicpClassification
aicpUtils        
)



  
#################
# Utils library #
#################
add_library(aicpUtils SHARED src/utils/common.cpp
                             src/utils/timing.cpp
                             src/utils/cloudIO.cpp
                             src/utils/fileIO.cpp
                             src/utils/icpMonitor.cpp
                             src/utils/filteringUtils.cpp)
target_link_libraries(aicpUtils ${libpointmatcher_LIBRARIES}
                                ${PCL_LIBRARIES})








##################
# Octree-overlap #
##################
add_library(aicpOverlap SHARED src/overlap/octrees_overlap.cpp)
target_link_libraries(aicpOverlap ${PCL_LIBRARIES}
                                  ${OCTOMAP_LIBRARIES})


#################
# Clasification #
#################
add_library(aicpClassification SHARED src/classification/svm.cpp)
target_link_libraries(aicpClassification ${OpenCV_LIBS})
                                         
add_executable(aicp_classification_main src/classification/main.cpp)
target_link_libraries(aicp_classification_main aicpClassification
                                               aicpUtils
                                               yaml-cpp)
add_executable(aicp_classification_example src/classification/example.cpp)
target_link_libraries(aicp_classification_example aicpClassification
                                                  aicpUtils
                                                  yaml-cpp)
                                                  
                                                  
################  
# Registration #
################
add_library(aicpRegistration SHARED src/registration/pointmatcher_registration.cpp
                                    src/registration/aligned_cloud.cpp
                                    src/registration/aligned_clouds_graph.cpp)
target_link_libraries(aicpRegistration ${libpointmatcher_LIBRARIES}
                                       ${PCL_LIBRARIES}
                                       yaml-cpp)

                                      
################  
# Core library #
################
add_library(${AICP_CORE_LIB} SHARED src/registration/app.cpp
                                    src/registration/yaml_configurator.cpp)
target_link_libraries(${AICP_CORE_LIB} yaml-cpp
                                       aicpRegistration
                                       aicpOverlap
                                       aicpClassification
                                       aicpUtils)


#############
# Unit test #
#############
add_executable(aicp_test test/aicp_test.cpp)
target_link_libraries(aicp_test ${AICP_CORE_LIB} ${PCL_LIBRARIES})



  


#############
## Install ##
#############
install(TARGETS ${AICP_CORE_LIB} aicpRegistration aicpUtils aicpOverlap aicpClassification
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

###############  
# Extra Tools #
###############
#add_subdirectory(src/tools)

ament_package()
