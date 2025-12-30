cmake_minimum_required(VERSION 3.5)
project(miivii_gmsl_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  camera_info_manager
  message_generation
)

find_package(yaml-cpp REQUIRED)

find_library(MVGMSLCAMERA_NOOPENCV_LIBRARY
  NAMES mvgmslcamera_noopencv
  PATHS /opt/miivii/lib)

# Add custom message
add_message_files(FILES
  MiiviiTime.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    camera_info_manager
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /opt/miivii/include
)

add_executable(miivii_gmsl_camera_node 
  src/miivii_gmsl_camera.cpp
  src/miivii_gmsl_camera_node.cpp
)

add_dependencies(miivii_gmsl_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(miivii_gmsl_camera_node
  ${catkin_LIBRARIES}
  ${MVGMSLCAMERA_NOOPENCV_LIBRARY}
  yaml-cpp
)

target_compile_definitions(miivii_gmsl_camera_node PUBLIC MIIVII_NO_OPENCV)

install(TARGETS miivii_gmsl_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

