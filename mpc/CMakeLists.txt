cmake_minimum_required(VERSION 3.14)
project(mpc LANGUAGES CXX CUDA)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) #-Wpedantic
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(CUDA REQUIRED)

set(CMAKE_CUDA_ARCHITECTURES 75 86 89)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.2/bin/nvcc" CACHE PATH "CUDA compiler")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/MPCGPU/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/MPCGPU/include/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/MPCGPU/GLASS)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/MPCGPU/GBD-PCG/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/MPCGPU/qdldl/include)

add_executable(robot_driver src/robot_driver_jpos.cpp)
ament_target_dependencies(robot_driver rclcpp std_msgs)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher rclcpp std_msgs)

add_executable(trajopt_node src/trajopt_node.cu)
set_target_properties(trajopt_node PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_compile_features(trajopt_node PUBLIC cxx_std_17)
ament_target_dependencies(trajopt_node rclcpp std_msgs rosgraph_msgs)
target_include_directories(trajopt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(trajopt_node ${CUDA_LIBRARIES} cublas)

# add_executable(trajopt src/trajopt.cpp)
# target_include_directories(trajopt PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(trajopt PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  robot_driver
  trajectory_publisher
  trajopt_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
