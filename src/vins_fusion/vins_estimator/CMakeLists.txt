cmake_minimum_required(VERSION 3.10)
project(vins)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs tf
  cv_bridge camera_models image_transport rdswarm_msgs rdswarm_features)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
option(RDSWARM_WITH_TENSORRT "Build the optional legacy TensorRT SuperPoint detector" OFF)
set(RDSWARM_SUPERPOINT_SOURCE src/featureTracker/superpoint_extractor_cpu.cpp)
set(RDSWARM_GPU_INCLUDES "")
set(RDSWARM_GPU_LIBRARIES "")
if(RDSWARM_WITH_TENSORRT)
  set(TENSORRT_ROOT "" CACHE PATH "TensorRT installation prefix")
  set(CUDA_ROOT "/usr/local/cuda" CACHE PATH "CUDA installation prefix")
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS ${TENSORRT_ROOT} ENV TENSORRT_ROOT
    PATH_SUFFIXES include targets/${CMAKE_LIBRARY_ARCHITECTURE}/include)
  find_library(TENSORRT_LIBRARY nvinfer HINTS ${TENSORRT_ROOT} ENV TENSORRT_ROOT
    PATH_SUFFIXES lib lib64 targets/${CMAKE_LIBRARY_ARCHITECTURE}/lib)
  find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h HINTS ${CUDA_ROOT}
    PATH_SUFFIXES include targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include)
  find_library(CUDA_RUNTIME_LIBRARY cudart HINTS ${CUDA_ROOT}
    PATH_SUFFIXES lib64 lib targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib)
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT CUDA_RUNTIME_INCLUDE_DIR OR NOT CUDA_RUNTIME_LIBRARY)
    message(FATAL_ERROR "Optional TensorRT detector requested but TensorRT/CUDA was not found")
  endif()
  add_library(rdswarm_superpoint_tensorrt src/featureTracker/superpoint_extractor_tensorrt.cpp)
  target_include_directories(rdswarm_superpoint_tensorrt PRIVATE ${TENSORRT_INCLUDE_DIR}
    ${CUDA_RUNTIME_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  target_link_libraries(rdswarm_superpoint_tensorrt PRIVATE ${TENSORRT_LIBRARY}
    ${CUDA_RUNTIME_LIBRARY} ${OpenCV_LIBS} ${catkin_LIBRARIES})
  target_compile_features(rdswarm_superpoint_tensorrt PRIVATE cxx_std_17)
  install(TARGETS rdswarm_superpoint_tensorrt LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  set(RDSWARM_GPU_INCLUDES ${TENSORRT_INCLUDE_DIR} ${CUDA_RUNTIME_INCLUDE_DIR})
  set(RDSWARM_GPU_LIBRARIES rdswarm_superpoint_tensorrt)
endif()
message(STATUS "RDSwarm VINS TensorRT enabled: ${RDSWARM_WITH_TENSORRT}")
catkin_package(LIBRARIES vins_lib CATKIN_DEPENDS roscpp rdswarm_msgs camera_models rdswarm_features)
add_library(vins_lib
  src/estimator/parameters.cpp src/estimator/estimator.cpp src/estimator/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp src/factor/projectionTwoFrameOneCamFactor.cpp
  src/factor/projectionTwoFrameTwoCamFactor.cpp src/factor/projectionOneFrameTwoCamFactor.cpp
  src/factor/marginalization_factor.cpp src/utility/utility.cpp src/utility/visualization.cpp
  src/utility/rdswarm_export.cpp src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp src/initial/initial_aligment.cpp src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp ${RDSWARM_SUPERPOINT_SOURCE}
  src/featureTracker/feature_tracker.cpp)
if(RDSWARM_WITH_TENSORRT)
  target_compile_definitions(vins_lib PRIVATE RDSWARM_WITH_TENSORRT=1)
endif()
target_include_directories(vins_lib PUBLIC src ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${RDSWARM_GPU_INCLUDES})
target_link_libraries(vins_lib PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}
  ${RDSWARM_GPU_LIBRARIES})
add_dependencies(vins_lib ${catkin_EXPORTED_TARGETS})
add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib)
add_dependencies(vins_node ${catkin_EXPORTED_TARGETS})
if(CATKIN_ENABLE_TESTING)
  add_executable(superpoint_detector_probe test/detector_probe.cpp)
  target_link_libraries(superpoint_detector_probe vins_lib)
  add_dependencies(superpoint_detector_probe ${catkin_EXPORTED_TARGETS})
  add_executable(rdswarm_export_contract_fixture test/export_contract_fixture.cpp)
  target_link_libraries(rdswarm_export_contract_fixture vins_lib)
  add_dependencies(rdswarm_export_contract_fixture ${catkin_EXPORTED_TARGETS})
endif()
install(TARGETS vins_lib vins_node ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
