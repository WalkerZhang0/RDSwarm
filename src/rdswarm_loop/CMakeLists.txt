cmake_minimum_required(VERSION 3.10)
project(rdswarm_loop)
find_package(catkin REQUIRED COMPONENTS roscpp rdswarm_core rdswarm_msgs)
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp rdswarm_core rdswarm_msgs)
add_library(rdswarm_keyframe_store src/keyframe_store.cpp)
target_compile_features(rdswarm_keyframe_store PUBLIC cxx_std_17)
target_include_directories(rdswarm_keyframe_store PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(rdswarm_keyframe_store PUBLIC ${catkin_LIBRARIES})
add_dependencies(rdswarm_keyframe_store ${catkin_EXPORTED_TARGETS})
add_executable(rdswarm_loop_input_node src/loop_input_node.cpp)
target_link_libraries(rdswarm_loop_input_node rdswarm_keyframe_store)
add_dependencies(rdswarm_loop_input_node ${catkin_EXPORTED_TARGETS})
install(TARGETS rdswarm_keyframe_store rdswarm_loop_input_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
