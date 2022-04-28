cmake_minimum_required(VERSION 3.0.2)
project(ros_nodes)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(utils)
include_directories(../guidance)
include_directories(../models)
include_directories(../control)

add_executable(drone_control control/drone_control_node.cpp)
target_link_libraries(drone_control drone_mpc OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
add_dependencies(drone_control drone_optimal_control_generate_messages_cpp)

add_executable(drone_guidance guidance/drone_guidance_node.cpp)
target_link_libraries(drone_guidance drone_guidance_lib OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
add_dependencies(drone_guidance drone_optimal_control_generate_messages_cpp)

# add_executable(fixed_guidance guidance/fixed_guidance_node.cpp)
# target_link_libraries(fixed_guidance Eigen3::Eigen ${catkin_LIBRARIES})
# add_dependencies(fixed_guidance drone_optimal_control_generate_messages_cpp)

#add_executable(drone_navigation navigation/drone_navigation_node_old.cpp)
#target_link_libraries(drone_navigation Eigen3::Eigen ${catkin_LIBRARIES})
#add_dependencies(drone_navigation drone_optimal_control_generate_messages_cpp)

add_subdirectory(utils)