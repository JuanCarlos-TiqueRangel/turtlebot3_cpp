cmake_minimum_required(VERSION 3.5)
project(turtlebot3_cpp)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
		include
		${EIGEN3_INCLUDE_DIRS})

set(executable_name turtlebot3_mpc)
set(library_name ${executable_name}_core)
set(dependencies
  rclcpp
  std_msgs
	nav_msgs
	sensor_msgs
  geometry_msgs
	Eigen3)

add_library(${library_name} SHARED
  src/turtlebot3_get_data.cpp)
ament_target_dependencies(${library_name}
  ${dependencies})

add_executable(${executable_name}
  src/turtlebot3_get_data_node.cpp)
target_link_libraries(${executable_name}
  ${library_name})
ament_target_dependencies(${executable_name}
  ${dependencies})

install(TARGETS
  ${executable_name}
  ${library_name}

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
