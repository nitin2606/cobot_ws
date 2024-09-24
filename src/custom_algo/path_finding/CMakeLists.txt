cmake_minimum_required(VERSION 3.8)
project(path_finding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(PLANNER_SOURCES
  planner/dijkstra.cpp
  planner/waypoint_navigation.cpp
)


add_executable(occupancy_grid_subs src/occupancy_grid_subs.cpp)
ament_target_dependencies(occupancy_grid_subs rclcpp nav_msgs geometry_msgs sensor_msgs)

add_executable(map_follower src/map_follower.cpp ${PLANNER_SOURCES})
ament_target_dependencies(map_follower rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs)


install(DIRECTORY planner/
        DESTINATION share/${PROJECT_NAME}/planner)




install(TARGETS
  occupancy_grid_subs
  map_follower
  
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
