cmake_minimum_required(VERSION 3.8)
project(hub_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensoryData.msg"
  "msg/SemanticData.msg"
  "srv/metacam_service/DistortionRemoval.srv"
  "srv/metacam_service/SemanticInfo.srv"
  "srv/metacam_service/PointsFusion.srv"
  "srv/semantic_map_service/StartMapBuilder.srv"
  "srv/semantic_map_service/EndMapBuilder.srv"
  "srv/semantic_map_service/EndGenSemantic.srv"
  "srv/semantic_map_service/StartGenSemantic.srv"
  DEPENDENCIES
    sensor_msgs
    nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)


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

