cmake_minimum_required(VERSION 3.8)
project(mirte_telemetrix_test_helpers)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mirte_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")


add_library(image_service_tester_component SHARED
  src/image_service_tester_component.cpp)
target_compile_definitions(image_service_tester_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(image_service_tester_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "mirte_msgs"
  "cv_bridge")
target_link_libraries(image_service_tester_component ${OpenCV_LIBS})
rclcpp_components_register_nodes(image_service_tester_component "mirte_telemetrix_test_helpers::ImageServiceTester")
set(node_plugins "${node_plugins}mirte_telemetrix_test_helpers::ImageServiceTester;$<TARGET_FILE:image_service_tester_component>\n")

rclcpp_components_register_node(
  image_service_tester_component
    PLUGIN "mirte_telemetrix_test_helpers::ImageServiceTester"
    EXECUTABLE image_service_tester
)

add_library(image_streamer_component SHARED
  src/image_streamer_component.cpp)
target_compile_definitions(image_streamer_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(image_streamer_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "mirte_msgs"
  "cv_bridge"
  "image_transport")
target_link_libraries(image_streamer_component ${OpenCV_LIBS})
rclcpp_components_register_nodes(image_streamer_component "mirte_telemetrix_test_helpers::ImageStreamer")
set(node_plugins "${node_plugins}mirte_telemetrix_test_helpers::ImageStreamer;$<TARGET_FILE:image_streamer_component>\n")

rclcpp_components_register_node(
  image_streamer_component
    PLUGIN "mirte_telemetrix_test_helpers::ImageStreamer"
    EXECUTABLE image_streamer
)


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  image_service_tester_component
  image_streamer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
