cmake_minimum_required(VERSION 3.8)
project(video_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc objdetect)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(face_recognition_node src/face_recognition_node.cpp)
target_compile_features(face_recognition_node PUBLIC cxx_std_17)
target_include_directories(face_recognition_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(face_recognition_node ${OpenCV_LIBS})
ament_target_dependencies(
  face_recognition_node
  cv_bridge
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS face_recognition_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
