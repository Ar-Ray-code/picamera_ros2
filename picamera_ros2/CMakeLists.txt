cmake_minimum_required(VERSION 3.5)
project(picamera_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# libcamera
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
include_directories(include ${LIBCAMERA_INCLUDE_DIRS})

# picamera_pub ======================================
set(TARGET picamera_pub)
ament_auto_add_library(${TARGET} SHARED src/${TARGET}.cpp)
rclcpp_components_register_nodes(${TARGET} "picamera_ros::PiCameraROS")
# target_link_libraries(${TARGET} -llccv)
# ===================================================================

# picamera_pub_exec ======================================
ament_auto_add_executable(${TARGET}_exec src/${TARGET}.cpp)
# target_link_libraries(${TARGET}_exec -llccv)
# ===================================================================

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()