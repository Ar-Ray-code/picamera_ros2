cmake_minimum_required(VERSION 3.6)

project(lccv)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
#     if (NOT CMAKE_BUILD_TYPE)
#         set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
#         message(STATUS "No previous build - default to Release build")
#     endif()
# endif()

add_definitions(-D_FILE_OFFSET_BITS=64)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

IF (NOT ENABLE_COMPILE_FLAGS_FOR_TARGET)
    # On a Pi this will give us armhf or arm64.
    execute_process(COMMAND dpkg-architecture -qDEB_HOST_ARCH
        OUTPUT_VARIABLE ENABLE_COMPILE_FLAGS_FOR_TARGET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
message(STATUS "Platform: ${ENABLE_COMPILE_FLAGS_FOR_TARGET}")
if ("${ENABLE_COMPILE_FLAGS_FOR_TARGET}" STREQUAL "arm64")
    # 64-bit binaries can be fully optimised.
    add_definitions(-ftree-vectorize)
elseif ("${ENABLE_COMPILE_FLAGS_FOR_TARGET}" STREQUAL "armv8-neon")
    # Only build with 32-bit Pi 3/4 specific optimisations if requested on the command line.
    add_definitions(-mfpu=neon-fp-armv8 -ftree-vectorize)
endif()

# Source package generation setup.
set(CPACK_GENERATOR "TXZ")
set(CPACK_PACKAGE_FILE_NAME "lccv-build")
set(CPACK_SOURCE_GENERATOR "TXZ")
set(CPACK_INSTALL_SCRIPTS ${CMAKE_SOURCE_DIR}/package.cmake)
set(CPACK_SOURCE_PACKAGE_FILE_NAME "lccv-src")
set(CPACK_SOURCE_IGNORE_FILES "/\.git*;/build;")
include(CPack)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(LIBCAMERA REQUIRED libcamera)
message(STATUS "libcamera library found:")
message(STATUS "    version: ${LIBCAMERA_VERSION}")
message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")
include_directories(include ${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})

include(GNUInstallDirs)

set(SOURCES
    src/lccv.cpp
    src/libcamera_app.cpp
    src/libcamera_app_options.cpp
)
set(HEADERS
    include/lccv.hpp
    include/libcamera_app.hpp
    include/libcamera_app_options.hpp
)

# add_library(liblccv ${SOURCES} ${HEADERS})
ament_auto_add_library(liblccv SHARED ${SOURCES} ${HEADERS})
target_link_libraries(liblccv pthread ${LIBCAMERA_LINK_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS liblccv
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_package()
ament_auto_package()