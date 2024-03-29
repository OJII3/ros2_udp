# =====================================================
# cmakeを書く手間を減らすため、ament_cmake_autoを使う
# =====================================================

cmake_minimum_required(VERSION 3.8)
project(ros2_udp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME}
  SHARED 
  src/udp_broadcaster.cpp
  src/udp_receiver.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_auto_package()
