cmake_minimum_required(VERSION 3.5)
project(package_name)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Targets
ament_auto_add_library(package_name_node_component SHARED
  src/package_name_node/package_name.cpp
  src/package_name_node/package_name_node.cpp
)

rclcpp_components_register_node(package_name_node_component
  PLUGIN "package_name::PackageNameNode"
  EXECUTABLE package_name_node
)

## Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
