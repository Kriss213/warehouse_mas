cmake_minimum_required(VERSION 3.8)
project(warehouse_mas)

# Define the library name
set(lib_name ${PROJECT_NAME}_core)

# Add compiler options for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# If additional dependencies are required, add them below:
# find_package(<dependency> REQUIRED)

# Check if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python executables
ament_python_install_package(${PROJECT_NAME})
#ament_python_install_module(scripts/classes.py)

# Install the system manager script
install(
  PROGRAMS
    scripts/system_manager/system_manager.py
    scripts/loader_manager.py
    scripts/robot_manager.py
    scripts/loader.py
    scripts/robot.py
    scripts/unloading_point.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# Install the launch directory
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

# install server files
install(
  DIRECTORY
    scripts/system_manager
  DESTINATION
    share/${PROJECT_NAME}/
)

# Include the "include" directory for header files
include_directories(include)


# Finalize the package
ament_package()
