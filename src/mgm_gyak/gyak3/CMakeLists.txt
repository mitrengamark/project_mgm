cmake_minimum_required(VERSION 3.5)
project(gyak3)

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
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(rclpy REQUIRED)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/path.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_auto_package(INSTALL_TO_SHARE
  bag
  launch
  rviz
)