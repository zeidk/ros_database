cmake_minimum_required(VERSION 3.8)
project(database_connector)

# pip3 install mysql-connector-python

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})


# Install Python scripts
install(PROGRAMS
  script/main.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )


ament_package()
