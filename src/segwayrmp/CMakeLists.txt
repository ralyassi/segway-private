cmake_minimum_required(VERSION 3.8)
project(segwayrmp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funroll-loops -fPIC -Wall -Wno-sign-compare -Werror=return-type -g")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(segway_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)

include_directories(
  include
)

# both tools commented
# add_executable(ChassisResponseTest
#  tools/ChassisResponseTest.cpp
# )
# ament_target_dependencies(ChassisResponseTest
#  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs segway_msgs tf2 tf2_ros
# )

# add_executable(drive_segway_sample
#  tools/drive_segway_sample.cpp
# )
# ament_target_dependencies(drive_segway_sample
#  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs segway_msgs tf2 tf2_ros
# )

add_executable(SmartCar
  src/SmartCar.cpp
  src/robot.cpp
  src/Ge_encoder_odometry.cpp
)
ament_target_dependencies(SmartCar
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs segway_msgs tf2 tf2_ros
)

# If you still have to link against a custom or system library:
target_link_libraries(SmartCar
  ${PROJECT_SOURCE_DIR}/lib/libctrl_arm64-v8a.so
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libctrl_arm64-v8a.so
  DESTINATION lib
)

install(TARGETS
  #ChassisResponseTest
  #drive_segway_sample
  SmartCar
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
