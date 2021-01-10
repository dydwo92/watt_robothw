cmake_minimum_required(VERSION 3.0.2)
project(watt_robothw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  controller_manager
)

find_package(Threads REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/innfos
  include/zltech
)

## zltech node
add_executable(zltech_hw
  src/zltech_hw_node.cpp
  src/zltech/zltech_hw_driver.cpp
  src/zltech/zltech_can_driver.cpp
  src/zltech/CANOpen.c
  src/zltech/CANOpen_hw_appl.c
)
target_link_libraries(zltech_hw ${catkin_LIBRARIES})
target_link_libraries(zltech_hw ${CMAKE_THREAD_LIBS_INIT})

## innfos node
add_executable(innfos_hw
  src/innfos_hw_node.cpp
  src/innfos/innfos_hw_driver.cpp
  src/innfos/innfos_can_driver.cpp
  src/innfos/innfos_can_interface.c
)
target_link_libraries(innfos_hw ${catkin_LIBRARIES})
target_link_libraries(innfos_hw ${CMAKE_THREAD_LIBS_INIT})
