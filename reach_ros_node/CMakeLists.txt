cmake_minimum_required(VERSION 2.8.3)
project(reach_ros_node)

find_package(catkin REQUIRED COMPONENTS roslint message_generation)

add_message_files(
    FILES
    ReachFixStatus.msg
)

catkin_python_setup()

generate_messages()

catkin_package(
    CATKIN_DEPENDS message_runtime
)


install(PROGRAMS
   scripts/nmea_tcp_driver
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Check package for pep8 style, add a test to fail on violations.
roslint_python()
roslint_add_test()
