cmake_minimum_required( VERSION 2.8.3 )
project( train_control )

find_package( catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    roscpp
    rospy
)

add_message_files(
    FILES
    SingleState.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

set( message_package
    std_msgs
)

catkin_package( 
    CATKIN_DEPENDS roscpp ${message_package}
)

include_directories(
    ${catkin_INCKUDE_DIRS}
)
