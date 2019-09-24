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
    ${catkin_INCLUDE_DIRS}
)

add_executable( first_model problem/plan_1_dimension.cpp )
add_dependencies( first_model 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( first_model ${catkin_LIBRARIES})

add_executable( first_control HW01/control.cpp)
add_dependencies( first_control 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( first_control ${catkin_LIBRARIES})
