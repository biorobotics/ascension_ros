cmake_minimum_required(VERSION 2.8.3)
project(ascension)

find_package(catkin REQUIRED  COMPONENTS roscpp tf geometry_msgs message_generation visualization_msgs)

add_message_files(
  FILES
  Sensor_poses.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)
# build ACT3DG library

include(ExternalProject)

ExternalProject_Add(act3dg
    GIT_REPOSITORY https://github.com/marcoesposito1988/ATC3DGTracker.git
    GIT_TAG fe2dad56dae6f543550c58d2dfc5bf011535b816
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    # Disable install step
    INSTALL_COMMAND ""
)

ExternalProject_Get_Property(act3dg source_dir)
set(ATC3DG_INCLUDE_DIRS ${source_dir}/include)

ExternalProject_Get_Property(act3dg binary_dir)
set(ATC3DG_LIBRARY_DIRS ${binary_dir})

# find_library(ATC3DG_LIBRARY atclib ${ATC3DG_LIBRARY_DIRS})
set(ATC3DG_LIBRARY "${ATC3DG_LIBRARY_DIRS}/libatclib.so")

catkin_package(
    CATKIN_DEPENDS roscpp tf geometry_msgs message_runtime visualization_msgs
)

###########
## Build ##
###########

include_directories(${ATC3DG_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_publisher src/ascension_node.cpp)
add_executable(${PROJECT_NAME}_transform src/transform_node.cpp)
add_executable(${PROJECT_NAME}_listener src/listener_node.cpp)
add_executable(${PROJECT_NAME}_sensor_listener src/sensor_poses_listener.cpp)

target_link_libraries(${PROJECT_NAME}_publisher
  ${catkin_LIBRARIES}
  ${ATC3DG_LIBRARY}
)

target_link_libraries(${PROJECT_NAME}_transform
 ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_listener
 ${catkin_LIBRARIES}
 )

 target_link_libraries(${PROJECT_NAME}_sensor_listener
 ${catkin_LIBRARIES}
 )


add_dependencies(${PROJECT_NAME}_publisher act3dg)
add_dependencies(${PROJECT_NAME}_transform act3dg)
add_dependencies(${PROJECT_NAME}_listener act3dg)
add_dependencies(${PROJECT_NAME}_sensor_listener act3dg)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ascension.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
