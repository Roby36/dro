cmake_minimum_required(VERSION 3.0.2)

project(dro)

add_compile_options(-std=c++11 -I./net/client -I./net/server -I./net/common )

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs mavros_msgs nav_msgs tf)

find_package( PkgConfig REQUIRED ) ## keyboard_mover
pkg_check_modules( ncurses++ REQUIRED ncurses++ ) ## keyboard_mover

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(JController ./handlers/SubHandler.cpp ./handlers/PubHandler.cpp ./transforms/TransformBroadcaster2.cpp ./nav/VelController.cpp ./testing/ServiceHandler.cpp ./testing/JController.cpp ./testing/JCUnitTest.cpp)
target_link_libraries(JController ${catkin_LIBRARIES} ncurses)
target_link_libraries(JController ${ncurses++_LIBRARIES}) ## keyboard_mover

# add_executable(JControllerServer JController.cpp JControllerServer.cpp ./net/net.a)
# find_library(NET net ./net/net.a)
# target_link_libraries(JControllerServer ${NET})
# target_link_libraries(JControllerServer ${catkin_LIBRARIES} ncurses)
# target_link_libraries(JControllerServer ${ncurses++_LIBRARIES}) ## keyboard_mover

add_executable(TransformNode ./handlers/SubHandler.cpp ./transforms/TransformBroadcaster2.cpp ./transforms/TransformNode.cpp)
target_link_libraries(TransformNode ${catkin_LIBRARIES})

add_dependencies(JController ${catkin_EXPORTED_TARGETS} JController) ## keyboard_mover
# add_dependencies(JControllerServer ${catkin_EXPORTED_TARGETS} JControllerServer) ## keyboard_mover

