cmake_minimum_required(VERSION 3.0.2)

project(dro)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs mavros_msgs nav_msgs tf)

find_package( PkgConfig REQUIRED ) ## keyboard_mover
pkg_check_modules( ncurses++ REQUIRED ncurses++ ) ## keyboard_mover

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Dro dro.cpp)

target_link_libraries(Dro ${catkin_LIBRARIES} ncurses)
target_link_libraries(Dro ${ncurses++_LIBRARIES}) ## keyboard_mover
add_dependencies(Dro ${catkin_EXPORTED_TARGETS} Dro) ## keyboard_mover