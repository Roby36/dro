# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/catkin_ws/ardupilot_ws/src/dro/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/catkin_ws/ardupilot_ws/src/dro/build

# Include any dependencies generated for this target.
include CMakeFiles/JControllerServer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/JControllerServer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/JControllerServer.dir/flags.make

CMakeFiles/JControllerServer.dir/DSocket.cpp.o: CMakeFiles/JControllerServer.dir/flags.make
CMakeFiles/JControllerServer.dir/DSocket.cpp.o: /root/catkin_ws/ardupilot_ws/src/dro/src/DSocket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/catkin_ws/ardupilot_ws/src/dro/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/JControllerServer.dir/DSocket.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/JControllerServer.dir/DSocket.cpp.o -c /root/catkin_ws/ardupilot_ws/src/dro/src/DSocket.cpp

CMakeFiles/JControllerServer.dir/DSocket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/JControllerServer.dir/DSocket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/catkin_ws/ardupilot_ws/src/dro/src/DSocket.cpp > CMakeFiles/JControllerServer.dir/DSocket.cpp.i

CMakeFiles/JControllerServer.dir/DSocket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/JControllerServer.dir/DSocket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/catkin_ws/ardupilot_ws/src/dro/src/DSocket.cpp -o CMakeFiles/JControllerServer.dir/DSocket.cpp.s

CMakeFiles/JControllerServer.dir/DSocket.cpp.o.requires:

.PHONY : CMakeFiles/JControllerServer.dir/DSocket.cpp.o.requires

CMakeFiles/JControllerServer.dir/DSocket.cpp.o.provides: CMakeFiles/JControllerServer.dir/DSocket.cpp.o.requires
	$(MAKE) -f CMakeFiles/JControllerServer.dir/build.make CMakeFiles/JControllerServer.dir/DSocket.cpp.o.provides.build
.PHONY : CMakeFiles/JControllerServer.dir/DSocket.cpp.o.provides

CMakeFiles/JControllerServer.dir/DSocket.cpp.o.provides.build: CMakeFiles/JControllerServer.dir/DSocket.cpp.o


CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o: CMakeFiles/JControllerServer.dir/flags.make
CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o: /root/catkin_ws/ardupilot_ws/src/dro/src/JControllerServer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/catkin_ws/ardupilot_ws/src/dro/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o -c /root/catkin_ws/ardupilot_ws/src/dro/src/JControllerServer.cpp

CMakeFiles/JControllerServer.dir/JControllerServer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/JControllerServer.dir/JControllerServer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/catkin_ws/ardupilot_ws/src/dro/src/JControllerServer.cpp > CMakeFiles/JControllerServer.dir/JControllerServer.cpp.i

CMakeFiles/JControllerServer.dir/JControllerServer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/JControllerServer.dir/JControllerServer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/catkin_ws/ardupilot_ws/src/dro/src/JControllerServer.cpp -o CMakeFiles/JControllerServer.dir/JControllerServer.cpp.s

CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o.requires:

.PHONY : CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o.requires

CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o.provides: CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o.requires
	$(MAKE) -f CMakeFiles/JControllerServer.dir/build.make CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o.provides.build
.PHONY : CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o.provides

CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o.provides.build: CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o


# Object files for target JControllerServer
JControllerServer_OBJECTS = \
"CMakeFiles/JControllerServer.dir/DSocket.cpp.o" \
"CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o"

# External object files for target JControllerServer
JControllerServer_EXTERNAL_OBJECTS =

/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: CMakeFiles/JControllerServer.dir/DSocket.cpp.o
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: CMakeFiles/JControllerServer.dir/build.make
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libtf.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libtf2_ros.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libactionlib.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libmessage_filters.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libroscpp.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libxmlrpcpp.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libtf2.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libroscpp_serialization.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/librosconsole.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/librostime.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /opt/ros/melodic/lib/libcpp_common.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libboost_system.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libpthread.so
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer: CMakeFiles/JControllerServer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/catkin_ws/ardupilot_ws/src/dro/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/JControllerServer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/JControllerServer.dir/build: /root/catkin_ws/ardupilot_ws/src/dro/devel/lib/dro/JControllerServer

.PHONY : CMakeFiles/JControllerServer.dir/build

CMakeFiles/JControllerServer.dir/requires: CMakeFiles/JControllerServer.dir/DSocket.cpp.o.requires
CMakeFiles/JControllerServer.dir/requires: CMakeFiles/JControllerServer.dir/JControllerServer.cpp.o.requires

.PHONY : CMakeFiles/JControllerServer.dir/requires

CMakeFiles/JControllerServer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/JControllerServer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/JControllerServer.dir/clean

CMakeFiles/JControllerServer.dir/depend:
	cd /root/catkin_ws/ardupilot_ws/src/dro/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/catkin_ws/ardupilot_ws/src/dro/src /root/catkin_ws/ardupilot_ws/src/dro/src /root/catkin_ws/ardupilot_ws/src/dro/build /root/catkin_ws/ardupilot_ws/src/dro/build /root/catkin_ws/ardupilot_ws/src/dro/build/CMakeFiles/JControllerServer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/JControllerServer.dir/depend

