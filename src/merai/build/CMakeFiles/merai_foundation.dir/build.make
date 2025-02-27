# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/abhishek/hand_control_linux/src/merai

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/abhishek/hand_control_linux/src/merai/build

# Include any dependencies generated for this target.
include CMakeFiles/merai_foundation.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/merai_foundation.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/merai_foundation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/merai_foundation.dir/flags.make

CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o: CMakeFiles/merai_foundation.dir/flags.make
CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o: /home/abhishek/hand_control_linux/src/merai/src/RAII_SharedMemory.cpp
CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o: CMakeFiles/merai_foundation.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/abhishek/hand_control_linux/src/merai/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o -MF CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o.d -o CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o -c /home/abhishek/hand_control_linux/src/merai/src/RAII_SharedMemory.cpp

CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhishek/hand_control_linux/src/merai/src/RAII_SharedMemory.cpp > CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.i

CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhishek/hand_control_linux/src/merai/src/RAII_SharedMemory.cpp -o CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.s

CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o: CMakeFiles/merai_foundation.dir/flags.make
CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o: /home/abhishek/hand_control_linux/src/merai/src/ParameterServer.cpp
CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o: CMakeFiles/merai_foundation.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/abhishek/hand_control_linux/src/merai/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o -MF CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o.d -o CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o -c /home/abhishek/hand_control_linux/src/merai/src/ParameterServer.cpp

CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhishek/hand_control_linux/src/merai/src/ParameterServer.cpp > CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.i

CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhishek/hand_control_linux/src/merai/src/ParameterServer.cpp -o CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.s

# Object files for target merai_foundation
merai_foundation_OBJECTS = \
"CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o" \
"CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o"

# External object files for target merai_foundation
merai_foundation_EXTERNAL_OBJECTS =

libmerai_foundation.a: CMakeFiles/merai_foundation.dir/src/RAII_SharedMemory.cpp.o
libmerai_foundation.a: CMakeFiles/merai_foundation.dir/src/ParameterServer.cpp.o
libmerai_foundation.a: CMakeFiles/merai_foundation.dir/build.make
libmerai_foundation.a: CMakeFiles/merai_foundation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/abhishek/hand_control_linux/src/merai/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libmerai_foundation.a"
	$(CMAKE_COMMAND) -P CMakeFiles/merai_foundation.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/merai_foundation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/merai_foundation.dir/build: libmerai_foundation.a
.PHONY : CMakeFiles/merai_foundation.dir/build

CMakeFiles/merai_foundation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/merai_foundation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/merai_foundation.dir/clean

CMakeFiles/merai_foundation.dir/depend:
	cd /home/abhishek/hand_control_linux/src/merai/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abhishek/hand_control_linux/src/merai /home/abhishek/hand_control_linux/src/merai /home/abhishek/hand_control_linux/src/merai/build /home/abhishek/hand_control_linux/src/merai/build /home/abhishek/hand_control_linux/src/merai/build/CMakeFiles/merai_foundation.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/merai_foundation.dir/depend

