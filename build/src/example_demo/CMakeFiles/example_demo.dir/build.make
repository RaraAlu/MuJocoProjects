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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/icebot/work/MojoProjects

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/icebot/work/MojoProjects/build

# Include any dependencies generated for this target.
include src/example_demo/CMakeFiles/example_demo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/example_demo/CMakeFiles/example_demo.dir/compiler_depend.make

# Include the progress variables for this target.
include src/example_demo/CMakeFiles/example_demo.dir/progress.make

# Include the compile flags for this target's objects.
include src/example_demo/CMakeFiles/example_demo.dir/flags.make

src/example_demo/CMakeFiles/example_demo.dir/main.cpp.o: src/example_demo/CMakeFiles/example_demo.dir/flags.make
src/example_demo/CMakeFiles/example_demo.dir/main.cpp.o: /home/icebot/work/MojoProjects/src/example_demo/main.cpp
src/example_demo/CMakeFiles/example_demo.dir/main.cpp.o: src/example_demo/CMakeFiles/example_demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/icebot/work/MojoProjects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/example_demo/CMakeFiles/example_demo.dir/main.cpp.o"
	cd /home/icebot/work/MojoProjects/build/src/example_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/example_demo/CMakeFiles/example_demo.dir/main.cpp.o -MF CMakeFiles/example_demo.dir/main.cpp.o.d -o CMakeFiles/example_demo.dir/main.cpp.o -c /home/icebot/work/MojoProjects/src/example_demo/main.cpp

src/example_demo/CMakeFiles/example_demo.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/example_demo.dir/main.cpp.i"
	cd /home/icebot/work/MojoProjects/build/src/example_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/icebot/work/MojoProjects/src/example_demo/main.cpp > CMakeFiles/example_demo.dir/main.cpp.i

src/example_demo/CMakeFiles/example_demo.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/example_demo.dir/main.cpp.s"
	cd /home/icebot/work/MojoProjects/build/src/example_demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/icebot/work/MojoProjects/src/example_demo/main.cpp -o CMakeFiles/example_demo.dir/main.cpp.s

# Object files for target example_demo
example_demo_OBJECTS = \
"CMakeFiles/example_demo.dir/main.cpp.o"

# External object files for target example_demo
example_demo_EXTERNAL_OBJECTS =

bin/example_demo: src/example_demo/CMakeFiles/example_demo.dir/main.cpp.o
bin/example_demo: src/example_demo/CMakeFiles/example_demo.dir/build.make
bin/example_demo: lib/libsphere_demo.a
bin/example_demo: /usr/local/lib/libmujoco.so
bin/example_demo: lib/libmouse_controller.a
bin/example_demo: /usr/lib/x86_64-linux-gnu/libGLX.so
bin/example_demo: /usr/lib/x86_64-linux-gnu/libOpenGL.so
bin/example_demo: /usr/lib/x86_64-linux-gnu/libglfw.so.3.3
bin/example_demo: src/example_demo/CMakeFiles/example_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/icebot/work/MojoProjects/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/example_demo"
	cd /home/icebot/work/MojoProjects/build/src/example_demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_demo.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold "Copying model files to build directory"
	cd /home/icebot/work/MojoProjects/build/src/example_demo && /usr/local/bin/cmake -E copy_directory /home/icebot/work/MojoProjects/src/example_demo/model /home/icebot/work/MojoProjects/build/bin/model

# Rule to build all files generated by this target.
src/example_demo/CMakeFiles/example_demo.dir/build: bin/example_demo
.PHONY : src/example_demo/CMakeFiles/example_demo.dir/build

src/example_demo/CMakeFiles/example_demo.dir/clean:
	cd /home/icebot/work/MojoProjects/build/src/example_demo && $(CMAKE_COMMAND) -P CMakeFiles/example_demo.dir/cmake_clean.cmake
.PHONY : src/example_demo/CMakeFiles/example_demo.dir/clean

src/example_demo/CMakeFiles/example_demo.dir/depend:
	cd /home/icebot/work/MojoProjects/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/icebot/work/MojoProjects /home/icebot/work/MojoProjects/src/example_demo /home/icebot/work/MojoProjects/build /home/icebot/work/MojoProjects/build/src/example_demo /home/icebot/work/MojoProjects/build/src/example_demo/CMakeFiles/example_demo.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : src/example_demo/CMakeFiles/example_demo.dir/depend

