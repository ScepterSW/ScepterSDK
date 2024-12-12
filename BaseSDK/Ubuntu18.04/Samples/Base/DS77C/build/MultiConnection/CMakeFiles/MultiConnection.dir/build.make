# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build

# Include any dependencies generated for this target.
include MultiConnection/CMakeFiles/MultiConnection.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include MultiConnection/CMakeFiles/MultiConnection.dir/compiler_depend.make

# Include the progress variables for this target.
include MultiConnection/CMakeFiles/MultiConnection.dir/progress.make

# Include the compile flags for this target's objects.
include MultiConnection/CMakeFiles/MultiConnection.dir/flags.make

MultiConnection/CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o: MultiConnection/CMakeFiles/MultiConnection.dir/flags.make
MultiConnection/CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o: ../MultiConnection/MultiConnection.cpp
MultiConnection/CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o: MultiConnection/CMakeFiles/MultiConnection.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object MultiConnection/CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT MultiConnection/CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o -MF CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o.d -o CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o -c /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/MultiConnection/MultiConnection.cpp

MultiConnection/CMakeFiles/MultiConnection.dir/MultiConnection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MultiConnection.dir/MultiConnection.cpp.i"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/MultiConnection/MultiConnection.cpp > CMakeFiles/MultiConnection.dir/MultiConnection.cpp.i

MultiConnection/CMakeFiles/MultiConnection.dir/MultiConnection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MultiConnection.dir/MultiConnection.cpp.s"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/MultiConnection/MultiConnection.cpp -o CMakeFiles/MultiConnection.dir/MultiConnection.cpp.s

# Object files for target MultiConnection
MultiConnection_OBJECTS = \
"CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o"

# External object files for target MultiConnection
MultiConnection_EXTERNAL_OBJECTS =

/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/MultiConnection: MultiConnection/CMakeFiles/MultiConnection.dir/MultiConnection.cpp.o
/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/MultiConnection: MultiConnection/CMakeFiles/MultiConnection.dir/build.make
/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/MultiConnection: MultiConnection/CMakeFiles/MultiConnection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/MultiConnection"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MultiConnection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
MultiConnection/CMakeFiles/MultiConnection.dir/build: /work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/MultiConnection
.PHONY : MultiConnection/CMakeFiles/MultiConnection.dir/build

MultiConnection/CMakeFiles/MultiConnection.dir/clean:
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnection && $(CMAKE_COMMAND) -P CMakeFiles/MultiConnection.dir/cmake_clean.cmake
.PHONY : MultiConnection/CMakeFiles/MultiConnection.dir/clean

MultiConnection/CMakeFiles/MultiConnection.dir/depend:
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/MultiConnection /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnection /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnection/CMakeFiles/MultiConnection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : MultiConnection/CMakeFiles/MultiConnection.dir/depend

