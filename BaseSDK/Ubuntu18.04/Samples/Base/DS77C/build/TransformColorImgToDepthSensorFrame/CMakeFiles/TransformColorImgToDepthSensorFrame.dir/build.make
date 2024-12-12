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
include TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/compiler_depend.make

# Include the progress variables for this target.
include TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/progress.make

# Include the compile flags for this target's objects.
include TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/flags.make

TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o: TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/flags.make
TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o: ../TransformColorImgToDepthSensorFrame/TransformColorImgToDepthSensorFrame.cpp
TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o: TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformColorImgToDepthSensorFrame && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o -MF CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o.d -o CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o -c /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/TransformColorImgToDepthSensorFrame/TransformColorImgToDepthSensorFrame.cpp

TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.i"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformColorImgToDepthSensorFrame && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/TransformColorImgToDepthSensorFrame/TransformColorImgToDepthSensorFrame.cpp > CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.i

TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.s"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformColorImgToDepthSensorFrame && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/TransformColorImgToDepthSensorFrame/TransformColorImgToDepthSensorFrame.cpp -o CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.s

# Object files for target TransformColorImgToDepthSensorFrame
TransformColorImgToDepthSensorFrame_OBJECTS = \
"CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o"

# External object files for target TransformColorImgToDepthSensorFrame
TransformColorImgToDepthSensorFrame_EXTERNAL_OBJECTS =

/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/TransformColorImgToDepthSensorFrame: TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/TransformColorImgToDepthSensorFrame.cpp.o
/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/TransformColorImgToDepthSensorFrame: TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/build.make
/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/TransformColorImgToDepthSensorFrame: TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/TransformColorImgToDepthSensorFrame"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformColorImgToDepthSensorFrame && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TransformColorImgToDepthSensorFrame.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/build: /work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/TransformColorImgToDepthSensorFrame
.PHONY : TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/build

TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/clean:
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformColorImgToDepthSensorFrame && $(CMAKE_COMMAND) -P CMakeFiles/TransformColorImgToDepthSensorFrame.dir/cmake_clean.cmake
.PHONY : TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/clean

TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/depend:
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/TransformColorImgToDepthSensorFrame /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformColorImgToDepthSensorFrame /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : TransformColorImgToDepthSensorFrame/CMakeFiles/TransformColorImgToDepthSensorFrame.dir/depend

