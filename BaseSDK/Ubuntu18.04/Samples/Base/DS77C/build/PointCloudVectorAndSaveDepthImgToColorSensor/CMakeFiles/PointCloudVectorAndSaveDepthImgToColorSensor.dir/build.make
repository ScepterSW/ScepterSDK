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
include PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/compiler_depend.make

# Include the progress variables for this target.
include PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/progress.make

# Include the compile flags for this target's objects.
include PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/flags.make

PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o: PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/flags.make
PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o: ../PointCloudVectorAndSaveDepthImgToColorSensor/PointCloudVectorAndSaveDepthImgToColorSensor.cpp
PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o: PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSaveDepthImgToColorSensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o -MF CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o.d -o CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o -c /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/PointCloudVectorAndSaveDepthImgToColorSensor/PointCloudVectorAndSaveDepthImgToColorSensor.cpp

PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.i"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSaveDepthImgToColorSensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/PointCloudVectorAndSaveDepthImgToColorSensor/PointCloudVectorAndSaveDepthImgToColorSensor.cpp > CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.i

PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.s"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSaveDepthImgToColorSensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/PointCloudVectorAndSaveDepthImgToColorSensor/PointCloudVectorAndSaveDepthImgToColorSensor.cpp -o CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.s

# Object files for target PointCloudVectorAndSaveDepthImgToColorSensor
PointCloudVectorAndSaveDepthImgToColorSensor_OBJECTS = \
"CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o"

# External object files for target PointCloudVectorAndSaveDepthImgToColorSensor
PointCloudVectorAndSaveDepthImgToColorSensor_EXTERNAL_OBJECTS =

/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/PointCloudVectorAndSaveDepthImgToColorSensor: PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/PointCloudVectorAndSaveDepthImgToColorSensor.cpp.o
/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/PointCloudVectorAndSaveDepthImgToColorSensor: PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/build.make
/work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/PointCloudVectorAndSaveDepthImgToColorSensor: PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/PointCloudVectorAndSaveDepthImgToColorSensor"
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSaveDepthImgToColorSensor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/build: /work/ScepterSDK/BaseSDK/Ubuntu18.04/PrecompiledSamples/DS77C_Samples/PointCloudVectorAndSaveDepthImgToColorSensor
.PHONY : PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/build

PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/clean:
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSaveDepthImgToColorSensor && $(CMAKE_COMMAND) -P CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/cmake_clean.cmake
.PHONY : PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/clean

PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/depend:
	cd /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/PointCloudVectorAndSaveDepthImgToColorSensor /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSaveDepthImgToColorSensor /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : PointCloudVectorAndSaveDepthImgToColorSensor/CMakeFiles/PointCloudVectorAndSaveDepthImgToColorSensor.dir/depend

