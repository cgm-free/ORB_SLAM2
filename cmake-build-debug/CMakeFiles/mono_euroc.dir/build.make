# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/cgm/下载/CLion-2019.3.6/clion-2019.3.6/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/cgm/下载/CLion-2019.3.6/clion-2019.3.6/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cgm/ORB_SLAM2_detailed_comments-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cgm/ORB_SLAM2_detailed_comments-master/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/mono_euroc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mono_euroc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mono_euroc.dir/flags.make

CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.o: CMakeFiles/mono_euroc.dir/flags.make
CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.o: ../Examples/Monocular/mono_euroc.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cgm/ORB_SLAM2_detailed_comments-master/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.o -c /home/cgm/ORB_SLAM2_detailed_comments-master/Examples/Monocular/mono_euroc.cc

CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cgm/ORB_SLAM2_detailed_comments-master/Examples/Monocular/mono_euroc.cc > CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.i

CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cgm/ORB_SLAM2_detailed_comments-master/Examples/Monocular/mono_euroc.cc -o CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.s

# Object files for target mono_euroc
mono_euroc_OBJECTS = \
"CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.o"

# External object files for target mono_euroc
mono_euroc_EXTERNAL_OBJECTS =

../Examples/Monocular/mono_euroc: CMakeFiles/mono_euroc.dir/Examples/Monocular/mono_euroc.cc.o
../Examples/Monocular/mono_euroc: CMakeFiles/mono_euroc.dir/build.make
../Examples/Monocular/mono_euroc: ../lib/libORB_SLAM2.so
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_gapi.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_stitching.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_aruco.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_bgsegm.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_bioinspired.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_ccalib.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_cvv.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_dnn_objdetect.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_dnn_superres.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_dpm.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_highgui.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_face.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_freetype.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_fuzzy.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_hdf.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_hfs.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_img_hash.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_line_descriptor.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_quality.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_reg.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_rgbd.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_saliency.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_sfm.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_stereo.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_structured_light.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_phase_unwrapping.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_superres.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_optflow.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_surface_matching.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_tracking.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_datasets.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_plot.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_text.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_dnn.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_videostab.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_videoio.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_viz.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_xfeatures2d.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_ml.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_shape.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_ximgproc.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_video.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_xobjdetect.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_objdetect.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_calib3d.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_features2d.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_flann.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_xphoto.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_photo.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_imgproc.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libopencv_core.so.4.2.0
../Examples/Monocular/mono_euroc: /usr/local/lib/libpangolin.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libpython3.8.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libdc1394.so
../Examples/Monocular/mono_euroc: /usr/lib/libOpenNI.so
../Examples/Monocular/mono_euroc: /usr/lib/libOpenNI2.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/Monocular/mono_euroc: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/Monocular/mono_euroc: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/Monocular/mono_euroc: ../Thirdparty/g2o/lib/libg2o.so
../Examples/Monocular/mono_euroc: CMakeFiles/mono_euroc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cgm/ORB_SLAM2_detailed_comments-master/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/Monocular/mono_euroc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mono_euroc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mono_euroc.dir/build: ../Examples/Monocular/mono_euroc

.PHONY : CMakeFiles/mono_euroc.dir/build

CMakeFiles/mono_euroc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mono_euroc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mono_euroc.dir/clean

CMakeFiles/mono_euroc.dir/depend:
	cd /home/cgm/ORB_SLAM2_detailed_comments-master/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cgm/ORB_SLAM2_detailed_comments-master /home/cgm/ORB_SLAM2_detailed_comments-master /home/cgm/ORB_SLAM2_detailed_comments-master/cmake-build-debug /home/cgm/ORB_SLAM2_detailed_comments-master/cmake-build-debug /home/cgm/ORB_SLAM2_detailed_comments-master/cmake-build-debug/CMakeFiles/mono_euroc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mono_euroc.dir/depend

