# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nazrul/fuerte_workspace/ptam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nazrul/fuerte_workspace/ptam/build

# Utility rule file for ROSBUILD_genmsg_cpp.

# Include the progress variables for this target.
include CMakeFiles/ROSBUILD_genmsg_cpp.dir/progress.make

CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/points_visible.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/map.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/map_info.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/RandT.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/point.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/pos_robot.h

../msg_gen/cpp/include/ptam/points_visible.h: ../msg/points_visible.msg
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../msg_gen/cpp/include/ptam/points_visible.h: ../manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/roslang/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/roslib/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/pcl/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/rospy/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/rostest/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../msg_gen/cpp/include/ptam/points_visible.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/ptam/points_visible.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py /home/nazrul/fuerte_workspace/ptam/msg/points_visible.msg

../msg_gen/cpp/include/ptam/map.h: ../msg/map.msg
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../msg_gen/cpp/include/ptam/map.h: ../manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/roslang/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/roslib/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/pcl/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/rospy/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/rostest/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../msg_gen/cpp/include/ptam/map.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/ptam/map.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py /home/nazrul/fuerte_workspace/ptam/msg/map.msg

../msg_gen/cpp/include/ptam/map_info.h: ../msg/map_info.msg
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../msg_gen/cpp/include/ptam/map_info.h: ../manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/roslang/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/roslib/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/pcl/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/rospy/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/rostest/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../msg_gen/cpp/include/ptam/map_info.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/ptam/map_info.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py /home/nazrul/fuerte_workspace/ptam/msg/map_info.msg

../msg_gen/cpp/include/ptam/RandT.h: ../msg/RandT.msg
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../msg_gen/cpp/include/ptam/RandT.h: ../manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/roslang/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/roslib/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/pcl/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/rospy/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/rostest/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../msg_gen/cpp/include/ptam/RandT.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/ptam/RandT.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py /home/nazrul/fuerte_workspace/ptam/msg/RandT.msg

../msg_gen/cpp/include/ptam/point.h: ../msg/point.msg
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../msg_gen/cpp/include/ptam/point.h: ../manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/roslang/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/roslib/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/pcl/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/rospy/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/rostest/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../msg_gen/cpp/include/ptam/point.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/ptam/point.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py /home/nazrul/fuerte_workspace/ptam/msg/point.msg

../msg_gen/cpp/include/ptam/pos_robot.h: ../msg/pos_robot.msg
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../msg_gen/cpp/include/ptam/pos_robot.h: ../manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/roslang/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/roslib/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/pcl/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/rospy/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/rostest/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../msg_gen/cpp/include/ptam/pos_robot.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/ptam/pos_robot.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py /home/nazrul/fuerte_workspace/ptam/msg/pos_robot.msg

ROSBUILD_genmsg_cpp: CMakeFiles/ROSBUILD_genmsg_cpp
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/points_visible.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/map.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/map_info.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/RandT.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/point.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/ptam/pos_robot.h
ROSBUILD_genmsg_cpp: CMakeFiles/ROSBUILD_genmsg_cpp.dir/build.make
.PHONY : ROSBUILD_genmsg_cpp

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_genmsg_cpp.dir/build: ROSBUILD_genmsg_cpp
.PHONY : CMakeFiles/ROSBUILD_genmsg_cpp.dir/build

CMakeFiles/ROSBUILD_genmsg_cpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_genmsg_cpp.dir/clean

CMakeFiles/ROSBUILD_genmsg_cpp.dir/depend:
	cd /home/nazrul/fuerte_workspace/ptam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nazrul/fuerte_workspace/ptam /home/nazrul/fuerte_workspace/ptam /home/nazrul/fuerte_workspace/ptam/build /home/nazrul/fuerte_workspace/ptam/build /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles/ROSBUILD_genmsg_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_genmsg_cpp.dir/depend

