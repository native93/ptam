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

# Utility rule file for ROSBUILD_gensrv_cpp.

# Include the progress variables for this target.
include CMakeFiles/ROSBUILD_gensrv_cpp.dir/progress.make

CMakeFiles/ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/move_robot.h
CMakeFiles/ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/VisibilityCalculator.h
CMakeFiles/ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/Frontier_check.h
CMakeFiles/ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/trajectory_checker.h
CMakeFiles/ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/FrontierExtractor.h
CMakeFiles/ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/cluster.h

../srv_gen/cpp/include/ptam/move_robot.h: ../srv/move_robot.srv
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../srv_gen/cpp/include/ptam/move_robot.h: ../manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/roslang/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/roslib/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/pcl/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/rospy/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/rostest/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../srv_gen/cpp/include/ptam/move_robot.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../srv_gen/cpp/include/ptam/move_robot.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py /home/nazrul/fuerte_workspace/ptam/srv/move_robot.srv

../srv_gen/cpp/include/ptam/VisibilityCalculator.h: ../srv/VisibilityCalculator.srv
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: ../manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/roslang/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/roslib/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/pcl/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/rospy/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/rostest/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../srv_gen/cpp/include/ptam/VisibilityCalculator.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../srv_gen/cpp/include/ptam/VisibilityCalculator.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py /home/nazrul/fuerte_workspace/ptam/srv/VisibilityCalculator.srv

../srv_gen/cpp/include/ptam/Frontier_check.h: ../srv/Frontier_check.srv
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../srv_gen/cpp/include/ptam/Frontier_check.h: ../manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/roslang/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/roslib/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/pcl/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/rospy/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/rostest/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../srv_gen/cpp/include/ptam/Frontier_check.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../srv_gen/cpp/include/ptam/Frontier_check.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py /home/nazrul/fuerte_workspace/ptam/srv/Frontier_check.srv

../srv_gen/cpp/include/ptam/trajectory_checker.h: ../srv/trajectory_checker.srv
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../srv_gen/cpp/include/ptam/trajectory_checker.h: ../manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/roslang/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/roslib/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/pcl/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/rospy/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/rostest/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../srv_gen/cpp/include/ptam/trajectory_checker.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../srv_gen/cpp/include/ptam/trajectory_checker.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py /home/nazrul/fuerte_workspace/ptam/srv/trajectory_checker.srv

../srv_gen/cpp/include/ptam/FrontierExtractor.h: ../srv/FrontierExtractor.srv
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../srv_gen/cpp/include/ptam/FrontierExtractor.h: ../manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/roslang/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/roslib/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/pcl/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/rospy/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/rostest/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../srv_gen/cpp/include/ptam/FrontierExtractor.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../srv_gen/cpp/include/ptam/FrontierExtractor.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py /home/nazrul/fuerte_workspace/ptam/srv/FrontierExtractor.srv

../srv_gen/cpp/include/ptam/cluster.h: ../srv/cluster.srv
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/roscpp/rosbuild/scripts/genmsg_cpp.py
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/roslib/bin/gendeps
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../srv_gen/cpp/include/ptam/cluster.h: ../msg/point.msg
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../srv_gen/cpp/include/ptam/cluster.h: ../manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/roslang/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/roscpp/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/std_msgs/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/roslib/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/rosconsole/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/message_filters/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/pcl/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/rosbag/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/rospy/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/rosservice/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/bullet/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/rostest/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/share/roswtf/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../srv_gen/cpp/include/ptam/cluster.h: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../srv_gen/cpp/include/ptam/cluster.h"
	/opt/ros/fuerte/share/roscpp/rosbuild/scripts/gensrv_cpp.py /home/nazrul/fuerte_workspace/ptam/srv/cluster.srv

ROSBUILD_gensrv_cpp: CMakeFiles/ROSBUILD_gensrv_cpp
ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/move_robot.h
ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/VisibilityCalculator.h
ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/Frontier_check.h
ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/trajectory_checker.h
ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/FrontierExtractor.h
ROSBUILD_gensrv_cpp: ../srv_gen/cpp/include/ptam/cluster.h
ROSBUILD_gensrv_cpp: CMakeFiles/ROSBUILD_gensrv_cpp.dir/build.make
.PHONY : ROSBUILD_gensrv_cpp

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_gensrv_cpp.dir/build: ROSBUILD_gensrv_cpp
.PHONY : CMakeFiles/ROSBUILD_gensrv_cpp.dir/build

CMakeFiles/ROSBUILD_gensrv_cpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_gensrv_cpp.dir/clean

CMakeFiles/ROSBUILD_gensrv_cpp.dir/depend:
	cd /home/nazrul/fuerte_workspace/ptam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nazrul/fuerte_workspace/ptam /home/nazrul/fuerte_workspace/ptam /home/nazrul/fuerte_workspace/ptam/build /home/nazrul/fuerte_workspace/ptam/build /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles/ROSBUILD_gensrv_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_gensrv_cpp.dir/depend

