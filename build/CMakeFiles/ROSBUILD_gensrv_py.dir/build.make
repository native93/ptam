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

# Utility rule file for ROSBUILD_gensrv_py.

# Include the progress variables for this target.
include CMakeFiles/ROSBUILD_gensrv_py.dir/progress.make

CMakeFiles/ROSBUILD_gensrv_py: ../src/ptam/srv/__init__.py

../src/ptam/srv/__init__.py: ../src/ptam/srv/_move_robot.py
../src/ptam/srv/__init__.py: ../src/ptam/srv/_VisibilityCalculator.py
../src/ptam/srv/__init__.py: ../src/ptam/srv/_Frontier_check.py
../src/ptam/srv/__init__.py: ../src/ptam/srv/_trajectory_checker.py
../src/ptam/srv/__init__.py: ../src/ptam/srv/_FrontierExtractor.py
../src/ptam/srv/__init__.py: ../src/ptam/srv/_cluster.py
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/ptam/srv/__init__.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --initpy /home/nazrul/fuerte_workspace/ptam/srv/move_robot.srv /home/nazrul/fuerte_workspace/ptam/srv/VisibilityCalculator.srv /home/nazrul/fuerte_workspace/ptam/srv/Frontier_check.srv /home/nazrul/fuerte_workspace/ptam/srv/trajectory_checker.srv /home/nazrul/fuerte_workspace/ptam/srv/FrontierExtractor.srv /home/nazrul/fuerte_workspace/ptam/srv/cluster.srv

../src/ptam/srv/_move_robot.py: ../srv/move_robot.srv
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/ptam/srv/_move_robot.py: ../manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/pcl/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/rosbag/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/rosservice/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/ptam/srv/_move_robot.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/ptam/srv/_move_robot.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/nazrul/fuerte_workspace/ptam/srv/move_robot.srv

../src/ptam/srv/_VisibilityCalculator.py: ../srv/VisibilityCalculator.srv
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../src/ptam/srv/_VisibilityCalculator.py: ../manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/pcl/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/rosbag/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/rosservice/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/ptam/srv/_VisibilityCalculator.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/ptam/srv/_VisibilityCalculator.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/nazrul/fuerte_workspace/ptam/srv/VisibilityCalculator.srv

../src/ptam/srv/_Frontier_check.py: ../srv/Frontier_check.srv
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../src/ptam/srv/_Frontier_check.py: ../manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/pcl/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/rosbag/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/rosservice/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/ptam/srv/_Frontier_check.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/ptam/srv/_Frontier_check.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/nazrul/fuerte_workspace/ptam/srv/Frontier_check.srv

../src/ptam/srv/_trajectory_checker.py: ../srv/trajectory_checker.srv
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/ptam/srv/_trajectory_checker.py: ../manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/pcl/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/rosbag/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/rosservice/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/ptam/srv/_trajectory_checker.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/ptam/srv/_trajectory_checker.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/nazrul/fuerte_workspace/ptam/srv/trajectory_checker.srv

../src/ptam/srv/_FrontierExtractor.py: ../srv/FrontierExtractor.srv
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../src/ptam/srv/_FrontierExtractor.py: ../manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/pcl/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/rosbag/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/rosservice/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/ptam/srv/_FrontierExtractor.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/ptam/srv/_FrontierExtractor.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/nazrul/fuerte_workspace/ptam/srv/FrontierExtractor.srv

../src/ptam/srv/_cluster.py: ../srv/cluster.srv
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/geometry_msgs/msg/PoseArray.msg
../src/ptam/srv/_cluster.py: ../msg/point.msg
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/geometry_msgs/msg/Quaternion.msg
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/std_msgs/msg/Header.msg
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/geometry_msgs/msg/Pose.msg
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/geometry_msgs/msg/Point.msg
../src/ptam/srv/_cluster.py: ../manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/pcl/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/rosbag/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/rosservice/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/ptam/srv/_cluster.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/ptam/srv/_cluster.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/nazrul/fuerte_workspace/ptam/srv/cluster.srv

ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py
ROSBUILD_gensrv_py: ../src/ptam/srv/__init__.py
ROSBUILD_gensrv_py: ../src/ptam/srv/_move_robot.py
ROSBUILD_gensrv_py: ../src/ptam/srv/_VisibilityCalculator.py
ROSBUILD_gensrv_py: ../src/ptam/srv/_Frontier_check.py
ROSBUILD_gensrv_py: ../src/ptam/srv/_trajectory_checker.py
ROSBUILD_gensrv_py: ../src/ptam/srv/_FrontierExtractor.py
ROSBUILD_gensrv_py: ../src/ptam/srv/_cluster.py
ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py.dir/build.make
.PHONY : ROSBUILD_gensrv_py

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_gensrv_py.dir/build: ROSBUILD_gensrv_py
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/build

CMakeFiles/ROSBUILD_gensrv_py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/clean

CMakeFiles/ROSBUILD_gensrv_py.dir/depend:
	cd /home/nazrul/fuerte_workspace/ptam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nazrul/fuerte_workspace/ptam /home/nazrul/fuerte_workspace/ptam /home/nazrul/fuerte_workspace/ptam/build /home/nazrul/fuerte_workspace/ptam/build /home/nazrul/fuerte_workspace/ptam/build/CMakeFiles/ROSBUILD_gensrv_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/depend

