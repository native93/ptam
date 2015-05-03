FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam/msg"
  "../src/ptam/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/ptam/move_robot.h"
  "../srv_gen/cpp/include/ptam/VisibilityCalculator.h"
  "../srv_gen/cpp/include/ptam/Frontier_check.h"
  "../srv_gen/cpp/include/ptam/trajectory_checker.h"
  "../srv_gen/cpp/include/ptam/FrontierExtractor.h"
  "../srv_gen/cpp/include/ptam/cluster.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
