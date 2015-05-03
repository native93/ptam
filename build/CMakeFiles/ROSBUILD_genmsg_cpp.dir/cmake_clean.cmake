FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam/msg"
  "../src/ptam/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/ptam/points_visible.h"
  "../msg_gen/cpp/include/ptam/map.h"
  "../msg_gen/cpp/include/ptam/map_info.h"
  "../msg_gen/cpp/include/ptam/RandT.h"
  "../msg_gen/cpp/include/ptam/point.h"
  "../msg_gen/cpp/include/ptam/pos_robot.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
