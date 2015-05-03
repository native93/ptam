FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam/msg"
  "../src/ptam/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/points_visible.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_points_visible.lisp"
  "../msg_gen/lisp/map.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_map.lisp"
  "../msg_gen/lisp/map_info.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_map_info.lisp"
  "../msg_gen/lisp/RandT.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_RandT.lisp"
  "../msg_gen/lisp/point.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_point.lisp"
  "../msg_gen/lisp/pos_robot.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_pos_robot.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
