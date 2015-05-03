FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam/msg"
  "../src/ptam/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/move_robot.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_move_robot.lisp"
  "../srv_gen/lisp/VisibilityCalculator.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_VisibilityCalculator.lisp"
  "../srv_gen/lisp/Frontier_check.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Frontier_check.lisp"
  "../srv_gen/lisp/trajectory_checker.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_trajectory_checker.lisp"
  "../srv_gen/lisp/FrontierExtractor.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_FrontierExtractor.lisp"
  "../srv_gen/lisp/cluster.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_cluster.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
