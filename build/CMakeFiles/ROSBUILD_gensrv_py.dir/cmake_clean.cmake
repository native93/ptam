FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam/msg"
  "../src/ptam/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/ptam/srv/__init__.py"
  "../src/ptam/srv/_move_robot.py"
  "../src/ptam/srv/_VisibilityCalculator.py"
  "../src/ptam/srv/_Frontier_check.py"
  "../src/ptam/srv/_trajectory_checker.py"
  "../src/ptam/srv/_FrontierExtractor.py"
  "../src/ptam/srv/_cluster.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
