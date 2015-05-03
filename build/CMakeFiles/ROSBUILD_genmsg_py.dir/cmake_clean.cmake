FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/ptam/msg"
  "../src/ptam/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ptam/msg/__init__.py"
  "../src/ptam/msg/_points_visible.py"
  "../src/ptam/msg/_map.py"
  "../src/ptam/msg/_map_info.py"
  "../src/ptam/msg/_RandT.py"
  "../src/ptam/msg/_point.py"
  "../src/ptam/msg/_pos_robot.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
