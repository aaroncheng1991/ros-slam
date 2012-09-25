FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/laser_to_wall/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/laser_to_wall/msg/__init__.py"
  "../src/laser_to_wall/msg/_WallScan.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
