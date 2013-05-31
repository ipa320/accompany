FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/AccompanyService/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/AccompanyService/srv/__init__.py"
  "../src/AccompanyService/srv/_AddTwoInts.py"
  "../src/AccompanyService/srv/_db_msg.py"
  "../src/AccompanyService/srv/_AccompanyAction.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
