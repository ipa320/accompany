FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/AccompanyService/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/AccompanyService/AddTwoInts.h"
  "../srv_gen/cpp/include/AccompanyService/db_msg.h"
  "../srv_gen/cpp/include/AccompanyService/AccompanyAction.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
