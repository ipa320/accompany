FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/AccompanyService/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/AddTwoInts.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AddTwoInts.lisp"
  "../srv_gen/lisp/db_msg.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_db_msg.lisp"
  "../srv_gen/lisp/AccompanyAction.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AccompanyAction.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
