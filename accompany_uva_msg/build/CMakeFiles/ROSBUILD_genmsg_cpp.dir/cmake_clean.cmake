FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/accompany_uva_msg/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/accompany_uva_msg/TrackedHuman.h"
  "../msg_gen/cpp/include/accompany_uva_msg/TrackedHumans.h"
  "../msg_gen/cpp/include/accompany_uva_msg/Appearance.h"
  "../msg_gen/cpp/include/accompany_uva_msg/HumanLocationsParticles.h"
  "../msg_gen/cpp/include/accompany_uva_msg/HumanLocationsParticle.h"
  "../msg_gen/cpp/include/accompany_uva_msg/HumanLocations.h"
  "../msg_gen/cpp/include/accompany_uva_msg/HumanDetection.h"
  "../msg_gen/cpp/include/accompany_uva_msg/HumanDetections.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
