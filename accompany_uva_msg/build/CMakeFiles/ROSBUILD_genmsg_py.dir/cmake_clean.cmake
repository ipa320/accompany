FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/accompany_uva_msg/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/accompany_uva_msg/msg/__init__.py"
  "../src/accompany_uva_msg/msg/_TrackedHuman.py"
  "../src/accompany_uva_msg/msg/_TrackedHumans.py"
  "../src/accompany_uva_msg/msg/_Appearance.py"
  "../src/accompany_uva_msg/msg/_HumanLocationsParticles.py"
  "../src/accompany_uva_msg/msg/_HumanLocationsParticle.py"
  "../src/accompany_uva_msg/msg/_HumanLocations.py"
  "../src/accompany_uva_msg/msg/_HumanDetection.py"
  "../src/accompany_uva_msg/msg/_HumanDetections.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
