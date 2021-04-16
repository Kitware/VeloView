# Crossplatform Bundling scripts for $lidarview_appname 

set(LidarViewSuperBuild_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../")
set(LidarViewSuperBuild_CMAKE_DIR "${LidarViewSuperBuild_SOURCE_DIR}/lidarview-superbuild/CMake")

# Trigger Crossplatform LidarView Bundling
include(${LidarViewSuperBuild_CMAKE_DIR}/bundle/LidarviewBundleCommon.cmake)

# here we can add some optionnal lidarview executables in common with all platforms
list(APPEND lidarview_executables
    "PacketFileSender"
    )

# here we can add some optionnal python modules in common with all platforms
list(APPEND python_modules
  camera_path
  colormap_tools
  temporal_animation_cue_helpers
  example_temporal_animation
  example_non_temporal_animation
  )

#auto load eye dome lighting plugin
set(paraview_plugin_EyeDomeLightingView_auto_load ON)
