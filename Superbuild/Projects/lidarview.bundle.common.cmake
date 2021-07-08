# Crossplatform Bundling scripts for $lidarview_appname 
set(LidarViewSuperBuild_SOURCE_DIR "${superbuild_source_directory}")
set(LidarViewSuperBuild_CMAKE_DIR "${LidarViewSuperBuild_SOURCE_DIR}/lidarview-superbuild/CMake")
set(LidarViewSuperBuild_LVCORE_CMAKE_DIR "${LidarViewSuperBuild_SOURCE_DIR}/../LVCore/CMake")

# Make sure Branding is set early
include(${LidarViewSuperBuild_SOURCE_DIR}/../Application/branding.cmake)

# Trigger Crossplatform VeloView Bundling
include(${LidarViewSuperBuild_CMAKE_DIR}/bundle/LidarviewBundleCommon.cmake)

# Add $lidarview_appname specific executables in common with all platforms
list(APPEND lidarview_executables
  "PacketFileSender"
  )

# Add $lidarview_appname specific modules in common with all platforms
#list(APPEND lidarview_modules ...)

# Add $lidarview_appname specific python modules in common with all platforms
list(APPEND lidarview_modules_python
  VelodynePlugin
  lidarviewpythonplugin
  
  camera_path
  colormap_tools
  temporal_animation_cue_helpers
  example_temporal_animation
  example_non_temporal_animation
  numpy
  )

# Set $lidarview_appname license file.
set(CPACK_RESOURCE_FILE_LICENSE "${LidarViewSuperBuild_SOURCE_DIR}/lidarview-superbuild/LICENSE")
