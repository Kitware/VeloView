# Bundling Scripts Stack Entry for $lidarview_appname - Apple Specific
include(lidarview.bundle.common)

# append non-common lidarview modules to be processed in LidarviewBundle
list(APPEND lidarview_modules "VelodynePluginPython.so")
list(APPEND lidarview_modules "libVelodynePluginPythonD.dylib")

# Trigger Apple-specific LidarView Bundling
include(${LidarViewSuperBuild_CMAKE_DIR}/bundle/apple/LidarviewBundle.cmake)

# Sensor calibration files
file(GLOB shared_files "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Resources/*.xml")
install(FILES ${shared_files}
        DESTINATION "${lidarview_appname}/Contents/Resources"
        COMPONENT superbuild)
unset(shared_files)
