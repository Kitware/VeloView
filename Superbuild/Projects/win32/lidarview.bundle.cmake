# Bundling Scripts Stack Entry for $lidarview_appname - Win32 Specific
include(lidarview.bundle.common)

# append non-common lidarview modules to be processed in LidarviewBundle
list(APPEND lidarview_modules "${superbuild_install_location}/bin/VelodynePlugin.dll")
list(APPEND lidarview_modules "${superbuild_install_location}/bin/VelodynePluginPython.pyd")
list(APPEND lidarview_modules "${superbuild_install_location}/bin/VelodynePluginPythonD.dll")

# Trigger Win32-specific LidarView Bundling
include(${LidarViewSuperBuild_CMAKE_DIR}/bundle/win32/LidarviewBundle.cmake)

# Sensor calibration files
file(GLOB shared_files "${superbuild_install_location}/share/*.xml")
install(FILES ${shared_files}
        DESTINATION "share"
        COMPONENT superbuild)
unset(shared_files)

#Install Veloview User Guide
install(FILES "${superbuild_install_location}/doc/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
