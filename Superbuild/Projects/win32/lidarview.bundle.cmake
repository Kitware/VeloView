# Bundling Scripts Stack Entry for $lidarview_appname - Win32 Specific
include(lidarview.bundle.common)

# Trigger Win32-specific VeloView Bundling
include(${LidarViewSuperBuild_CMAKE_DIR}/bundle/win32/LidarviewBundle.cmake)

# VeloView-Win32 Specifics

# Install Sensor calibration files
file(GLOB shared_files "${share_path}/*.xml")
install(FILES ${shared_files}
        DESTINATION "${LV_INSTALL_RESOURCE_DIR}")
unset(shared_files)

#Install Veloview User Guide
install(FILES "${superbuild_install_location}/doc/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
