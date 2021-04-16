# Bundling Scripts Stack Entry for $lidarview_appname - Unix Specific
include(lidarview.bundle.common)

# Trigger Unix-specific LidarView Bundling
include(${LidarViewSuperBuild_CMAKE_DIR}/bundle/unix/LidarviewBundle.cmake)

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
