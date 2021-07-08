# Bundling Scripts Stack Entry for $lidarview_appname - Unix Specific
include(lidarview.bundle.common)

# Trigger Unix-specific VeloView Bundling
include(${LidarViewSuperBuild_CMAKE_DIR}/bundle/unix/LidarviewBundle.cmake)

# VeloView-Unix Specifics

# Install Sensor calibration files
file(GLOB shared_files "${share_path}/*.xml")
install(FILES ${shared_files}
        DESTINATION "${share_dest}")
unset(shared_files)

#Install Veloview User Guide
install(FILES "${superbuild_install_location}/doc/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
