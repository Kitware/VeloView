# Bundling Scripts Stack Entry for $lidarview_appname - Apple Specific
include(lidarview.bundle.common)

# Trigger Apple-specific VeloView Bundling
include(${LidarViewSuperBuild_CMAKE_DIR}/bundle/apple/LidarviewBundle.cmake)

# VeloView-Apple Specifics

# Install Sensor calibration files
file(GLOB shared_files "${share_path}/*.xml")
install(FILES ${shared_files}
        DESTINATION "${share_dest}")
unset(shared_files)


#Install Veloview User Guide
install(FILES "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Resources/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
