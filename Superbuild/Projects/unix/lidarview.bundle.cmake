# include(paraview-version)

include(lidarview.bundle.common)

set(lidarview_component "${SOFTWARE_NAME}")
include(lidarview.bundle.unix)

install(DIRECTORY "${superbuild_install_location}/lib/paraview-${PARAVIEW_VERSION}"
  DESTINATION "lib"
  USE_SOURCE_PERMISSIONS
  COMPONENT superbuild)

# install all libraries
install(DIRECTORY "${superbuild_install_location}/lib/lidarview-${VV_VERSION}"
  DESTINATION "lib"
  USE_SOURCE_PERMISSIONS
  COMPONENT superbuild)

# Sensor calibration files
file(GLOB shared_files "${superbuild_install_location}/share/*.xml")
install(FILES ${shared_files}
  DESTINATION "share"
  COMPONENT superbuild
)
unset(shared_files)


# Workaround to ship required .so in the .sh installer
file(GLOB lib_files_so
  "${superbuild_install_location}/lib/libPythonQt.so"
  "${superbuild_install_location}/lib/libpcap*.so*"
  "${superbuild_install_location}/lib/liblas*.so*"
  "${superbuild_install_location}/lib/libboost*.so*" # could be replaced by a package dependency
  "${superbuild_install_location}/lib/libQt*.so*" #  could be replaced by a package dependency
  "${superbuild_install_location}/lib/libfreetype*.so*"
  "${superbuild_install_location}/lib/libfontconfig*.so*"
)

install(DIRECTORY
  # install all qt plugins (including sqllite).
  # FIXME: we can reconfigure Qt to be built with inbuilt sqllite support to
  # avoid the need for plugins.
  "${superbuild_install_location}/plugins/"
  DESTINATION "lib/lidarview-${VV_VERSION}"
  COMPONENT superbuild
  PATTERN "*.a" EXCLUDE
  PATTERN "lidarview-${VV_VERSION}" EXCLUDE
  PATTERN "fontconfig" EXCLUDE
  PATTERN "*.jar" EXCLUDE
  PATTERN "*.debug.*" EXCLUDE
  PATTERN "libboost*" EXCLUDE)

install(FILES ${lib_files_so}
  DESTINATION "lib/lidarview-${VV_VERSION}"
  COMPONENT superbuild)
unset(lib_files_so)

install(FILES "${superbuild_install_location}/doc/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
