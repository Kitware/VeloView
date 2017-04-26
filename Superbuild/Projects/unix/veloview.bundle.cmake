#SET(CPACK_GENERATOR "DEB")
#SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "David Oroshnik <doroshnik@velodyne.com>") #required
#SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.1-6), libqt4-core (>= 4.8.4), libqt4-core (>= 4.8.4), python (>= 2.7)")
#SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

include(veloview.bundle.common)
include(CPack)

# install all ParaView's shared libraries.
install(DIRECTORY "${install_location}/lib/paraview-${PARAVIEW_VERSION}"
  DESTINATION "lib"
  USE_SOURCE_PERMISSIONS
  COMPONENT superbuild)

# install all velodyne libraries
install(DIRECTORY "${install_location}/lib/veloview-${VV_VERSION}"
  DESTINATION "lib"
  USE_SOURCE_PERMISSIONS
  COMPONENT superbuild)
# Sensor calibration files
file(GLOB shared_files "${install_location}/share/*.xml")
install(FILES ${shared_files}
  DESTINATION "share"
  COMPONENT superbuild
)
unset(shared_files)

if (qt_ENABLED AND NOT USE_SYSTEM_qt)
  install(DIRECTORY
    # install all qt plugins (including sqllite).
    # FIXME: we can reconfigure Qt to be built with inbuilt sqllite support to
    # avoid the need for plugins.
    "${install_location}/plugins/"
    DESTINATION "lib/veloview-${VV_VERSION}"
    COMPONENT superbuild
    PATTERN "*.a" EXCLUDE
    PATTERN "veloview-${VV_VERSION}" EXCLUDE
    PATTERN "fontconfig" EXCLUDE
    PATTERN "*.jar" EXCLUDE
    PATTERN "*.debug.*" EXCLUDE
    PATTERN "libboost*" EXCLUDE)
endif()

# install executables
foreach(executable VeloView)
  install(PROGRAMS "${install_location}/bin/${executable}"
    DESTINATION "bin"
    COMPONENT superbuild)
endforeach()
