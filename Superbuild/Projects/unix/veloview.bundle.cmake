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
