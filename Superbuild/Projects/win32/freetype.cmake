add_external_project(
  freetype
  DEPENDS zlib
  CONFIGURE_COMMAND ""
  BUILD_IN_SOURCE 1
  BUILD_COMMAND
    ${CMAKE_COMMAND}
      -DFTJAM_EXECUTABLE:PATH=${FTJAM_EXECUTABLE}
      -DWORKING_DIRECTORY:PATH=<SOURCE_DIR>
      -P ${SuperBuild_PROJECTS_DIR}/win32/freetype-build.cmake
  INSTALL_COMMAND
    ${CMAKE_COMMAND}
      -DSOURCE_DIR:PATH=<SOURCE_DIR>
      -DINSTALL_DIR:PATH=<INSTALL_DIR>
      -P ${SuperBuild_PROJECTS_DIR}/win32/freetype-install.cmake
)
