add_external_project(paraview
  DEPENDS qt 

  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
    -DPARAVIEW_BUILD_QT_GUI:BOOL=ON

    # specify the apple app install prefix. No harm in specifying it for all
    # platforms.
    -DMACOSX_APP_INSTALL_PREFIX:PATH=<INSTALL_DIR>/Applications

  LIST_SEPARATOR +
)
