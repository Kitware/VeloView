add_external_project(pythonqt
  DEPENDS qt python

  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON

    -DPythonQt_Wrap_Qtcore:BOOL=ON
    -DPythonQt_Wrap_Qtgui:BOOL=ON
    -DPythonQt_Wrap_Qtuitools:BOOL=ON

    # specify the apple app install prefix. No harm in specifying it for all
    # platforms.
    -DMACOSX_APP_INSTALL_PREFIX:PATH=<INSTALL_DIR>/Applications

  LIST_SEPARATOR +
)
