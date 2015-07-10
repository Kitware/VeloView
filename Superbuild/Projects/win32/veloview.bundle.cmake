include(veloview.bundle.common)

#------------------------------------------------------------------------------
# set NSIS install specific stuff.

set (CPACK_NSIS_MENU_LINKS
  "bin/VeloView.exe" "VeloView"
  "doc/VeloView_Developer_Guide.pdf" "Developer Guide")

set(CPACK_PACKAGE_EXECUTABLES "VeloView" "VeloView" ${CPACK_PACKAGE_EXECUTABLES})
set(CPACK_CREATE_DESKTOP_LINKS "VeloView" ${CPACK_CREATE_DESKTOP_LINKS})
set(CPACK_NSIS_MODIFY_PATH ON)

set(AppName VeloView)

install(DIRECTORY "${install_location}/bin/"
        DESTINATION "bin"
        COMPONENT ${AppName})

# install python since (since python dlls are not in the install location)
if (python_ENABLED AND NOT USE_SYSTEM_python)
  # install the Python's modules.
  install(DIRECTORY "${SuperBuild_BINARY_DIR}/python/src/python/Lib"
          DESTINATION "bin"
          USE_SOURCE_PERMISSIONS
          COMPONENT ${AppName})

  # install python dlls.
  get_filename_component(python_bin_dir "${pv_python_executable}" PATH)
  install(DIRECTORY "${python_bin_dir}/"
          DESTINATION "bin"
          USE_SOURCE_PERMISSIONS
          COMPONENT ${AppName}
          FILES_MATCHING PATTERN "python*.dll")

  # install python pyd objects (python dlls).
  # For 64 bit builds, these are in an amd64/ subdir
  set(PYTHON_PCBUILD_SUBDIR "")
  if(CMAKE_CL_64)
    set(PYTHON_PCBUILD_SUBDIR "amd64/")
  endif()
  install(DIRECTORY "${SuperBuild_BINARY_DIR}/python/src/python/PCbuild/${PYTHON_PCBUILD_SUBDIR}"
          DESTINATION "bin/Lib"
          USE_SOURCE_PERMISSIONS
          COMPONENT ${AppName}
          FILES_MATCHING PATTERN "*.pyd")
endif()

# install paraview python modules and others.
install(DIRECTORY "${install_location}/lib/paraview-${PARAVIEW_VERSION}"
        DESTINATION "lib"
        USE_SOURCE_PERMISSIONS
        COMPONENT ${AppName}
        PATTERN "*.lib" EXCLUDE)

install(FILES "${install_location}/share/HDL-32.xml" "${install_location}/share/VLP-16.xml"
        DESTINATION "share"
        COMPONENT ${AppName})

install(FILES "${VeloViewSuperBuild_SOURCE_DIR}/../Documentation/VeloView_Developer_Guide.pdf"
        DESTINATION "doc"
        COMPONENT ${AppName})

#------------------------------------------------------------------------------
set (CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_LIST_DIR}/InstallerIcon.ico")

if (64bit_build)
  set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
endif()

# install system runtimes.
set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION "bin")
include(InstallRequiredSystemLibraries)
include(CPack)
