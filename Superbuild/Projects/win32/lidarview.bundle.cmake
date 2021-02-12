include(lidarview.bundle.common)

# Set NSIS install specific stuff.
if (CMAKE_CL_64)
  # Change default installation root path for Windows x64.
  set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
endif ()

set(CPACK_NSIS_HELP_LINK "https://www.paraview.org/lidarview/")
set(${SOFTWARE_NAME}_description "${SOFTWARE_NAME} ${lidarview_version_full}")
set(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_LIST_DIR}/../../../Application/SoftwareInformation/logo.ico")

set(library_paths "${superbuild_install_location}/lib"
                  "${superbuild_install_location}/bin"
                  "${superbuild_install_location}/Python")

if (Qt5_DIR)
  list(APPEND library_paths
    "${Qt5_DIR}/../../../bin")
endif ()

# Install lidarview executables to bin.
foreach (executable IN LISTS lidarview_executables)
  if (DEFINED "${executable}_description")
    list(APPEND CPACK_NSIS_MENU_LINKS
      "bin/${executable}.exe" "${${executable}_description}")
  endif ()

  superbuild_windows_install_program("${executable}" "bin"
    SEARCH_DIRECTORIES "${library_paths}"
    EXCLUDE_REGEXES    ${exclude_regexes})
endforeach()

# install paraview plugins
foreach (lidarview_plugin_path IN LISTS lidarview_plugin_paths)
  superbuild_windows_install_plugin("${lidarview_plugin_path}"
    "bin"
    "${lidarview_plugin_subdir}"
    SEARCH_DIRECTORIES "${library_paths}"
    LOCATION "${lidarview_plugin_subdir}")
endforeach ()

install(FILES       "${plugins_file}"
        DESTINATION ${lidarview_plugin_subdir}
        COMPONENT   superbuild)

if (python3_enabled)
  if (python3_built_by_superbuild)
    include(python3.functions)
    superbuild_install_superbuild_python3()
  endif ()

  superbuild_windows_install_python(
    MODULES ${python_modules}
    MODULE_DIRECTORIES  "${superbuild_install_location}/bin/Lib"
                        "${superbuild_install_location}/bin/Lib/site-packages"
    SEARCH_DIRECTORIES  "${library_paths}")
endif ()

foreach (qt5_plugin_path IN LISTS qt5_plugin_paths)
  get_filename_component(qt5_plugin_group "${qt5_plugin_path}" DIRECTORY)
  get_filename_component(qt5_plugin_group "${qt5_plugin_group}" NAME)

  superbuild_windows_install_plugin(
    "${qt5_plugin_path}"
    "bin"
    "bin/${qt5_plugin_group}"
    SEARCH_DIRECTORIES "${library_paths}")
endforeach ()

if (qt5_enabled)
  foreach (qt5_opengl_lib IN ITEMS opengl32sw libEGL libGLESv2 libEGLd
      Qt5Core Qt5Gui Qt5Widgets Qt5Help Qt5Network Qt5PrintSupport Qt5Sql)
    superbuild_windows_install_plugin(
      "${Qt5_DIR}/../../../bin/${qt5_opengl_lib}.dll"
      "bin"
      "bin"
      SEARCH_DIRECTORIES "${library_paths}")
  endforeach ()
endif ()

# Sensor calibration files
file(GLOB shared_files "${superbuild_install_location}/share/*.xml")
install(FILES ${shared_files}
  DESTINATION "share"
  COMPONENT superbuild
)
unset(shared_files)

# some dlls are missing.
# They should be installed automatically because of paraview/vtk's cmake lists
file(GLOB vtk_dlls
  "${superbuild_install_location}/bin/vtk*.dll"
  "${superbuild_install_location}/bin/LidarPluginPython.pyd"
  "${superbuild_install_location}/bin/LidarPluginPythonD.dll"
  "${superbuild_install_location}/bin/VelodynePlugin.dll"
  "${superbuild_install_location}/bin/VelodynePluginPython.pyd"
  "${superbuild_install_location}/bin/VelodynePluginPythonD.dll")
install(FILES ${vtk_dlls}
  DESTINATION "bin"
  COMPONENT superbuild
)
unset(vtk_dlls)

file(GLOB boost_bin_dll "${superbuild_install_location}/bin/boost*.dll")
install(FILES ${boost_bin_dll}
  DESTINATION "bin"
  COMPONENT superbuild
)
unset(boost_bin_dll)

set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION "bin")
include(InstallRequiredSystemLibraries)

install(FILES "${superbuild_install_location}/doc/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
