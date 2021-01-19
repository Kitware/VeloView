include(lidarview.bundle.common)

# Set NSIS install specific stuff.
if (CMAKE_CL_64)
  # Change default installation root path for Windows x64.
  set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
endif ()

set(CPACK_NSIS_HELP_LINK "https://www.paraview.org/lidarview/")
set(${SOFTWARE_NAME}_description "${SOFTWARE_NAME} ${lidarview_version_full}")
set(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_LIST_DIR}/../../../Application/SoftwareInformation/logo.ico")


set(library_paths "lib")

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

  superbuild_windows_install_program("${executable}" "bin" SEARCH_DIRECTORIES
    "${library_paths}")
endforeach()

# previously was:
# install(DIRECTORY "${superbuild_install_location}/bin/"
#         DESTINATION "bin"
#         COMPONENT superbuild)
#


if (python_enabled)
  include(python.functions)
  superbuild_install_superbuild_python()

  superbuild_windows_install_python(
    MODULES paraview
            vtk
            vtkmodules
            ${python_modules}
    MODULE_DIRECTORIES  "${superbuild_install_location}/bin/Lib/site-packages"
                        "${superbuild_install_location}/lib/site-packages"
                        "${superbuild_install_location}/lib/python2.7/site-packages"
                        "${superbuild_install_location}/lib/paraview-${paraview_version_major}.${paraview_version_minor}/site-packages"
    SEARCH_DIRECTORIES  "lib" "${superbuild_install_location}/bin")

  if (matplotlib_enabled)
    install(
      DIRECTORY   "${superbuild_install_location}/bin/Lib/site-packages/matplotlib/mpl-data/"
      DESTINATION "bin/Lib/site-packages/matplotlib/mpl-data"
      COMPONENT   superbuild)
  endif ()
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
  # file(GLOB platforms_dll "${Qt5_DIR}/../../../plugins/platforms/*.dll")
  install(DIRECTORY "${Qt5_DIR}/../../../plugins/platforms"
    DESTINATION "bin"
    COMPONENT superbuild
  )
  # required to have Windows Vista style (else Windows 98 look):
  # see https://gitlab.kitware.com/paraview/paraview/issues/18551
  # the bug fix is not yet integrated to the Superbuild,
  # and we do not use the superbuild for Qt (yet) anyway
  # note that this will only add the directoy to the package,
  # not to build_dir/install
  install(DIRECTORY "${Qt5_DIR}/../../../plugins/styles"
    DESTINATION "bin"
    COMPONENT superbuild
  )
  # unset(platforms_dll)
endif ()

install(DIRECTORY "${superbuild_install_location}/lib/paraview-${PARAVIEW_VERSION}"
        DESTINATION "lib"
        USE_SOURCE_PERMISSIONS
        COMPONENT superbuild
        PATTERN "*.lib" EXCLUDE)

# could/should be done with superbuild_windows_install_python ?
install(DIRECTORY "${superbuild_install_location}/bin/site-packages"
        DESTINATION "bin"
        COMPONENT superbuild)

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
  "${superbuild_install_location}/bin/VelodynePluginPythonD.dll"
  )
install(FILES ${vtk_dlls}
  DESTINATION "bin"
  COMPONENT superbuild
)
unset(vtk_dlls)

file(GLOB boost_lib_dll "${superbuild_install_location}/lib/boost*.dll")
install(FILES ${boost_lib_dll}
  DESTINATION "lib"
  COMPONENT superbuild
)
unset(boost_lib_dll)

file(GLOB boost_bin_dll "${superbuild_install_location}/bin/boost*.dll")
install(FILES ${boost_bin_dll}
  DESTINATION "bin"
  COMPONENT superbuild
)
unset(boost_bin_dll)

file(GLOB pointcloud_dll
	"${superbuild_install_location}/bin/PointCloudPlugin.dll"
	"${superbuild_install_location}/bin/EyeDomeLightingView.dll")
install(FILES ${pointcloud_dll}
  DESTINATION "bin"
  COMPONENT superbuild
)
unset(pointcloud_dll)

# Install all auto-loaded plugins located in bin/plugins and their dependencies
file(GLOB lv_plugins "${superbuild_install_location}/bin/plugins/*.dll")
foreach (lv_plugin IN LISTS lv_plugins)
  superbuild_windows_install_plugin(
    "${lv_plugin}"
    "bin"
    "bin/plugins"
    SEARCH_DIRECTORIES "${superbuild_install_location}/bin")
endforeach ()

set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION "bin")
include(InstallRequiredSystemLibraries)

install(FILES "${superbuild_install_location}/doc/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
