include(lidarview.bundle.common)

set(library_paths "${superbuild_install_location}/lib"
                  "${superbuild_install_location}/lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}"
                  "${superbuild_install_location}/bin")

if (Qt5_DIR)
  list(APPEND library_paths
    "${Qt5_DIR}/../..")
endif ()

# Install lidarview executable.
superbuild_unix_install_program_fwd("${SOFTWARE_NAME}"
  "lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}"
  SEARCH_DIRECTORIES  "${library_paths}")

# Install PacketFileSender executables.
superbuild_unix_install_program("${superbuild_install_location}/bin/PacketFileSender"
  "lib"
  SEARCH_DIRECTORIES  "${library_paths}")

# install paraview plugins
foreach (paraview_plugin_path IN LISTS paraview_plugin_paths)
  superbuild_unix_install_plugin("${paraview_plugin_path}"
    "lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}"
    "${paraview_plugin_subdir}"
    LOADER_PATHS  "${library_paths}"
    LOCATION  "${paraview_plugin_subdir}")
endforeach ()

# install .plugins file list
install(FILES       "${plugins_file}"
        DESTINATION ${paraview_plugin_subdir}
        COMPONENT   superbuild)

# These module are not processed automatically by superbuild because there is 
# no path leading to them in binary LidarView or in any of its .so dependencies
file(GLOB so_names
  RELATIVE
  "${superbuild_install_location}/lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}"
  "${superbuild_install_location}/lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}/*PluginPython*.so*")
foreach (so_name IN LISTS so_names)
  superbuild_unix_install_plugin("${so_name}"
    "lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}"
    "lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}"
    LOADER_PATHS "${library_paths}"
    LOCATION  "lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}")
endforeach ()

if (python_enabled)
  if (python3_built_by_superbuild)
    include(python3.functions)
    superbuild_install_superbuild_python3(LIBSUFFIX "/python${superbuild_python_version}")
  endif ()

  superbuild_unix_install_python(
    LIBDIR              "lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}"
    MODULES             ${python_modules}
    MODULE_DIRECTORIES  "${superbuild_install_location}/lib/python${superbuild_python_version}/site-packages"
    LOADER_PATHS        "${library_paths}")
endif ()

# An empty paraview directory is created as it is needed to run lidarview
# This is because all plugins are now stored in the lib/lidarview-{version}/plugins directory
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib/paraview-${PARAVIEW_VERSION}")
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib/paraview-${PARAVIEW_VERSION}"
        DESTINATION "lib"
        COMPONENT   superbuild)
        
if (qt5_enabled AND qt5_plugin_paths)
  # install an empty qt.cong file
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/qt.conf" "[Paths]\nPrefix = ..\n")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/qt.conf"
          DESTINATION "lib"
          COMPONENT   superbuild)
endif()

# install qt plugins
foreach (qt5_plugin_path IN LISTS qt5_plugin_paths)
  get_filename_component(qt5_plugin_group "${qt5_plugin_path}" DIRECTORY)
  get_filename_component(qt5_plugin_group "${qt5_plugin_group}" NAME)

  superbuild_unix_install_plugin("${qt5_plugin_path}"
    "lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}"
    "lib/lidarview-${LV_VERSION_MAJOR}.${LV_VERSION_MINOR}/${qt5_plugin_group}/"
    LOADER_PATHS "${library_paths}")
endforeach ()

# Sensor calibration files
file(GLOB shared_files "${superbuild_install_location}/share/*.xml")
install(FILES ${shared_files}
        DESTINATION "share"
        COMPONENT superbuild)
unset(shared_files)

install(FILES "${superbuild_install_location}/doc/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
