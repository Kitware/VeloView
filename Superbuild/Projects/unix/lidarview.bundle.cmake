set(paraview_plugin_path "lib/paraview-${PARAVIEW_VERSION}/plugins")
include(lidarview.bundle.common)

set(library_paths "${superbuild_install_location}/lib")

if (Qt5_DIR)
  list(APPEND library_paths
    "${Qt5_DIR}/../..")
endif ()

set(include_regexes)
set(exclude_regexes)



file(GLOB paraview_so_names
  "${superbuild_install_location}/lib/lib*.so*")
foreach (paraview_so_name IN LISTS paraview_so_names)
  superbuild_unix_install_plugin("${paraview_so_name}"
    "lib"
    "${lidarview_plugin_path}"
    LOADER_PATHS "${library_paths}"
    LOCATION     "lib")
endforeach ()

# Install lidarview executable.
superbuild_unix_install_program_fwd("${SOFTWARE_NAME}"
  "${lidarview_plugin_path}"
  INCLUDE_REGEXES     ${include_regexes}
  EXCLUDE_REGEXES     ${exclude_regexes})

# Install PacketFileSender executables.
superbuild_unix_install_program("${superbuild_install_location}/bin/PacketFileSender"
  "lib"
  SEARCH_DIRECTORIES  "${library_paths}"
  INCLUDE_REGEXES     ${include_regexes}
  EXCLUDE_REGEXES     ${exclude_regexes})

# install everything under lidarview.
file(GLOB so_names
  RELATIVE
  "${superbuild_install_location}/${lidarview_plugin_path}"
  "${superbuild_install_location}/${lidarview_plugin_path}/*.so*")
foreach (so_name IN LISTS so_names)
  superbuild_unix_install_plugin("${so_name}"
    "${lidarview_plugin_path}"
    "${lidarview_plugin_path}"
    LOADER_PATHS "${library_paths}"
    LOCATION     "${lidarview_plugin_path}"
    SEARCH_DIRECTORIES  "${library_paths}")
endforeach ()

if (python_enabled)
  if (python3_built_by_superbuild)
    include(python3.functions)
    superbuild_install_superbuild_python3(LIBSUFFIX "/python${superbuild_python_version}")
  endif ()

  superbuild_unix_install_python(
    LIBDIR              "lib"
    MODULES             ${python_modules}
    INCLUDE_REGEXES     ${include_regexes}
    EXCLUDE_REGEXES     ${exclude_regexes}
    MODULE_DIRECTORIES  "${superbuild_install_location}/lib/python${superbuild_python_version}/site-packages"
    LOADER_PATHS        "${library_paths}")
endif ()

if (qt5_enabled AND qt5_plugin_paths)
  # install an empty qt.cong file
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/qt.conf" "[Paths]\nPrefix = ..\n")
  install(FILES       "${CMAKE_CURRENT_BINARY_DIR}/qt.conf"
  DESTINATION "lib"
          COMPONENT   superbuild)
endif()

# install qt plugins
foreach (qt5_plugin_path IN LISTS qt5_plugin_paths)
  get_filename_component(qt5_plugin_group "${qt5_plugin_path}" DIRECTORY)
  get_filename_component(qt5_plugin_group "${qt5_plugin_group}" NAME)

  superbuild_unix_install_plugin("${qt5_plugin_path}"
    "${lidarview_plugin_path}"
    "${lidarview_plugin_path}/${qt5_plugin_group}/"
    LOADER_PATHS    "${library_paths}"
    INCLUDE_REGEXES     ${include_regexes}
    EXCLUDE_REGEXES     ${exclude_regexes})
endforeach ()

# Sensor calibration files
file(GLOB shared_files "${superbuild_install_location}/share/*.xml")
install(FILES ${shared_files}
  DESTINATION "share"
  COMPONENT superbuild
)
unset(shared_files)

install(FILES "${superbuild_install_location}/doc/VeloView_User_Guide.pdf"
  DESTINATION "doc"
  COMPONENT superbuild
)
