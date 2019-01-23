set(library_paths
  "${superbuild_install_location}/lib")

if (Qt5_DIR)
  list(APPEND library_paths
    "${Qt5_DIR}/../..")
endif ()

set(include_regexes)
set(exclude_regexes)

foreach (executable IN LISTS veloview_executables)
  superbuild_unix_install_program_fwd("${executable}"
    "bin"
    SEARCH_DIRECTORIES  "${library_paths}"
    INCLUDE_REGEXES     ${include_regexes}
    EXCLUDE_REGEXES     ${exclude_regexes})
endforeach ()

# this was present in ParaView's paraview.bundle.unix.cmake,
# delete when it is certain that this is not needed for VeloView:
if (python_enabled)
  include(python.functions)
  superbuild_install_superbuild_python(
    LIBSUFFIX "/python2.7")

  superbuild_unix_install_python(
    LIBDIR              "lib"
    MODULES             paraview
                        vtk
                        vtkmodules
                        ${python_modules}
    INCLUDE_REGEXES     ${include_regexes}
    EXCLUDE_REGEXES     ${exclude_regexes}
    MODULE_DIRECTORIES  "${superbuild_install_location}/lib/python2.7/site-packages"
    LOADER_PATHS        "${library_paths}")

  # if (matplotlib_built_by_superbuild)
  #   install(
  #     DIRECTORY   "${superbuild_install_location}/lib/python2.7/site-packages/matplotlib/mpl-data/"
  #     DESTINATION "lib/python2.7/site-packages/matplotlib/mpl-data"
  #     COMPONENT   superbuild)
  # endif ()
endif ()

if (qt5_enabled)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/qt.conf" "")
  install(
    FILES       "${CMAKE_CURRENT_BINARY_DIR}/qt.conf"
    DESTINATION "lib"
    COMPONENT   superbuild)
endif ()

# this was present in ParaView's paraview.bundle.unix.cmake,
# delete when it is certain that this is not needed for VeloView:
message(STATUS "TOP: qt5_plugin_paths is: ${qt5_plugin_path}")
# foreach (qt5_plugin_path IN LISTS qt5_plugin_paths)
#   get_filename_component(qt5_plugin_group "${qt5_plugin_path}" DIRECTORY)
#   get_filename_component(qt5_plugin_group "${qt5_plugin_group}" NAME)
# 
#   superbuild_unix_install_plugin("${qt5_plugin_path}"
#     "lib"
#     "lib/plugins/${qt5_plugin_group}/"
#     LOADER_PATHS    "${library_paths}"
#     INCLUDE_REGEXES ${include_regexes}
#     EXCLUDE_REGEXES ${exclude_regexes})
# endforeach ()

