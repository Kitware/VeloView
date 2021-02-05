# Enable CPack packaging.
set(LidarViewSuperBuild_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
include("${LidarViewSuperBuild_SOURCE_DIR}/../Application/SoftwareInformation/branding.cmake")

# Include CMake scripts for geting the version from Git
include("${LidarViewSuperBuild_SOURCE_DIR}/../LVCore/CMake/Git.cmake")
include("${LidarViewSuperBuild_SOURCE_DIR}/../LVCore/CMake/ParaViewDetermineVersion.cmake")
# Sets LV_VERSION_{MAJOR,MINOR,PATCH} for git
set(LV_VERSION_FILE ${LidarViewSuperBuild_SOURCE_DIR}/../version.txt)
file(STRINGS "${LV_VERSION_FILE}" version_txt)
extract_version_components("${version_txt}" "LV")
determine_version(${LidarViewSuperBuild_SOURCE_DIR} ${GIT_EXECUTABLE} "LV")
# Update the hard-coded version
extract_version_components("${version_txt}" "LV_file")
if((version_txt VERSION_LESS LV_VERSION_FULL)
   OR (version_txt VERSION_EQUAL LV_VERSION_FULL
       AND (LV_file_VERSION_PATCH_EXTRA STRLESS LV_VERSION_PATCH_EXTRA)))
  message(STATUS "Outdated version file updated from ${version_txt} to ${LV_VERSION_FULL} in " ${LV_VERSION_FILE})
  file(WRITE "${LV_VERSION_FILE}" "${LV_VERSION_FULL}")
endif()
if(NOT (version_txt STREQUAL LV_VERSION_FULL))
  message(STATUS "Git version (${LV_VERSION_FULL}) differs from version in file (${version_txt}) at " ${LV_VERSION_FILE})
endif()

# Sets GD_YEAR, GD_MONTH, GD_DAY
include(${LidarViewSuperBuild_SOURCE_DIR}/lidarview-superbuild/Projects/getdate.cmake)
GET_DATE()
set(PACKAGE_TIMESTAMP "${GD_YEAR}${GD_MONTH}${GD_DAY}")

set(CPACK_COMPONENT_LIDARVIEW_DISPLAY_NAME ${SOFTWARE_NAME})
set(CPACK_PACKAGE_VERSION_MAJOR ${LV_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${LV_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${LV_VERSION_PATCH})
if (NOT LV_VERSION_IS_RELEASE)
  set(CPACK_PACKAGE_VERSION_PATCH ${LV_VERSION_PATCH}-${LV_VERSION_PATCH_EXTRA})
else()
endif()

if (NOT LV_VERSION_IS_RELEASE)
  set(CPACK_PACKAGE_FILE_NAME
      "${CPACK_PACKAGE_NAME}-${LV_VERSION_FULL}-${PACKAGE_TIMESTAMP}-${package_suffix}")
else()
  set(CPACK_PACKAGE_FILE_NAME
      "${CPACK_PACKAGE_NAME}-${LV_VERSION_FULL}-${package_suffix}")
endif()
message(STATUS "Bundled package name will be: ${CPACK_PACKAGE_FILE_NAME}" )

# Set the license file.
set(CPACK_RESOURCE_FILE_LICENSE "${LidarViewSuperBuild_SOURCE_DIR}/lidarview-superbuild/LICENSE")

set(lidarview_executables
  "${SOFTWARE_NAME}"
  PacketFileSender
  # "PCAPTester"
	)

set(python_modules
  paraview
  vtk
  vtkmodules
  lidarview
  lidarviewcore)

list(APPEND python_modules
  camera_path
  colormap_tools
  temporal_animation_cue_helpers
  example_temporal_animation
  example_non_temporal_animation)

#auto load eye dome lighting plugin
set(paraview_plugin_EyeDomeLightingView_auto_load ON)

function (lidarview_add_plugin output)
  set(contents "<?xml version=\"1.0\"?>\n<Plugins>\n</Plugins>\n")
  foreach (name IN LISTS ARGN)
    set(auto_load 0)
    if (DEFINED paraview_plugin_${name}_auto_load)
      set(auto_load 1)
    endif ()
    set(plugin_directive "  <Plugin name=\"${name}\" auto_load=\"${auto_load}\" />\n")
    string(REPLACE "</Plugins>" "${plugin_directive}</Plugins>" contents "${contents}")
  endforeach ()
  file(WRITE "${output}" "${contents}")
endfunction ()


# set the relative path in which we can find plugins for each platform 
if (WIN32)
  set(lidarview_plugin_subdir "bin/plugins")
  set(plugin_name_regex "${superbuild_install_location}/${lidarview_plugin_subdir}/*.dll")
elseif (APPLE)
  set(paraview_plugin_subdir "Applications/paraview.app/Contents/Plugins")
  set(lidarview_plugin_subdir "bin/${SOFTWARE_NAME}.app/Contents/Plugins")
  set(plugin_name_regex "${superbuild_install_location}/${paraview_plugin_subdir}/lib*.dylib"
                        "${superbuild_install_location}/${lidarview_plugin_subdir}/lib*.dylib")
elseif (UNIX)
  set(paraview_plugin_subdir "lib/paraview-${PARAVIEW_VERSION}/plugins")
  set(lidarview_plugin_subdir "lib/plugins")
  set(plugin_name_regex "${superbuild_install_location}/${paraview_plugin_subdir}/lib*.so"
                        "${superbuild_install_location}/${lidarview_plugin_subdir}/lib*.so")
endif ()

# get all plugins installed in the lib/lidarview install dir
file(GLOB_RECURSE lidarview_plugin_paths ${plugin_name_regex})

# Get plugins name and set up the .plugins file list
set(lidarview_plugins)
foreach(lidarview_plugin_path ${lidarview_plugin_paths})
  get_filename_component(paraview_plugin "${lidarview_plugin_path}" NAME_WE)

  # remove 'lib' prefix in plugin name
  if(UNIX)
    string(REPLACE "lib" "" paraview_plugin "${paraview_plugin}")
  endif()
  list(APPEND lidarview_plugins ${paraview_plugin})
endforeach()

set(plugins_file "${CMAKE_CURRENT_BINARY_DIR}/.plugins")
lidarview_add_plugin("${plugins_file}" ${lidarview_plugins})

if (qt5_enabled)
  include(qt5.functions)

  set(qt5_plugin_prefix)
  if (NOT WIN32)
    set(qt5_plugin_prefix "lib")
  endif ()

  # Add SVG support, so ParaView can use SVG icons
  set(qt5_plugins
    iconengines/${qt5_plugin_prefix}qsvgicon
    imageformats/${qt5_plugin_prefix}qsvg
    sqldrivers/${qt5_plugin_prefix}qsqlite)

  if (WIN32)
    list(APPEND qt5_plugins
      platforms/qwindows)

    if (NOT qt5_version VERSION_LESS "5.10")
      list(APPEND qt5_plugins
        styles/qwindowsvistastyle)
    endif ()
  elseif (APPLE)
    list(APPEND qt5_plugins
      platforms/libqcocoa
      printsupport/libcocoaprintersupport)

    if (NOT qt5_version VERSION_LESS "5.10")
      list(APPEND qt5_plugins
        styles/libqmacstyle)
    endif ()
  elseif (UNIX)
    list(APPEND qt5_plugins
      platforms/libqxcb
      platforminputcontexts/libcomposeplatforminputcontextplugin
      xcbglintegrations/libqxcb-glx-integration)
  endif ()

  superbuild_install_qt5_plugin_paths(qt5_plugin_paths ${qt5_plugins})
else ()
  set(qt5_plugin_paths)
endif ()

