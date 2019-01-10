# Some advices when debugging:
# - get help by looking at:
#   (mainly) https://gitlab.kitware.com/paraview/paraview-superbuild/blob/master/projects/apple/paraview.bundle.cmake
#   (also) https://gitlab.kitware.com/cmb/cmb-superbuild/blob/master/projects/apple/cmb.bundle.apple.cmake
# - always clean the package-related directories of the build tree before retrying:
#   rm -rf _CPack_Packages && rm -rf cpack && cmake .
#   then try with: ctest -R cpack -V > package.log (very useful to grep the log)
# - to test if the application has correct paths and patched executable/libraries:
#   inside build directory:
#   `open _CPack_Packages/Darwin/DragNDrop/VeloView-3.6.0-196-g3490e1d-20190109-Darwin-64bit/veloview.app`
#   (much, much faster than having to mount the .dmg using the finder)
# - testing the .dmg on another computer (that does not have Qt) is important because that is our target
# - testing the .dmg on another computer + executing VeloView from terminal
#   (with `/Volume/something.dmg/veloview.app/Contents/MacOS/VeloView`) will sometime give you much
#   very clear indications about absolute path that have not been fixed / missing files
# - to have a hint at "should this file really by inside the .dmg ?", look inside Paraview's .dmg
#   or (better) recent working VeloView's .dmg
# - do not hesistate to hack inside ../common-superbuild/cmake/scripts/fixup_bundle.apple.py
#   to print/modify/add paths, then get rid of your hack using only this file
#   (we do not want to modify the common-superbuild)

include(veloview.bundle.common)
include(${VeloViewSuperBuild_SOURCE_DIR}/../SoftwareInformation/branding.cmake)

# the variable veloview_appname:
# - must be a valid dirname: will be a directory at the top of the .dmg
# - is visible in the macOS GUI when opening the .dmg
# - MUST end with .app (else its tree is not considered as an app by macOS)
set(veloview_appname "${SOFTWARE_NAME}.app")
set(veloview_executables "${SOFTWARE_NAME}")

# VeloView is based on ParaView and can load ParaView plugins
set(paraview_plugin_path "bin/${veloview_appname}/Contents/Plugins")

# this must be done before calling superbuild_apple_create_app,
# because superbuild_apple_create_app uses paraview_plugin_paths
# this was copied from ParaView's superbuild
# TODO: could/should be inside veloview.bundle.common ?
set(paraview_plugins "PointCloudPlugin")
set(paraview_plugin_paths)
foreach (paraview_plugin IN LISTS paraview_plugins)
  if (EXISTS "${superbuild_install_location}/Applications/paraview.app/Contents/Plugins/lib${paraview_plugin}.dylib")
    list(APPEND paraview_plugin_paths
	    "${superbuild_install_location}/Applications/paraview.app/Contents/Plugins/lib${paraview_plugin}.dylib")
    continue ()
  endif ()

  foreach (path IN ITEMS "" "paraview-${paraview_version}")
    if (EXISTS "${superbuild_install_location}/lib/${path}/lib${paraview_plugin}.dylib")
      list(APPEND paraview_plugin_paths
        "${superbuild_install_location}/lib/${path}/lib${paraview_plugin}.dylib")
      break ()
    endif ()
  endforeach ()
endforeach ()


superbuild_apple_create_app(
  "\${CMAKE_INSTALL_PREFIX}"
  "${veloview_appname}"
  "${superbuild_install_location}/bin/${veloview_appname}/Contents/MacOS/${SOFTWARE_NAME}"
  CLEAN
  PLUGINS ${paraview_plugin_paths}
  SEARCH_DIRECTORIES "${superbuild_install_location}/lib" "${superbuild_install_location}/bin/${veloview_appname}/Contents/Libraries"
  INCLUDE_REGEXES     ${include_regexes})


function (paraview_add_plugin output)
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
set(plugins_file "${CMAKE_CURRENT_BINARY_DIR}/veloview.plugins")
paraview_add_plugin("${plugins_file}" ${paraview_plugins})

install(
  FILES       "${plugins_file}"
  DESTINATION "${veloview_appname}/Contents/Plugins"
  COMPONENT   superbuild
  RENAME      ".plugins")

install(
  FILES       "${superbuild_install_location}/Applications/paraview.app/Contents/Resources/pvIcon.icns"
  DESTINATION "${veloview_appname}/Contents/Resources"
  COMPONENT   superbuild)
install(
  FILES       "${superbuild_install_location}/bin/${veloview_appname}/Contents/Info.plist"
  DESTINATION "${veloview_appname}/Contents"
  COMPONENT   superbuild)

# Remove "VeloView" from the list since we just installed it above.
list(REMOVE_ITEM veloview_executables
  ${SOFTWARE_NAME})

foreach (executable IN LISTS veloview_executables)
  superbuild_apple_install_utility(
    "\${CMAKE_INSTALL_PREFIX}"
    "${veloview_appname}"
    "${superbuild_install_location}/bin/${veloview_appname}/Contents/bin/${executable}"
    SEARCH_DIRECTORIES "${superbuild_install_location}/lib"
    INCLUDE_REGEXES     ${include_regexes})
endforeach ()

if (qt5_enabled)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/qt.conf" "[Paths]\nPlugins = Plugins\n")
  install(
    FILES       "${CMAKE_CURRENT_BINARY_DIR}/qt.conf"
    DESTINATION "${veloview_appname}/Contents/Resources"
    COMPONENT   superbuild)
endif ()

if (python_enabled)
  superbuild_apple_install_python(
    "\${CMAKE_INSTALL_PREFIX}"
    "${veloview_appname}"
    MODULES paraview
    	    veloview
	    vtk
            vtkmodules
            ${python_modules}
    MODULE_DIRECTORIES
            "${superbuild_install_location}/bin/${veloview_appname}/Contents/Python"
            "${superbuild_install_location}/lib/python2.7/site-packages"
    SEARCH_DIRECTORIES
            "${superbuild_install_location}/bin/${veloview_appname}/Contents/Libraries"
            "${superbuild_install_location}/lib")

  if (matplotlib_enabled)
    install(
      DIRECTORY   "${superbuild_install_location}/lib/python2.7/site-packages/matplotlib/mpl-data/"
      DESTINATION "${veloview_appname}/Contents/Python/matplotlib/mpl-data"
      COMPONENT   superbuild)
  endif ()
endif ()

# For some reason these .so files are not processed by the command
# superbuild_apple_install_python above, so we have to specify them manually
# it could be that I failed to find the correct name(s) to add in parameter
# "MODULE" but I do not think so because there are 86 such files, and because
# they seem to be part of vtk which is already specified like that in ParaView
file(GLOB missing_python_so "${superbuild_install_location}/bin/${veloview_appname}/Contents/Libraries/vtk*Python.so")
foreach (python_so ${missing_python_so})
  superbuild_apple_install_module(
    "\${CMAKE_INSTALL_PREFIX}"
    "${veloview_appname}"
    "${python_so}"
    "Contents/Libraries") # destination path inside bundle
endforeach()

# My understanding is that these module are not processed automatically
# by superbuild_apple_create_app because there is no path leading to
# them in binary VeloView or in any of its .dylib dependencies
set(my_modules)
list(APPEND my_modules "VelodyneHDLPluginPython.so")
list(APPEND my_modules "libVelodyneHDLPluginPythonD.dylib")
foreach (module ${my_modules})
  superbuild_apple_install_module(
    "\${CMAKE_INSTALL_PREFIX}"
    "${veloview_appname}"
    "${superbuild_install_location}/bin/${veloview_appname}/Contents/Libraries/${module}"
    "Contents/Libraries") # destination path inside bundle
endforeach()

# Configure CMakeDMGSetup.scpt to replace the app name in the script.
configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/files/CMakeDMGSetup.scpt.in"
  "${CMAKE_CURRENT_BINARY_DIR}/CMakeDMGSetup.scpt"
  @ONLY)

set(CPACK_DMG_BACKGROUND_IMAGE "${CMAKE_CURRENT_LIST_DIR}/files/CMakeDMGBackground.tif")
set(CPACK_DMG_DS_STORE_SETUP_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/CMakeDMGSetup.scpt")

message(STATUS "qt5_plugin_paths is ${qt5_plugin_paths}")
foreach (qt5_plugin_path IN LISTS qt5_plugin_paths)
  get_filename_component(qt5_plugin_group "${qt5_plugin_path}" DIRECTORY)
  get_filename_component(qt5_plugin_group "${qt5_plugin_group}" NAME)

  superbuild_apple_install_module(
    "\${CMAKE_INSTALL_PREFIX}"
    "${veloview_appname}"
    "${qt5_plugin_path}"
    "Contents/Plugins/${qt5_plugin_group}"
    SEARCH_DIRECTORIES  "${library_paths}")
endforeach ()


install(DIRECTORY "${superbuild_install_location}/bin/${veloview_appname}/Contents/Resources"
  DESTINATION "${veloview_appname}/Contents"
  COMPONENT superbuild)
