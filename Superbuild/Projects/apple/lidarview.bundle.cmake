# Some advices when debugging:
# - get help by looking at:
#   (mainly) https://gitlab.kitware.com/paraview/paraview-superbuild/blob/master/projects/apple/paraview.bundle.cmake
#   (also) https://gitlab.kitware.com/cmb/cmb-superbuild/blob/master/projects/apple/cmb.bundle.apple.cmake
# - always clean the package-related directories of the build tree before retrying:
#   rm -rf _CPack_Packages && rm -rf cpack && cmake .
#   then try with: ctest -R cpack -V > package.log (very useful to grep the log)
# - to test if the application has correct paths and patched executable/libraries:
#   inside build directory:
#   `open _CPack_Packages/Darwin/DragNDrop/LidarView-3.6.0-196-g3490e1d-20190109-Darwin-64bit/lidarview.app`
#   (much, much faster than having to mount the .dmg using the finder)
# - testing the .dmg on another computer (that does not have Qt) is important because that is our target
# - testing the .dmg on another computer + executing LidarView from terminal
#   (with `/Volume/something.dmg/lidarview.app/Contents/MacOS/LidarView`) will sometime give you much
#   very clear indications about absolute path that have not been fixed / missing files
# - to have a hint at "should this file really by inside the .dmg ?", look inside Paraview's .dmg
#   or (better) recent working LidarView's .dmg
# - do not hesistate to hack inside ../common-superbuild/cmake/scripts/fixup_bundle.apple.py
#   to print/modify/add paths, then get rid of your hack using only this file
#   (we do not want to modify the common-superbuild)

include(lidarview.bundle.common)
include("${LidarViewSuperBuild_SOURCE_DIR}/../Application/SoftwareInformation/branding.cmake")

# the variable lidarview_appname:
# - must be a valid dirname: will be a directory at the top of the .dmg
# - is visible in the macOS GUI when opening the .dmg
# - MUST end with .app (else its tree is not considered as an app by macOS)
set(lidarview_appname "${SOFTWARE_NAME}.app")

superbuild_apple_create_app(
  "\${CMAKE_INSTALL_PREFIX}"
  "${lidarview_appname}"
  "${superbuild_install_location}/bin/${lidarview_appname}/Contents/MacOS/${SOFTWARE_NAME}"
  CLEAN
  PLUGINS ${paraview_plugin_paths}
  SEARCH_DIRECTORIES "${superbuild_install_location}/lib" "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Libraries")

install(
  FILES       "${plugins_file}"
  DESTINATION "${lidarview_appname}/Contents/Plugins"
  COMPONENT   superbuild)
install(
  FILES       "${superbuild_install_location}/Applications/paraview.app/Contents/Resources/pvIcon.icns"
  DESTINATION "${lidarview_appname}/Contents/Resources"
  COMPONENT   superbuild)
install(
  FILES       "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Info.plist"
  DESTINATION "${lidarview_appname}/Contents"
  COMPONENT   superbuild)

# Remove "LidarView" from the list since we just installed it above.
list(REMOVE_ITEM lidarview_executables
  ${SOFTWARE_NAME})

foreach (executable IN LISTS lidarview_executables)
  superbuild_apple_install_utility(
    "\${CMAKE_INSTALL_PREFIX}"
    "${lidarview_appname}"
    "${superbuild_install_location}/bin/${lidarview_appname}/Contents/bin/${executable}"
    SEARCH_DIRECTORIES "${superbuild_install_location}/lib")
endforeach ()

if (qt5_enabled)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/qt.conf" "[Paths]\nPlugins = Plugins\n")
  install(
    FILES       "${CMAKE_CURRENT_BINARY_DIR}/qt.conf"
    DESTINATION "${lidarview_appname}/Contents/Resources"
    COMPONENT   superbuild)
endif ()

if (python3_enabled)
  # install python modules
  if (python3_built_by_superbuild)
    include(python3.functions)
    superbuild_install_superbuild_python3(BUNDLE "${lidarview_appname}")
  endif()

  superbuild_apple_install_python(
  "\${CMAKE_INSTALL_PREFIX}"
  "${lidarview_appname}"
  MODULES ${python_modules}
  MODULE_DIRECTORIES
          "${superbuild_install_location}/Applications/paraview.app/Contents/Python"
          "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Python"
          "${superbuild_install_location}/lib/python${superbuild_python_version}/site-packages"
  SEARCH_DIRECTORIES
          "${superbuild_install_location}/Applications/paraview.app/Contents/Libraries"
          "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Libraries"
          "${superbuild_install_location}/lib")

endif ()

# For some reason these .so files are not processed by the command
# superbuild_apple_install_python above, so we have to specify them manually
# it could be that I failed to find the correct name(s) to add in parameter
# "MODULE" but I do not think so because there are 86 such files, and because
# they seem to be part of vtk which is already specified like that in ParaView
file(GLOB missing_python_so "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Libraries/vtk*Python.so")
foreach (python_so ${missing_python_so})
  superbuild_apple_install_module(
    "\${CMAKE_INSTALL_PREFIX}"
    "${lidarview_appname}"
    "${python_so}"
    "Contents/Libraries") # destination path inside bundle
endforeach()

# My understanding is that these module are not processed automatically
# by superbuild_apple_create_app because there is no path leading to
# them in binary LidarView or in any of its .dylib dependencies
set(my_modules)
list(APPEND my_modules "LidarPluginPython.so")
list(APPEND my_modules "libLidarPluginPythonD.dylib")
list(APPEND my_modules "VelodynePluginPython.so")
list(APPEND my_modules "libVelodynePluginPythonD.dylib")
foreach (module ${my_modules})
  superbuild_apple_install_module(
    "\${CMAKE_INSTALL_PREFIX}"
    "${lidarview_appname}"
    "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Libraries/${module}"
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
    "${lidarview_appname}"
    "${qt5_plugin_path}"
    "Contents/Plugins/${qt5_plugin_group}"
    SEARCH_DIRECTORIES  "${library_paths}")
endforeach ()


install(DIRECTORY "${superbuild_install_location}/bin/${lidarview_appname}/Contents/Resources"
  DESTINATION "${lidarview_appname}/Contents"
  COMPONENT superbuild)
