set (configuration)
set (python_executable_dir)
if (64bit_build)
  set (configuration "Release|x64")
  set (python_executable_dir
    "${SuperBuild_BINARY_DIR}/python/src/python/PCbuild/amd64")
else()
  set (configuration "Release|Win32")
  set (python_executable_dir
    "${SuperBuild_BINARY_DIR}/python/src/python/PCbuild")
endif()


#------------------------------------------------------------------------------
# in the following build commands, we use devenv explicitly since the generator
# the user has chosen could be nmake, in which case CMAKE_BUILD_TOOL is not the
# right tool. Since devenv is in the path in the nmake build environment as well
# as VS environment, we can safely call it.
add_external_project_or_use_system(python
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ${DEVENV_PATH} PCbuild/pcbuild.sln /build ${configuration}
                                    /project python
  #devenv doesn't seem to building all specified projects when I list them in
  #same command line. So making them separate calls.

  # We need to copy pyconfig.h from PC/ to Include.
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_if_different
                  <SOURCE_DIR>/PC/pyconfig.h
                  <SOURCE_DIR>/Include/pyconfig.h
)

#------------------------------------------------------------------------------
set (python_projects_to_build
  select
  make_versioninfo
  make_buildinfo
  kill_python
  w9xpopen
  pythoncore
  _socket
  _testcapi
  _msi
  _elementtree
  _ctypes_test
  _ctypes
  winsound
  pyexpat
  _multiprocessing
  pythonw
  unicodedata
)

foreach(dep IN LISTS python_projects_to_build)
  add_external_project_step(python-project-${dep}
    COMMAND ${DEVENV_PATH} <SOURCE_DIR>/PCbuild/pcbuild.sln /build ${configuration}
                                /project ${dep}
    DEPENDEES build
    DEPENDERS install)
endforeach()

#------------------------------------------------------------------------------
set (pv_python_executable "${python_executable_dir}/python.exe")
add_extra_cmake_args(
  -DPYTHON_EXECUTABLE:FILEPATH=${python_executable_dir}/python.exe
  -DPYTHON_INCLUDE_DIR:FILEPATH=${SuperBuild_BINARY_DIR}/python/src/python/Include
  -DPYTHON_LIBRARY:FILEPATH=${python_executable_dir}/python27.lib)
