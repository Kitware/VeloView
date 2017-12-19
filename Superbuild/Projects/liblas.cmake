#[[
 	This is a copy from paraview superbuild project :
 	https://gitlab.kitware.com/paraview/paraview-superbuild/blob/c8006d9a96eaa06228a011bc8d646541106643ec/projects/las.cmake
]]

superbuild_add_project(liblas
  DEPENDS boost
  CMAKE_ARGS
    -DWITH_GDAL:BOOL=FALSE
    -DBUILD_OSGEO4W:BOOL=OFF
    -DWITH_GEOTIFF:BOOL=FALSE
    -DWITH_LASZIP:BOOL=FALSE
    -DWITH_TESTS:BOOL=FALSE
    -DWITH_UTILITIES:BOOL=FALSE
    -DBoost_USE_STATIC_LIBS:BOOL=FALSE)

# this patch is commited upstream at 4dbc30a7e7e099cbe01a7c192ec19d231cc26894
superbuild_apply_patch(liblas respect-with-geotiff
  "find_package GeoTIFF only if WITH_GEOTIFF")

superbuild_apply_patch(liblas enable-outside-boost-options
  "Enable outside boost options")

superbuild_apply_patch(liblas add-boost-include-dirs
  "Boost include dirs are needed on Windows")

if (WIN32)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_NO_LIB" PROJECT_ONLY)
endif()

if (APPLE)
  superbuild_append_flags(cxx_flags "-stdlib=libc++" PROJECT_ONLY)
  superbuild_append_flags(ld_flags "-stdlib=libc++" PROJECT_ONLY)
endif ()
