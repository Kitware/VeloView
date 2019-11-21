superbuild_add_project(eigen
  PATCH_COMMAND
    ${CMAKE_COMMAND} -E copy_if_different
    ${SuperBuild_PROJECTS_DIR}/patches/eigen.cmake.language_support.cmake
    <SOURCE_DIR>/cmake/language_support.cmake
  CMAKE_ARGS
    -DEIGEN_BUILD_PKGCONFIG=off
  )
