add_external_project(liblas
  DEPENDS boost
  PATCH_COMMAND
    git apply --ignore-space-change ${SuperBuild_PROJECTS_DIR}/patches/liblas.uint64.boost.patch
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
)
