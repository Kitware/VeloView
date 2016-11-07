add_external_project(liblas
  DEPENDS boost
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
)
