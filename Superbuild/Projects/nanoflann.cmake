superbuild_add_project(nanoflann
  DEPENDS eigen

  CMAKE_ARGS
  -DBUILD_EXAMPLES=OFF
  -DBUILD_TESTS=OFF
  -DEIGEN3_DIR=<INSTALL_DIR>/include/eigen3
  -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>/nanoflann
  )
