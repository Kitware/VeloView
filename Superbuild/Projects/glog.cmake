superbuild_add_project(glog
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>/glog
    -DBUILD_TESTING=false
    -DWITH_GFLAGS=false
)
