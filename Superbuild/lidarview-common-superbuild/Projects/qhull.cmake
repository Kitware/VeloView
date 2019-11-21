## Builds qhull from git://gitorious.org/qhull/qhull.git

if(CMAKE_BUILD_TYPE MATCHES "[dD]ebug")
    set(qhull_STATIC qhullstatic_d)
else()
    set(qhull_STATIC qhullstatic)
endif()
superbuild_add_project(qhull
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>/qhull
    -Dqhull_TARGETS_INSTALL=${qhull_STATIC}
)
