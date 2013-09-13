add_external_project(eigen
  BUILD_IN_SOURCE 0
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>/Eigen <INSTALL_DIR>/eigen/Eigen
)
