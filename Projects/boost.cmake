add_external_project(boost
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND
    <SOURCE_DIR>/bootstrap.sh --prefix=<INSTALL_DIR>
                              --with-libraries=date_time,thread
  BUILD_COMMAND <SOURCE_DIR>/bjam
  INSTALL_COMMAND <SOURCE_DIR>/bjam --prefix=<INSTALL_DIR> install
)
