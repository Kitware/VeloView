add_external_project(boost
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND
    <SOURCE_DIR>/bootstrap.sh --prefix=<INSTALL_DIR>
                              --with-libraries=date_time,thread,regex,system
  BUILD_COMMAND <SOURCE_DIR>/bjam address-model=64 threading=multi
  INSTALL_COMMAND <SOURCE_DIR>/bjam  address-model=64 threading=multi --prefix=<INSTALL_DIR> install
)
