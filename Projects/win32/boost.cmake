add_external_project(boost
  DEPENDS zlib
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND
    <SOURCE_DIR>/bootstrap.bat
  BUILD_COMMAND <SOURCE_DIR>/b2 --prefix=<INSTALL_DIR> --with-date_time install
  INSTALL_COMMAND ""
)
