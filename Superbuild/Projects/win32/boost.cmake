add_external_project_or_use_system(boost
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND
    <SOURCE_DIR>/bootstrap.bat
  BUILD_COMMAND <SOURCE_DIR>/b2
    address-model=${VV_BUILD_ARCHITECTURE} --prefix=<INSTALL_DIR> --with-regex --with-system --with-date_time --with-program_options --with-iostreams --with-filesystem --with-thread install
  INSTALL_COMMAND ""
)
