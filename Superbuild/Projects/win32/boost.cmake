
set(boost_address_model 32)
if (64bit_build)
  set(boost_address_model 64)
endif()

add_external_project_or_use_system(boost
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND
    <SOURCE_DIR>/bootstrap.bat
  BUILD_COMMAND <SOURCE_DIR>/b2
    toolset=msvc-9.0 address-model=${boost_address_model} --prefix=<INSTALL_DIR> --with-regex --with-system --with-date_time --with-program_options --with-iostreams --with-filesystem --with-thread install
  INSTALL_COMMAND ""
)
