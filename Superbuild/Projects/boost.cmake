set (boost_toolset "")
set (boost_osx_opts "")
if (APPLE)
  set (boost_toolset "--with-toolset=clang")
  set (boost_osx_opts "toolset=clang cxxflags=\"-mmacosx-version-min=${CMAKE_OSX_DEPLOYMENT_TARGET}\" cflags=\"-mmacosx-version-min=${CMAKE_OSX_DEPLOYMENT_TARGET}\"")
  message(STATUS "${boost_osx_opts}")
 #macosx-version=${CMAKE_OSX_DEPLOYMENT_TARGET} macosx-version-min=${CMAKE_OSX_DEPLOYMENT_TARGET}")
endif(APPLE)
add_external_project_or_use_system(boost
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND
    <SOURCE_DIR>/bootstrap.sh ${boost_toolset} --prefix=<INSTALL_DIR>
                              --with-libraries=date_time,thread,regex,system,program_options,filesystem,iostreams,chrono
  BUILD_COMMAND <SOURCE_DIR>/bjam address-model=${VV_BUILD_ARCHITECTURE} threading=multi --with-regex --with-system --with-date_time --with-program_options --with-iostreams --with-filesystem --with-thread --with-chrono ${boost_osx_opts}  --prefix=<INSTALL_DIR> install
  INSTALL_COMMAND ""
)
