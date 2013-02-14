execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different
  "${SOURCE_DIR}/objs/freetype.lib"
  "${INSTALL_DIR}/lib/freetype.lib"
)
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_directory
  "${SOURCE_DIR}/include"
  "${INSTALL_DIR}/include/freetype2"
)
