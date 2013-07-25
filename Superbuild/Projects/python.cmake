if (APPLE)
  message(FATAL_ERROR "ABORT")
endif()

add_external_project_or_use_system(python
  DEPENDS png
  CONFIGURE_COMMAND <SOURCE_DIR>/configure
                    --prefix=<INSTALL_DIR>
                    --enable-unicode
                    --enable-shared
  )
set (pv_python_executable "${install_location}/bin/python" CACHE INTERNAL "" FORCE)
