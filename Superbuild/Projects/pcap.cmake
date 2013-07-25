add_external_project_or_use_system(pcap
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND <SOURCE_DIR>/configure
                    --prefix=<INSTALL_DIR>
  BUILD_COMMAND make
)
