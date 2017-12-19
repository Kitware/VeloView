superbuild_add_project(pcap
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND <SOURCE_DIR>/configure
                    --prefix=<INSTALL_DIR>
  BUILD_COMMAND make
  CAN_USE_SYSTEM
)
