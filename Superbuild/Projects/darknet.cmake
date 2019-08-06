superbuild_add_project(darknet
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND sed -i "s/GPU=0/GPU=1/g" Makefile # enable GPU
  BUILD_COMMAND make
  INSTALL_COMMAND cp libdarknet.so <INSTALL_DIR>/lib  # copy lib
)
