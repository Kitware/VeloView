if(CMAKE_CL_64)
  set(wpcap_library_dir ${SuperBuild_BINARY_DIR}/pcap/src/pcap/Lib/x64)
else()
  set(wpcap_library_dir ${SuperBuild_BINARY_DIR}/pcap/src/pcap/Lib)
endif()

# Simply use WinPcap development files from their webside.
add_external_project(pcap
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ${CMAKE_COMMAND}
    -DPCAP_LIBRARY:FILEPATH=${wpcap_library_dir}/wpcap.dll
    -DPACKET_LIBRARY:FILEPATH=${wpcap_library_dir}/packet.dll
    -DPCAP_INSTALL_DIR:PATH=<INSTALL_DIR>/bin
	-P ${SuperBuild_PROJECTS_DIR}/win32/installpcap.cmake
)

add_extra_cmake_args(
  -DPCAP_INCLUDE_DIR:PATH=${SuperBuild_BINARY_DIR}/pcap/src/pcap/Include
  -DPCAP_LIBRARY:FILEPATH=${wpcap_library_dir}/wpcap.lib)

