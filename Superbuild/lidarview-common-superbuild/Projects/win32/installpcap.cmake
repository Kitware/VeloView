message("Installing pcap")

file(INSTALL ${PCAP_LIBRARY} ${PACKET_LIBRARY}
     DESTINATION "${PCAP_INSTALL_DIR}/bin")

file(GLOB include_files "${PCAP_INCLUDE_DIR}/*.h")
file(INSTALL ${include_files}
	DESTINATION "${PCAP_INSTALL_DIR}/include")

file(GLOB include_files "${PCAP_INCLUDE_DIR}/pcap/*.h")
file(INSTALL ${include_files}
	DESTINATION "${PCAP_INSTALL_DIR}/include/pcap")

message("Done pcap install")