message("Installing pcap")

file(INSTALL ${PCAP_LIBRARY} ${PACKET_LIBRARY}
     DESTINATION ${PCAP_INSTALL_DIR})

message("Done pcap install")