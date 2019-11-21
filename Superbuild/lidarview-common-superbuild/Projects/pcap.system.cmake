find_library(PCAP_LIBRARY pcap DOC "pcap library")
find_path(PCAP_INCLUDE_DIR pcap.h DOC "pcap include directory")
mark_as_advanced(PCAP_LIBRARY PCAP_INCLUDE_DIR)

superbuild_add_extra_cmake_args(
  -DPCAP_LIBRARY:FILEPATH=${PCAP_LIBRARY}
  -DPCAP_INCLUDE_DIR:PATH=${PCAP_INCLUDE_DIR}
  )
