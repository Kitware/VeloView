message(STATUS "Using system pcap.")
add_external_project_or_use_system(pcap)
set(USE_SYSTEM_pcap TRUE CACHE BOOL "" FORCE)
