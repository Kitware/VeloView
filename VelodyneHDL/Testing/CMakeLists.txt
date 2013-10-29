add_executable(TestReader TestReader.cxx)
target_link_libraries(TestReader VelodyneHDLPlugin)

add_executable(TestPositionReader TestPositionReader.cxx)
target_link_libraries(TestPositionReader VelodyneHDLPlugin)

add_test(TestReader1 ${CMAKE_BINARY_DIR}/bin/TestPositionReader ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap 299)
