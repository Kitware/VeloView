add_executable(TestReader TestReader.cxx)
target_link_libraries(TestReader VelodyneHDLPlugin)

add_executable(TestPositionReader TestPositionReader.cxx)
target_link_libraries(TestPositionReader VelodyneHDLPlugin)

add_executable(TestSensor TestSensor.cxx)
target_link_libraries(TestSensor VelodyneHDLPlugin)

add_test(TestReader1 ${CMAKE_BINARY_DIR}/bin/TestPositionReader ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap 299)

add_test(TestReaderMonterey
  ${CMAKE_BINARY_DIR}/bin/TestReader
  ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap
  ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
  ${CMAKE_SOURCE_DIR}/Testing/monterey.test
)

add_test(TestReaderMontereyWithCrop
  ${CMAKE_BINARY_DIR}/bin/TestReader
  ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap
  ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
  ${CMAKE_SOURCE_DIR}/Testing/monterey-with-crop.test
)

add_test(TestReaderDualReturn
  ${CMAKE_BINARY_DIR}/bin/TestReader
  ${CMAKE_SOURCE_DIR}/Testing/Data/dual-return-frames-5-to-15.pcap
  ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
  ${CMAKE_SOURCE_DIR}/Testing/dual.test
)

add_test(TestReaderVLP16
  ${CMAKE_BINARY_DIR}/bin/TestReader
  ${CMAKE_SOURCE_DIR}/Testing/Data/vlp-16-test-90deg-1-50-frames.pcap
  ${CMAKE_SOURCE_DIR}/share/VLP-16.xml
  ${CMAKE_SOURCE_DIR}/Testing/vlp-16.test
)

add_test(TestSensor
  ${CMAKE_BINARY_DIR}/bin/TestSensor
  ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap
  ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
  )
