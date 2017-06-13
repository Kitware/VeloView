add_executable(TestReader TestReader.cxx)
target_link_libraries(TestReader VelodyneHDLPlugin)

add_executable(TestPositionReader TestPositionReader.cxx)
target_link_libraries(TestPositionReader VelodyneHDLPlugin)

add_executable(TestSensor TestSensor.cxx)
target_link_libraries(TestSensor VelodyneHDLPlugin)

add_executable(TestReaderVTP TestReaderVTP.cxx TestHelpers.cxx)
target_link_libraries(TestReaderVTP VelodyneHDLPlugin)

add_executable(TestLiveStream TestHelpers.cxx TestLiveStream.cxx)
target_link_libraries(TestLiveStream VelodyneHDLPlugin)

# On windows, install the tests in the install/bin
# folder. Otherwise, the .exe tests can't find the required
# dll (Velodyne plugins and vtk)
if (WIN32)
    set(INSTALL_LOCAL_DIR "${CMAKE_BINARY_DIR}/../../../install/bin")
    install(TARGETS TestReader
          RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

    install(TARGETS TestPositionReader
          RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

    install(TARGETS TestSensor
          RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

    install(TARGETS TestReaderVTP
          RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

    install(TARGETS TestLiveStream
          RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

    add_test(TestReader1
             ${INSTALL_LOCAL_DIR}/TestPositionReader
             ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap 299
             WORKING_DIRECTORY ${INSTALL_LOCAL_DIR})

    add_test(TestReaderMonterey
      ${INSTALL_LOCAL_DIR}/TestReader
      ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap
      ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
      ${CMAKE_SOURCE_DIR}/Testing/monterey.test
      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
    )

    add_test(TestReaderMontereyWithCrop
      ${INSTALL_LOCAL_DIR}/TestReader
      ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap
      ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
      ${CMAKE_SOURCE_DIR}/Testing/monterey-with-crop.test
      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
    )

    add_test(TestReaderDualReturn
      ${INSTALL_LOCAL_DIR}/TestReader
      ${CMAKE_SOURCE_DIR}/Testing/Data/dual-return-frames-5-to-15.pcap
      ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
      ${CMAKE_SOURCE_DIR}/Testing/dual.test
      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
    )

    add_test(TestReaderVLP16
      ${INSTALL_LOCAL_DIR}/TestReader
      ${CMAKE_SOURCE_DIR}/Testing/Data/vlp-16-test-90deg-1-50-frames.pcap
      ${CMAKE_SOURCE_DIR}/share/VLP-16.xml
      ${CMAKE_SOURCE_DIR}/Testing/vlp-16.test
      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
    )

    add_test(TestReaderVTP
      ${INSTALL_LOCAL_DIR}/TestReaderVTP
      ${CMAKE_SOURCE_DIR}/Testing/Data/
      ${CMAKE_SOURCE_DIR}/share/
      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
    )

    add_test(TestLiveStream
      ${INSTALL_LOCAL_DIR}/TestLiveStream
      ${CMAKE_SOURCE_DIR}/Testing/Data/
      ${CMAKE_SOURCE_DIR}/share/
      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
    )

    add_test(TestSensor
      ${INSTALL_LOCAL_DIR}/TestSensor
      ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap
      ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
    )

else()
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

    add_test(TestReaderVTP
      ${CMAKE_BINARY_DIR}/bin/TestReaderVTP
      ${CMAKE_SOURCE_DIR}/Testing/Data/
      ${CMAKE_SOURCE_DIR}/share/
    )

    add_test(TestLiveStream
      ${CMAKE_BINARY_DIR}/bin/TestLiveStream
      ${CMAKE_SOURCE_DIR}/Testing/Data/
      ${CMAKE_SOURCE_DIR}/share/
    )

    add_test(TestSensor
      ${CMAKE_BINARY_DIR}/bin/TestSensor
      ${CMAKE_SOURCE_DIR}/Testing/Data/monterey-first-100-packets.pcap
      ${CMAKE_SOURCE_DIR}/share/HDL-32.xml
    )
endif()