add_executable(TestVelodyneHDLSource TestHelpers.cxx TestVelodyneHDLSource.cxx)
target_include_directories(TestVelodyneHDLSource PRIVATE ${plugin_include_dirs})
target_link_libraries(TestVelodyneHDLSource LINK_PUBLIC VelodyneHDLPlugin)

add_executable(TestVelodyneHDLReader TestVelodyneHDLReader.cxx TestHelpers.cxx)
target_include_directories(TestVelodyneHDLReader PRIVATE ${plugin_include_dirs})
target_link_libraries(TestVelodyneHDLReader LINK_PUBLIC VelodyneHDLPlugin)

add_executable(TestVelodyneHDLPositionReader TestVelodyneHDLPositionReader.cxx)
target_link_libraries(TestVelodyneHDLPositionReader VelodyneHDLPlugin)

add_executable(TestNMEAParser TestNMEAParser.cxx TestHelpers.cxx)
target_link_libraries(TestNMEAParser VelodyneHDLPlugin)

set(sensors "HDL-64"
            "VLP-16"
            "VLP-32c")

if (WIN32)
    set(INSTALL_LOCAL_DIR "${CMAKE_BINARY_DIR}/../../../install/bin")

    install(TARGETS TestVelodyneHDLSource
                RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
                LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
                COMPONENT Runtime)

    install(TARGETS TestVelodyneHDLReader
                RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
                LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
                COMPONENT Runtime)
else()
    set(INSTALL_LOCAL_DIR "${CMAKE_BINARY_DIR}/bin")
endif()

# generate script to generate new test baseline
configure_file(generateTestData.py.in ${INSTALL_LOCAL_DIR}/generateTestData.py)

# add test
foreach(sensor ${sensors})

    # VelodyneHDLSource test
    add_test(TestVelodyneHDLSource_${sensor}_Single
      ${INSTALL_LOCAL_DIR}/TestVelodyneHDLSource
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single.pcap
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single/files.txt
      ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
    )
    add_test(TestVelodyneHDLSource_${sensor}_Dual
      ${CMAKE_BINARY_DIR}/bin/TestVelodyneHDLSource
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual.pcap
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual/files.txt
      ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
    )

    # VelodyneHDLReader test
    add_test(TestVelodyneHDLReader_${sensor}_Single
      ${INSTALL_LOCAL_DIR}/TestVelodyneHDLReader
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single.pcap
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single/files.txt
      ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
    )
    add_test(TestVelodyneHDLReader_${sensor}_Dual
      ${INSTALL_LOCAL_DIR}/TestVelodyneHDLReader
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual.pcap
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual/files.txt
      ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
    )

endforeach(sensor)

# add special test for HDL-64 in autocalibration mode
# VelodyneHDLSource test
add_test(TestVelodyneHDLSource_HDL-64_Single-autocalib
  ${INSTALL_LOCAL_DIR}/TestVelodyneHDLSource
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib.pcap
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib/files.txt
  ""
)
add_test(TestVelodyneHDLSource_HDL-64_Dual-autocalib
  ${INSTALL_LOCAL_DIR}/TestVelodyneHDLSource
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib.pcap
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib/files.txt
  ""
)

# VelodyneHDLReader test
add_test(TestVelodyneHDLReader_HDL-64_Single-autocalib
  ${INSTALL_LOCAL_DIR}/TestVelodyneHDLReader
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib.pcap
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib/files.txt
  ""
)
add_test(TestVelodyneHDLReader_HDL-64_Dual-autocalib
  ${INSTALL_LOCAL_DIR}/TestVelodyneHDLReader
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib.pcap
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib/files.txt
  ""
)

add_test(TestVelodyneHDLPositionReader
  ${INSTALL_LOCAL_DIR}/TestVelodyneHDLPositionReader
  "${CMAKE_SOURCE_DIR}/TestData/HDL32-V2_R_into_Butterfield_into_Digital_Drive.pcap"
)

add_test(TestNMEAParser
  ${INSTALL_LOCAL_DIR}/TestNMEAParser
)
