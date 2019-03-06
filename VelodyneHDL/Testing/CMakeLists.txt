if (WIN32)
  set(INSTALL_LOCAL_DIR "${CMAKE_BINARY_DIR}/../../../install/bin")
else()
  set(INSTALL_LOCAL_DIR "${CMAKE_BINARY_DIR}/bin")
endif()

function(custom_add_executable)
  add_executable(${ARGV})
  if (WIN32)
    install(TARGETS ${ARGV0}
	    RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
	    LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
	    COMPONENT Runtime)
  endif(WIN32)
endfunction(custom_add_executable)

custom_add_executable(TestVelodyneHDLSource TestHelpers.cxx TestVelodyneHDLSource.cxx)
target_include_directories(TestVelodyneHDLSource PRIVATE ${plugin_include_dirs})
target_link_libraries(TestVelodyneHDLSource LINK_PUBLIC VelodyneHDLPlugin)

custom_add_executable(TestVelodyneHDLReader TestVelodyneHDLReader.cxx TestHelpers.cxx)
target_include_directories(TestVelodyneHDLReader PRIVATE ${plugin_include_dirs})
target_link_libraries(TestVelodyneHDLReader LINK_PUBLIC VelodyneHDLPlugin)

custom_add_executable(TestVelodyneHDLPositionReader TestVelodyneHDLPositionReader.cxx)
target_link_libraries(TestVelodyneHDLPositionReader VelodyneHDLPlugin)

custom_add_executable(TestNMEAParser TestNMEAParser.cxx TestHelpers.cxx)
target_link_libraries(TestNMEAParser VelodyneHDLPlugin)

custom_add_executable(TestTrailingFrame TestTrailingFrame.cxx)
target_link_libraries(TestTrailingFrame VelodyneHDLPlugin)

custom_add_executable(TestRansacPlaneModel TestRansacPlaneModel.cxx)
target_link_libraries(TestRansacPlaneModel VelodyneHDLPlugin)

custom_add_executable(TestVelodynePPSIdentification TestVelodynePPSIdentification.cxx)
target_link_libraries(TestVelodynePPSIdentification VelodyneHDLPlugin)

custom_add_executable(TestVtkEigenTools TestVtkEigenTools.cxx TestHelpers.cxx)
target_link_libraries(TestVtkEigenTools VelodyneHDLPlugin)

if (ENABLE_PCL AND ENABLE_Ceres)
  add_executable(TestGeometricCalibration-MM TestGeometricCalibration-MM.cxx)
  target_link_libraries(TestGeometricCalibration-MM VelodyneHDLPlugin)

  add_executable(TestGeometricCalibration-LaDoua TestGeometricCalibration-LaDoua.cxx)
  target_link_libraries(TestGeometricCalibration-LaDoua VelodyneHDLPlugin)
endif(ENABLE_PCL AND ENABLE_Ceres)

custom_add_executable(TestTemporalTransformsReaderWriter TestTemporalTransformsReaderWriter.cxx TestHelpers.cxx)
target_link_libraries(TestTemporalTransformsReaderWriter VelodyneHDLPlugin)

set(sensors "HDL-64"
            "VLP-16"
            "VLP-32c")


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

add_test(TestTrailingFrame
  ${INSTALL_LOCAL_DIR}/TestTrailingFrame
)

add_test(TestRansacPlaneModel
  ${INSTALL_LOCAL_DIR}/TestRansacPlaneModel
)

if (ENABLE_PCL AND ENABLE_Ceres)
  add_test(TestGeometricCalibration-MM
    ${INSTALL_LOCAL_DIR}/TestGeometricCalibration-MM
    ${CMAKE_SOURCE_DIR}/TestData/trajectories
  )

  add_test(TestGeometricCalibration-LaDoua
    ${INSTALL_LOCAL_DIR}/TestGeometricCalibration-LaDoua
    ${CMAKE_SOURCE_DIR}/TestData/trajectories/la_doua_dataset
  )
endif(ENABLE_PCL AND ENABLE_Ceres)

add_test(TestVelodynePPSIdentification
  ${INSTALL_LOCAL_DIR}/TestVelodynePPSIdentification
  "${CMAKE_SOURCE_DIR}/TestData/HDL32-V2_R_into_Butterfield_into_Digital_Drive.pcap"
  "${CMAKE_SOURCE_DIR}/TestData/VLP-16_Single.pcap"
  "${CMAKE_SOURCE_DIR}/TestData/VLP-16-with-GPRMC-with-PPS-timesync.pcap"
)

custom_add_executable(TestScaleCalibration-MM TestScaleCalibration-MM.cxx)
target_link_libraries(TestScaleCalibration-MM VelodyneHDLPlugin)
add_test(TestScaleCalibration-MM
  ${INSTALL_LOCAL_DIR}/TestScaleCalibration-MM
  ${CMAKE_SOURCE_DIR}/TestData/trajectories
)

custom_add_executable(TestTimeCalibration-LaDoua TestTimeCalibration-LaDoua.cxx)
target_link_libraries(TestTimeCalibration-LaDoua VelodyneHDLPlugin)
add_test(TestTimeCalibration-LaDoua
  ${INSTALL_LOCAL_DIR}/TestTimeCalibration-LaDoua
  ${CMAKE_SOURCE_DIR}/TestData/trajectories/la_doua_dataset
)

custom_add_executable(TestTimeCalibration-MM TestTimeCalibration-MM.cxx)
target_link_libraries(TestTimeCalibration-MM VelodyneHDLPlugin)

add_test(TestTimeCalibration-MM
  ${INSTALL_LOCAL_DIR}/TestTimeCalibration-MM
  ${CMAKE_SOURCE_DIR}/TestData/trajectories
)

custom_add_executable(TestCarGeometricCalibration-LaDoua TestCarGeometricCalibration-LaDoua.cxx)
target_link_libraries(TestCarGeometricCalibration-LaDoua VelodyneHDLPlugin)
add_test(TestCarGeometricCalibration-LaDoua
  ${INSTALL_LOCAL_DIR}/TestCarGeometricCalibration-LaDoua
  ${CMAKE_SOURCE_DIR}/TestData/trajectories/la_doua_dataset
)

custom_add_executable(TestCarGeometricCalibration-MM TestCarGeometricCalibration-MM.cxx)
target_link_libraries(TestCarGeometricCalibration-MM VelodyneHDLPlugin)
add_test(TestCarGeometricCalibration-MM
  ${INSTALL_LOCAL_DIR}/TestCarGeometricCalibration-MM
  ${CMAKE_SOURCE_DIR}/TestData/trajectories
)

add_test(TestTemporalTransformsReaderWriter
  ${INSTALL_LOCAL_DIR}/TestTemporalTransformsReaderWriter
  ${CMAKE_SOURCE_DIR}/TestData/trajectories/mm04/orbslam2-no-loop-closure.csv
  ${CMAKE_SOURCE_DIR}/TestData/trajectories/mm04/orbslam2-no-loop-closure.csv.temporary
)

add_test(TestVtkEigenTools
  ${INSTALL_LOCAL_DIR}/TestVtkEigenTools
)
