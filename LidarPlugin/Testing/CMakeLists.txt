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

custom_add_executable(TestLidarStream TestHelpers.cxx TestLidarStream.cxx)
target_include_directories(TestLidarStream PRIVATE ${plugin_include_dirs})
target_link_libraries(TestLidarStream LINK_PUBLIC LidarPlugin)

custom_add_executable(TestLidarReader TestLidarReader.cxx TestHelpers.cxx)
target_include_directories(TestLidarReader PRIVATE ${plugin_include_dirs})
target_link_libraries(TestLidarReader LINK_PUBLIC LidarPlugin)

custom_add_executable(TestVelodyneHDLPositionReader TestVelodyneHDLPositionReader.cxx)
target_link_libraries(TestVelodyneHDLPositionReader LidarPlugin)

custom_add_executable(TestNMEAParser TestNMEAParser.cxx TestHelpers.cxx)
target_link_libraries(TestNMEAParser LidarPlugin)

custom_add_executable(TestTrailingFrame TestTrailingFrame.cxx)
target_link_libraries(TestTrailingFrame LidarPlugin)

custom_add_executable(TestRansacPlaneModel TestRansacPlaneModel.cxx)
target_link_libraries(TestRansacPlaneModel LidarPlugin)

custom_add_executable(TestVelodynePPSIdentification TestVelodynePPSIdentification.cxx)
target_link_libraries(TestVelodynePPSIdentification LidarPlugin)

custom_add_executable(TestVtkEigenTools TestVtkEigenTools.cxx TestHelpers.cxx)
target_link_libraries(TestVtkEigenTools LidarPlugin)

custom_add_executable(TestBoundingBox TestBoundingBox.cxx)
target_link_libraries(TestBoundingBox LidarPlugin)

if (ENABLE_ceres)
  add_executable(TestCameraCalibration TestCameraCalibration.cxx)
  target_link_libraries(TestCameraCalibration LidarPlugin)
endif (ENABLE_ceres)

if (ENABLE_pcl AND ENABLE_ceres)
  add_executable(TestGeometricCalibration-MM TestGeometricCalibration-MM.cxx)
  target_link_libraries(TestGeometricCalibration-MM LidarPlugin)

  add_executable(TestGeometricCalibration-LaDoua TestGeometricCalibration-LaDoua.cxx)
  target_link_libraries(TestGeometricCalibration-LaDoua LidarPlugin)

  add_executable(TestTrajectoryReoptimization TestTrajectoryReoptimization.cxx)
  target_link_libraries(TestTrajectoryReoptimization LidarPlugin)
endif(ENABLE_PCL AND ENABLE_Ceres)

if (ENABLE_pcl AND ENABLE_ceres AND ENABLE_nanoflann)
  custom_add_executable(TestSlam TestSlam.cxx TestHelpers.cxx)
  target_link_libraries(TestSlam LINK_PUBLIC LidarPlugin)
endif(ENABLE_pcl AND ENABLE_ceres AND ENABLE_nanoflann)

if (ENABLE_pcl AND ENABLE_Ceres AND ENABLE_OpenCV)
  add_executable(TestMIDHOG TestMIDHOG.cxx)
  target_link_libraries(TestMIDHOG LidarPlugin)
endif(ENABLE_pcl AND ENABLE_Ceres AND ENABLE_OpenCV)

custom_add_executable(TestTemporalTransformsReaderWriter TestTemporalTransformsReaderWriter.cxx TestHelpers.cxx)
target_link_libraries(TestTemporalTransformsReaderWriter LidarPlugin)

set(sensors "HDL-64"
            "VLP-16"
            "VLP-32c")


# generate script to generate new test baseline
configure_file(generateTestData.py.in ${INSTALL_LOCAL_DIR}/generateTestData.py)

# add test
foreach(sensor ${sensors})

    # LidarStream test
    add_test(TestLidarStream_${sensor}_Single
      ${INSTALL_LOCAL_DIR}/TestLidarStream
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single.pcap
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single/files.txt
      ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
    )
    add_test(TestLidarStream_${sensor}_Dual
      ${CMAKE_BINARY_DIR}/bin/TestLidarStream
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual.pcap
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual/files.txt
      ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
    )

    # LidarReader test
    add_test(TestLidarReader_${sensor}_Single
      ${INSTALL_LOCAL_DIR}/TestLidarReader
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single.pcap
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single/files.txt
      ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single-reference-data.xml
    )
    add_test(TestLidarReader_${sensor}_Dual
      ${INSTALL_LOCAL_DIR}/TestLidarReader
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual.pcap
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual/files.txt
      ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
      ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual-reference-data.xml
    )

endforeach(sensor)

# add special test for HDL-64 in autocalibration mode
# LidarStream test
add_test(TestLidarStream_HDL-64_Single-autocalib
  ${INSTALL_LOCAL_DIR}/TestLidarStream
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib.pcap
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib/files.txt
  ""
)
add_test(TestLidarStream_HDL-64_Dual-autocalib
  ${INSTALL_LOCAL_DIR}/TestLidarStream
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib.pcap
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib/files.txt
  ""
)

# LidarReader test
add_test(TestLidarReader_HDL-64_Single-autocalib
  ${INSTALL_LOCAL_DIR}/TestLidarReader
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib.pcap
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib/files.txt
  ""
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib-reference-data.xml
)
add_test(TestLidarReader_HDL-64_Dual-autocalib
  ${INSTALL_LOCAL_DIR}/TestLidarReader
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib.pcap
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib/files.txt
  ""
  ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib-reference-data.xml
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

if (ENABLE_ceres)
  add_test(TestCameraCalibration
    ${INSTALL_LOCAL_DIR}/TestCameraCalibration
    ${CMAKE_SOURCE_DIR}/TestData/Camera/MatchedPoints_3D_2D
  )
endif (ENABLE_ceres)

if (ENABLE_pcl AND ENABLE_ceres)
  add_test(TestGeometricCalibration-MM
    ${INSTALL_LOCAL_DIR}/TestGeometricCalibration-MM
    ${CMAKE_SOURCE_DIR}/TestData/trajectories
  )

  add_test(TestGeometricCalibration-LaDoua
    ${INSTALL_LOCAL_DIR}/TestGeometricCalibration-LaDoua
    ${CMAKE_SOURCE_DIR}/TestData/trajectories/la_doua_dataset
  )

  add_test(TestTrajectoryReoptimization
    ${INSTALL_LOCAL_DIR}/TestTrajectoryReoptimization
    ${CMAKE_SOURCE_DIR}/TestData/trajectories/mm05_reoptimization
  )
endif(ENABLE_PCL AND ENABLE_Ceres)

if (ENABLE_pcl AND ENABLE_ceres AND ENABLE_nanoflann)
  add_test(TestSlam
    ${INSTALL_LOCAL_DIR}/TestSlam
    ${CMAKE_SOURCE_DIR}/TestData/Slam/VLP-16_slam_test_data.pcap
    ${CMAKE_SOURCE_DIR}/TestData/Slam/RefSlam.vtp
    ${CMAKE_SOURCE_DIR}/share/VLP-16.xml
  )
endif(ENABLE_pcl AND ENABLE_ceres AND ENABLE_nanoflann)

if (ENABLE_pcl AND ENABLE_Ceres AND ENABLE_OpenCV)
  add_test(TestMIDHOG
  ${INSTALL_LOCAL_DIR}/TestMIDHOG
  )
endif(ENABLE_pcl AND ENABLE_Ceres AND ENABLE_OpenCV)

add_test(TestVelodynePPSIdentification
  ${INSTALL_LOCAL_DIR}/TestVelodynePPSIdentification
  "${CMAKE_SOURCE_DIR}/TestData/HDL32-V2_R_into_Butterfield_into_Digital_Drive.pcap"
  "${CMAKE_SOURCE_DIR}/TestData/VLP-16_Single.pcap"
  "${CMAKE_SOURCE_DIR}/TestData/VLP-16-with-GPRMC-with-PPS-timesync.pcap"
)

custom_add_executable(TestScaleCalibration-MM TestScaleCalibration-MM.cxx)
target_link_libraries(TestScaleCalibration-MM LidarPlugin)
add_test(TestScaleCalibration-MM
  ${INSTALL_LOCAL_DIR}/TestScaleCalibration-MM
  ${CMAKE_SOURCE_DIR}/TestData/trajectories
)

custom_add_executable(TestTimeCalibration-LaDoua TestTimeCalibration-LaDoua.cxx)
target_link_libraries(TestTimeCalibration-LaDoua LidarPlugin)
add_test(TestTimeCalibration-LaDoua
  ${INSTALL_LOCAL_DIR}/TestTimeCalibration-LaDoua
  ${CMAKE_SOURCE_DIR}/TestData/trajectories/la_doua_dataset
)

custom_add_executable(TestTimeCalibration-MM TestTimeCalibration-MM.cxx)
target_link_libraries(TestTimeCalibration-MM LidarPlugin)

add_test(TestTimeCalibration-MM
  ${INSTALL_LOCAL_DIR}/TestTimeCalibration-MM
  ${CMAKE_SOURCE_DIR}/TestData/trajectories
)

custom_add_executable(TestCarGeometricCalibration-LaDoua TestCarGeometricCalibration-LaDoua.cxx)
target_link_libraries(TestCarGeometricCalibration-LaDoua LidarPlugin)
add_test(TestCarGeometricCalibration-LaDoua
  ${INSTALL_LOCAL_DIR}/TestCarGeometricCalibration-LaDoua
  ${CMAKE_SOURCE_DIR}/TestData/trajectories/la_doua_dataset
)

custom_add_executable(TestCarGeometricCalibration-MM TestCarGeometricCalibration-MM.cxx)
target_link_libraries(TestCarGeometricCalibration-MM LidarPlugin)
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

add_test(TestBoundingBox
  ${INSTALL_LOCAL_DIR}/TestBoundingBox
)
