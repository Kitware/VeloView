add_executable(TestVelodyneHDLSource TestHelpers.cxx TestVelodyneHDLSource.cxx)
target_link_libraries(TestVelodyneHDLSource VelodyneHDLPlugin)

add_executable(TestVelodyneHDLReader TestVelodyneHDLReader.cxx TestHelpers.cxx)
target_link_libraries(TestVelodyneHDLReader VelodyneHDLPlugin)

# TODO add test to OSX and Windows plateform.
# please do not duplicate code and use variable when there are difference between linux, windows, OSX!!!!

set(sensors "HDL-64"
            "VLP-16"
            "VLP-32c")

# On windows, install the tests in the install/bin
# folder. Otherwise, the .exe tests can't find the required
# dll (Velodyne plugins and vtk)
if (WIN32)
#    set(INSTALL_LOCAL_DIR "${CMAKE_BINARY_DIR}/../../../install/bin")

#    install(TARGETS TestVelodyneHDLReader
#          RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
#          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
#          COMPONENT Runtime)

#    install(TARGETS TestReaderLiveStream
#          RUNTIME DESTINATION ${INSTALL_LOCAL_DIR}
#          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
#          COMPONENT Runtime)

#    add_test(TestVelodyneHDLReader
#      ${INSTALL_LOCAL_DIR}/TestReaderVTP
#      ${CMAKE_SOURCE_DIR}/TestData/
#      ${CMAKE_SOURCE_DIR}/share/
#      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
#    )

#    add_test(TestReaderLiveStream
#      ${INSTALL_LOCAL_DIR}/TestReaderLiveStream
#      ${CMAKE_SOURCE_DIR}/TestData/
#      ${CMAKE_SOURCE_DIR}/share/
#      WORKING_DIRECTORY ${INSTALL_LOCAL_DIR}
#    )

#configure_file(../python/generateTestData.py.in
#  ${INSTALL_LOCAL_DIR}/generateTestData.py)
else()
    # generate script to generate new test baseline
    configure_file(generateTestData.py.in
      ${CMAKE_BINARY_DIR}/bin/generateTestData.py)

    # add test
    foreach(sensor ${sensors})

        # VelodyneHDLSource test
        add_test(TestVelodyneHDLSource_${sensor}_Single
          ${CMAKE_BINARY_DIR}/bin/TestVelodyneHDLSource
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

        # VelodyneHDLSource test
        add_test(TestVelodyneHDLReader_${sensor}_Single
          ${CMAKE_BINARY_DIR}/bin/TestVelodyneHDLReader
          ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single.pcap
          ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Single/files.txt
          ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
        )
        add_test(TestVelodyneHDLReader_${sensor}_Dual
          ${CMAKE_BINARY_DIR}/bin/TestVelodyneHDLReader
          ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual.pcap
          ${CMAKE_SOURCE_DIR}/TestData/${sensor}_Dual/files.txt
          ${CMAKE_SOURCE_DIR}/share/${sensor}.xml
        )

        # Set the DYLD_LIBRARY_PATH used to find libraries for the tests on MacOS
        if(APPLE)
         set_property(TEST
           TestVelodyneHDLSource_${sensor}_Single
           TestVelodyneHDLSource_${sensor}_Dual
           TestVelodyneHDLReader_${sensor}_Single
           TestVelodyneHDLReader_${sensor}_Dual
           PROPERTY ENVIRONMENT
           "DYLD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/../../../install/lib:$ENV{DYLD_LIBRARY_PATH}")
        endif()
    endforeach(sensor)

    # add special test for HDL-64 in autocalibration mode

    # VelodyneHDLReader test
    add_test(TestVelodyneHDLReader_HDL-64_Single-autocalib
      ${CMAKE_BINARY_DIR}/bin/TestVelodyneHDLReader
      ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib.pcap
      ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib/files.txt
      ""
    )
    add_test(TestVelodyneHDLReader_HDL-64_Dual-autocalib
      ${CMAKE_BINARY_DIR}/bin/TestVelodyneHDLReader
      ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib.pcap
      ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib/files.txt
      ""
    )
    
    # VelodyneHDLSource test
    add_test(TestVelodyneHDLSource_HDL-64_Single-autocalib
      ${CMAKE_BINARY_DIR}/bin/TestVelodyneHDLSource
      ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib.pcap
      ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Single-autocalib/files.txt
      ""
    )
    add_test(TestVelodyneHDLSource_HDL-64_Dual-autocalib
      ${CMAKE_BINARY_DIR}/bin/TestVelodyneHDLSource
      ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib.pcap
      ${CMAKE_SOURCE_DIR}/TestData/HDL-64_Dual-autocalib/files.txt
      ""
    )

    # Set the DYLD_LIBRARY_PATH used to find libraries for the tests on MacOS
    if(APPLE)
     set_property(TEST
       TestVelodyneHDLReader_HDL-64_Single-autocalib
       TestVelodyneHDLReader_HDL-64_Dual-autocalib
       TestVelodyneHDLSource_HDL-64_Single-autocalib
       TestVelodyneHDLSource_HDL-64_Dual-autocalib
       PROPERTY ENVIRONMENT
       "DYLD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/../../../install/lib:$ENV{DYLD_LIBRARY_PATH}")
    endif()
endif()
