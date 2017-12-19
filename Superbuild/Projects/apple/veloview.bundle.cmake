include(veloview.bundle.common)

set (CPACK_GENERATOR DragNDrop)
include(CPack)

install(CODE "
     file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}\" USE_SOURCE_PERMISSIONS TYPE DIRECTORY FILES
          \"${superbuild_install_location}/bin/${SOFTWARE_NAME}.app\")

     file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}\" USE_SOURCE_PERMISSIONS TYPE FILE FILES
          \"${VeloViewSuperBuild_SOURCE_DIR}/../Documentation/VeloView_Developer_Guide.pdf\")

     file(WRITE \"\${CMAKE_INSTALL_PREFIX}/${SOFTWARE_NAME}.app/Contents/Resources/qt.conf\"
                \"\")
     execute_process(
       COMMAND ${CMAKE_CURRENT_LIST_DIR}/fixup_bundle.py
               \"\${CMAKE_INSTALL_PREFIX}/${SOFTWARE_NAME}.app\"
               \"${superbuild_install_location}/lib\"
               \"${superbuild_install_location}/Applications/paraview.app/Contents/Libraries\"
               \"${superbuild_install_location}/Applications/paraview.app/Contents/Plugins\")
   "
   COMPONENT superbuild)
