include(velodyneviewer.bundle.common)

set (CPACK_GENERATOR DragNDrop)
include(CPack)

# we only to paraview explicitly.
install(CODE "
     file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}\" USE_SOURCE_PERMISSIONS TYPE DIRECTORY FILES
          \"${install_location}/bin/VeloView.app\")

     file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}\" USE_SOURCE_PERMISSIONS TYPE FILE FILES
          \"${SuperBuild_BINARY_DIR}/velodyneviewer/src/velodyneviewer/Doc/VeloView_Developer_Guide.pdf\")

     file(WRITE \"\${CMAKE_INSTALL_PREFIX}/VeloView.app/Contents/Resources/qt.conf\"
                \"\")
     execute_process(
       COMMAND ${CMAKE_CURRENT_LIST_DIR}/fixup_bundle.py
               \"\${CMAKE_INSTALL_PREFIX}/VeloView.app\"
               \"${install_location}/lib\"
               \"${SuperBuild_BINARY_DIR}/paraview/src/paraview-build/lib\"
               \"${install_location}/plugins\")
   "
   COMPONENT superbuild)
