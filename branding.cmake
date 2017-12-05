set(SOFTWARE_NAME "VeloView")
set(VENDOR "Velodyne Lidar")
set(PROJECT_NAME "${SOFTWARE_NAME}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${SOFTWARE_NAME}")
set(CPACK_PACKAGE_NAME "${SOFTWARE_NAME}")
set(CPACK_PACKAGE_VENDOR "${VENDOR}")
# Please make sure to adapt the AboutDialog text in the followin file
#  VelodyneHDL\python\veloview\aboutDialog.py
# Images to change:
#   - logo.ico (app icon)
#   - logo.icsn (bunbled app icon)
#   - Splash.jpg -> Image display when the software is loading
#   - logo.png
#   - VelodyneHDL/images/bottom_logo.png (bottom logo)

add_definitions( -DPROJECT_NAME="${PROJECT_NAME}" )
add_definitions( -DSOFTWARE_NAME="${SOFTWARE_NAME}" )
#add_definitions( -DVENDOR="${VENDOR}" ) # spaces here confuses the vtkWrap