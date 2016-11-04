# Include CMake scripts for geting the version from Git
include(Git)
include(ParaViewDetermineVersion)
# Sets VV_VERSION_{MAJOR,MINOR,PATCH} for git
file(STRINGS ${VeloViewSuperBuild_SOURCE_DIR}/../version.txt version_txt)
extract_version_components("${version_txt}" "VV")
determine_version(${VeloViewSuperBuild_SOURCE_DIR} ${GIT_EXECUTABLE} "VV")

# Sets GD_YEAR, GD_MONTH, GD_DAY
include(${VeloViewSuperBuild_SOURCE_DIR}/Projects/getdate.cmake)
GET_DATE()
set(PACKAGE_TIMESTAMP "${GD_YEAR}${GD_MONTH}${GD_DAY}")

# Enable CPack packaging.
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "VeloView")
set(CPACK_PACKAGE_NAME "VeloView")
set(CPACK_PACKAGE_VENDOR "Velodyne Lidar")
set(CPACK_PACKAGE_VERSION_MAJOR ${VV_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VV_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VV_VERSION_PATCH})
if (VV_VERSION_SUFFIX)
  set(CPACK_PACKAGE_VERSION_PATCH ${VV_VERSION_PATCH}-${VV_VERSION_PATCH_EXTRA})
else()
endif()

set(CPACK_RESOURCE_FILE_LICENSE "${VeloViewSuperBuild_SOURCE_DIR}/LICENSE")

set(CPACK_PACKAGE_FILE_NAME
    "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_FULL}-${PACKAGE_TIMESTAMP}-${package_suffix}")
