# We hardcode the version numbers since we cannot determine versions during
# configure stage.
set (vv_version_major 0)
set (vv_version_minor 0)
set (vv_version_patch 1)
set (vv_version_suffix)
set (vv_version "${vv_version_major}.${vv_version_minor}")

# Enable CPack packaging.
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
  "Velodyne Viewer is a scientific visualization tool.")
set(CPACK_PACKAGE_NAME "VelodyneViewer")
set(CPACK_PACKAGE_VENDOR "Velodyne.")
set(CPACK_PACKAGE_VERSION_MAJOR ${vv_version_major})
set(CPACK_PACKAGE_VERSION_MINOR ${vv_version_minor})
if (vv_version_suffix)
  set(CPACK_PACKAGE_VERSION_PATCH ${vv_version_patch}-${vv_version_suffix})
else()
  set(CPACK_PACKAGE_VERSION_PATCH ${vv_version_patch})
endif()

set(CPACK_PACKAGE_FILE_NAME
    "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}-${package_suffix}")
