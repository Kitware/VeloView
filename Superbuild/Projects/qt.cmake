set (qt_options)
set (patch_command)
if (NOT APPLE AND UNIX)
  list (APPEND qt_depends freetype fontconfig png)
  list (APPEND qt_options
               -qt-libpng
               -I <INSTALL_DIR>/include/freetype2
               -I <INSTALL_DIR>/include/fontconfig)
  # Fix Qt build failure with GCC 4.1.
 set (patch_command PATCH_COMMAND
                    ${CMAKE_COMMAND} -E copy_if_different
                    ${SuperBuild_PROJECTS_DIR}/patches/qt.src.3rdparty.webkit.Source.WebKit.pri
                    <SOURCE_DIR>/src/3rdparty/webkit/Source/WebKit.pri)
elseif (APPLE)
  if ((NOT DEFINED CMAKE_OSX_SYSROOT OR CMAKE_OSX_SYSROOT STREQUAL "")
      OR (NOT DEFINED CMAKE_OSX_ARCHITECTURES OR CMAKE_OSX_ARCHITECTURES STREQUAL ""))
    message(FATAL_ERROR "CMAKE_OSX_SYSROOT or CMAKE_OSX_ARCHITECTURES are not configured")
  endif()
  list (APPEND qt_options
              #-sdk ${CMAKE_OSX_SYSROOT} #corewlan is failing on 10.9 with 10.11 sdk
              -arch ${CMAKE_OSX_ARCHITECTURES}
              -platform unsupported/macx-clang-libc++
              -qt-libpng
              -system-zlib
              )
  # Need to patch Qt code to build with Xcode 4.3 or newer (where SDK
  # location chnages using the following command:
  #find . -name "*.pro" -exec sed -i -e "s:/Developer/SDKs/:.*:g" {} \;
  set (patch_command
       PATCH_COMMAND /usr/bin/find . -name "*.pro" -exec sed -i -e "s:/Developer/SDKs/:.*:g" {} +)
  add_external_project_step(qt-patch-osx
    COMMAND git apply --whitespace=fix ${SuperBuild_PROJECTS_DIR}/patches/qt.elcapitan-macossdk.patch
    WORKING_DIRECTORY <SOURCE_DIR>
    DEPENDEES patch
    DEPENDERS configure)
endif()
set(qt_EXTRA_CONFIGURATION_OPTIONS ""
    CACHE STRING "Extra arguments to be passed to Qt when configuring.")

add_external_project_or_use_system(
    qt
    CONFIGURE_COMMAND <SOURCE_DIR>/configure
                      -prefix <INSTALL_DIR>
                      -confirm-license
                      -release
                      -no-audio-backend
                      -no-dbus
                      -nomake demos
                      -nomake examples
                      -nomake tests
                      #-nomake tools
                      -nomake docs
                      -no-multimedia
                      -no-openssl
                      -no-phonon
                      -no-qt3support
                      -no-xinerama
                      -no-script
                      -no-scripttools
                      -no-svg
                      -no-declarative-debug
                      -no-xvideo
                      -opensource
                      -qt-libjpeg
                      -qt-libtiff
                      -qt-zlib
                      -no-webkit
                      #-xmlpatterns
                      -I <INSTALL_DIR>/include
                      -L <INSTALL_DIR>/lib
                      ${qt_options}
                      ${qt_EXTRA_CONFIGURATION_OPTIONS}
    ${patch_command}
)

if ((VV_BUILD_ARCHITECTURE EQUAL 32) AND UNIX AND (NOT APPLE))
  # on 32-bit builds, we are incorrectly ending with QT_POINTER_SIZE chosen as
  # 8 (instead of 4) with GCC4.1 toolchain on old debians. This patch overcomes
  # that.
  add_external_project_step(qt-patch-configure
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                              ${SuperBuild_PROJECTS_DIR}/patches/qt.configure
			      <SOURCE_DIR>/configure
    DEPENDEES patch
    DEPENDERS configure)
endif()
