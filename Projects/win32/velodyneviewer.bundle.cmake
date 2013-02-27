include(velodyneviewer.bundle.common)

#------------------------------------------------------------------------------
# set NSIS install specific stuff.

set (CPACK_NSIS_MENU_LINKS
  "bin/VelodyneViewer.exe" "VelodyneViewer")

install(DIRECTORY "${install_location}/bin/"
        DESTINATION "bin"
        COMPONENT "VelodyneViewer")

SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS 
  "MessageBox MB_YESNO \\\"Setup will now download/install WinPCAP. Do you want to install WinPCAP? It is needed for VelodyneViewer to work.\\\" /SD IDYES IDNO endActiveSync
NSISdl::download http://www.winpcap.org/install/bin/WinPcap_4_1_2.exe $INSTDIR\\\\WinPcap_4_1_2.exe
ExecWait \\\"$INSTDIR\\\\WinPcap_4_1_2.exe\\\"
Delete \\\"$INSTDIR\\\\WinPcap_4_1_2.exe\\\"
Goto endActiveSync
endActiveSync:
")

#------------------------------------------------------------------------------
set (CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_LIST_DIR}/InstallerIcon.ico")

# install system runtimes.
set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION "bin")
include(InstallRequiredSystemLibraries)
include(CPack)
