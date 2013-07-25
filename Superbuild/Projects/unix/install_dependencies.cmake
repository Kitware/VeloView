# this file determines binary dependencies for ParaView and install thems.

# dependencies_root == directory where dependecies are installed.
# target_root == root directory where files are to be installed.

include(GetPrerequisites)

get_filename_component(exepath "${executable}" PATH)
get_filename_component(exename "${executable}" NAME)

message("Determining dependencies for '${exename}'")
get_prerequisites(
  ${executable}
  prerequisites
  1
  1
  ${exepath}
  ${dependencies_root}/lib
  )

message("Installing dependencies for '${exename}'")

# resolve symlinks.
set (resolved_prerequisites)
foreach(link ${prerequisites})
  if (NOT link MATCHES ".*fontconfig.*")
    if (IS_SYMLINK ${link})
      get_filename_component(resolved_link "${link}" REALPATH)
      # now link may not directly point to resolved_link.
      # so we install the resolved link as the link.
      get_filename_component(resolved_name "${link}" NAME)
      file(INSTALL
        DESTINATION "${target_root}"
        TYPE PROGRAM
        RENAME "${resolved_name}"
        FILES "${resolved_link}")
    else ()
      list(APPEND resolved_prerequisites ${link})
    endif()
  endif()
endforeach()

file(INSTALL ${resolved_prerequisites}
     DESTINATION ${target_root}
     USE_SOURCE_PERMISSIONS)
#file(INSTALL
#     DESTINATION ${target_root}/../../doc)
#file(DOWNLOAD "http://www.paraview.org/files/v${pv_version}/ParaViewUsersGuide.v${pv_version}.pdf"
#     ${target_root}/../../doc/ParaViewUsersGuide.v3.14.pdf
#     SHOW_PROGRESS)
