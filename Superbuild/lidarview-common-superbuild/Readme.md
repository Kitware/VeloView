# LidarView-Superbuild

LidarView-Superbuild is a project to build LidarView or any other LidarView-based application (VeloView, OusterView, ...) and its dependencies.
The project is a small wrapper above Paraview-Superbuild, and provide the common dependencies needed by all viewer and some extra ones.

## How to use the Superbuild

To use the superbuild, simply include this project into your Viewer as a submodule. We recommand you to use this architecture as it is similar to paraview's one.

    # customviewer repository
    ├── Superbuild
    │   ├── lidarview-common-superbuild # the common-superbuild
    │   ├── Projects                    # how to compile the projects
    │   │   └──customviewer.cmake
    │   ├── CMakeLists.txt
    │   └── versions.cmake              # how to get the project source code
    ├── ...
    ├── Application
    ├── LICENSE
    └── README.md

The CMakeLists.txt must define some function/variable to overight the common build default behaviour. Here is a minimalist example:

```cmake
set (CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/Projects"
  ${CMAKE_MODULE_PATH})

function (add_project_to_superbuild var)
  # list cannot append to parent's scope so we do it in two steps
  list(APPEND "${var}" customviewer)
  set("${var}" "${${var}}" PARENT_SCOPE)
endfunction ()

list(APPEND superbuild_version_files
  "${CMAKE_CURRENT_LIST_DIR}/versions.cmake")

add_subdirectory(lidarview-common-superbuild)
```
