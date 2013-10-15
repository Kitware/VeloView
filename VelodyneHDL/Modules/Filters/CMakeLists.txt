set(KIT_SRCS
  vtkPlaneFitter.cxx
  )

if(VTK_WRAP_PYTHON)
  set(${vtk-module}_WRAP_PYTHON ON)
endif()

vtk_module_library(${vtk-module} ${KIT_SRCS})

vtk_module_load(${vtk-module})
vtk_module_headers_load(${vtk-module})
vtk_add_python_wrapping(${vtk-module})

install(TARGETS ${vtk-module}Python
  RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries COMPONENT RuntimeLibraries
  LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries COMPONENT RuntimeLibraries
  )
