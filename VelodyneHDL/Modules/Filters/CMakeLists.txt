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
