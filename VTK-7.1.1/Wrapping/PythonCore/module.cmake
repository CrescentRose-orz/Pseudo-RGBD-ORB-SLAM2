vtk_module(vtkWrappingPythonCore
  COMPILE_DEPENDS
    vtkWrappingTools
  EXCLUDE_FROM_ALL
  EXCLUDE_FROM_WRAPPING
  DEPENDS
    vtkCommonCore
    vtkPython
    vtksys
  )