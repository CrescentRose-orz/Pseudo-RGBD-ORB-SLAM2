vtk_module(vtkCommonSystem
  GROUPS
    StandAlone
  TEST_DEPENDS
    vtkTestingCore
  KIT
    vtkCommon
  DEPENDS
    vtkCommonCore
  PRIVATE_DEPENDS
    vtksys
  )