import vtk

renderer = vtk.vtkRenderer()
renderer.SetBackground(0.95,0.95,0.95)
window = vtk.vtkRenderWindow()
window.AddRenderer(renderer)
window.SetSize(900,600)
interactor = vtk.vtkRenderWindowInteractor()
interactor.SetRenderWindow(window)
interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
interactor.Initialize()

stlReader = vtk.vtkSTLReader()
stlReader.SetFileName("D:\\5760高温机匣模型第三件.STL")

filter = vtk.vtkOutlineFilter()
filter.SetInputConnection(stlReader.GetOutputPort())
outLineMapper = vtk.vtkPolyDataMapper()
outLineMapper.SetInputConnection(filter.GetOutputPort())
outlineActor = vtk.vtkActor()
outlineActor.SetMapper(outLineMapper)
outlineActor.GetProperty().SetColor(0.1,0.1,0.1)
renderer.AddActor(outlineActor)

stlMapper = vtk.vtkPolyDataMapper()
stlMapper.SetInputConnection(stlReader.GetOutputPort())
stlActor = vtk.vtkActor()
stlActor.SetMapper(stlMapper)
renderer.AddActor(stlActor)

interactor.Start()