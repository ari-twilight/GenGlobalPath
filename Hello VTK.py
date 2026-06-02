import vtk

source = vtk.vtkCubeSource()

mapper = vtk.vtkPolyDataMapper()
mapper.SetInputConnection(source.GetOutputPort())

actor = vtk.vtkActor()
actor.SetMapper(mapper)
actor.GetProperty().SetColor(0.7,0.7,0.7)

renderer = vtk.vtkRenderer()
renderer.AddActor(actor)
renderer.SetBackground(0.9,0.9,0.9)

window = vtk.vtkRenderWindow()
window.AddRenderer(renderer)
window.SetSize(900,600)

interactor = vtk.vtkRenderWindowInteractor()
interactor.SetRenderWindow(window)
interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
interactor.Initialize()
interactor.Start()