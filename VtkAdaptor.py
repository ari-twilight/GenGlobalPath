import vtk
from GeomBase import *
from Line import *
from Ray import *
from Segment import *
from Plane import *
from Polyline import *

class VtkAdaptor:
    def __init__(self,bgClr=(0.95,0.95,0.95)):
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(bgClr)
        self.window = vtk.vtkRenderWindow()
        self.window.AddRenderer(self.renderer)
        self.window.SetSize(1000,1000)
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.window)
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.interactor.Initialize()
    def display(self):
        self.interactor.Start()
    def setBackgroundColor(self,r,g,b):
        return self.renderer.SetBackground(r,g,b)
    def drawAxes(self,length=100.0,shaftType=0,cylinderRadius=0.01,coneRadius=0.2):
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(length,length,length)
        axes.SetShaftType(shaftType)
        axes.SetCylinderRadius(cylinderRadius)
        axes.SetConeRadius(coneRadius)
        axes.SetAxisLabels(0)
        self.renderer.AddActor(axes)
        return axes
    def drawActor(self,actor):
        self.renderer.AddActor(actor)
        return actor
    def drawPdSrc(self,pdSrc):
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(pdSrc.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        return self.drawActor(actor)
    def drawSTLModel(self,stlFilePath):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(stlFilePath)
        return self.drawPdSrc(reader)
    def removeActor(self,actor):
        self.renderer.RemoveActor(actor)
    def drawPoint(self,point,radius=2.0):
        src = vtk.vtkSphereSource()
        src.SetCenter(point.x,point.y,point.z)
        src.SetRadius(radius)
        return self.drawPdSrc(src)
    def drawSegment(self,seg):
        src = vtk.vtkLineSource()
        src.SetPoint1(seg.A.x,seg.A.y,seg.A.z)
        src.SetPoint2(seg.B.x,seg.B.y,seg.B.z)
        return self.drawPdSrc(src)
    def drawPolyline(self,polyline):
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        for i in range(polyline.count()):
            pt = polyline.point(i)
            points.InsertNextPoint((pt.x,pt.y,pt.z))
        for i in range(polyline.count() - 1):
            p1 = polyline.point(i)
            p2 = polyline.point(i + 1)
            if getattr(p1, "w", 0) == 2 or getattr(p2, "w", 0) == 2:
                continue
            if p1.isCoincide(p2):
                continue
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, i)
            line.GetPointIds().SetId(1, i + 1)
            lines.InsertNextCell(line)
        poly_data = vtk.vtkPolyData()
        poly_data.SetPoints(points)
        poly_data.SetLines(lines)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly_data)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        return self.drawActor(actor)
    pass
