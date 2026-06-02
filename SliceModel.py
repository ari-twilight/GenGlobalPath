import SliceAlgo
from VtkAdaptor import *
from StlModel import *
class SliceModel:
    def __init__(self,stlModel,layerThk,sliceAlgo="brutal"):
        self.stlModel = stlModel
        self.layerThk = layerThk
        if sliceAlgo == 'brutal': self.slice_brutal()
        elif sliceAlgo == 'optimal': self.slice_optimal()
    def slice_brutal(self):
        self.layers = SliceAlgo.intersectStl_brutal(self.stlModel,self.layerThk)
        for layer in self.layers:
            layer.contours = SliceAlgo.linkSegs_brutal(layer.segments)
            SliceAlgo.adjustPolygonDirs(layer.contours)
    def slice_optimal(self): pass
    def writeSlcFile(self,path):
        SliceAlgo.writeSlcFile(self.layers,path)
    def readSlcFile(self,path):
        self.layers = SliceAlgo.readSlcFile(path)
    def drawLayerContours(self,va,start=0,stop=0xFFFF,step=1,clr=(0,0,0),lineWidth=1):
        for i in range(max(0,start),min(stop,len(self.layers)),step):
            layer = self.layers[i]
            for contour in layer.contours:
                contourActor = va.drawPolyline(contour)
                contourActor.GetProperty().SetColor(clr)
                contourActor.GetProperty().SetLineWidth(lineWidth)
if __name__=='__main__':
        vtkAdaptor = VtkAdaptor()
        stlModel = StlModel()
        stlModel.readStlFile("D:\\5760高温机匣模型第三件.STL")
        sliceModel = SliceModel(stlModel,5)
        sliceModel.drawLayerContours(vtkAdaptor)
        vtkAdaptor.display()