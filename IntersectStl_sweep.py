import SliceAlgo
from VtkAdaptor import *
from StlModel import *
from Layer import *
from GeomAlgo import *

class SweepPlane:
    def __init__(self):
        self.triangles = []
    pass

class IntersectStl_sweep:
    def __init__(self,stlModel,layerThk):
        self.stlModel = stlModel
        self.layerThk = layerThk
        self.layers = []
        self.intersect()
    def intersect(self):
        triangles = self.stlModel.triangles
        triangles.sort(key=lambda t:t.zMinPnt().z)
        zs = self.getLayerHeights()
        k = 0
        sweep = SweepPlane()
        for z in zs:
            for i in range(len(sweep.triangles)-1,-1,-1):
                if z > sweep.triangles[i].zMaxPnt().z:
                    del sweep.triangles[i]
            for i in range(k,len(triangles)):
                tri = triangles[i]
                if z >= tri.zMinPnt().z and z <= tri.zMaxPnt().z:
                    sweep.triangles.append(tri)
                elif z < tri.zMinPnt().z:
                    k = i
                    break
            layer = Layer(z)
            for triangle in sweep.triangles:
                seg = intersectTriangleZPlane(triangle,z)
                if seg is not None:
                    layer.segments.append(seg)
            self.layers.append(layer)
    def getLayerHeights(self):
        xMin, xMax, yMin, yMax, zMin, zMax = self.stlModel.getBounds()
        zs,z = [],zMin+self.layerThk
        while z < zMax:
            zs.append(z)
            z += self.layerThk
        return zs