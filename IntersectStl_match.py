import SliceAlgo
from VtkAdaptor import *
from StlModel import *
from Layer import *
from GeomAlgo import *

class IntersectStl_match:
    def __init__(self,stlModel,layerThk):
        self.stlModel = stlModel
        self.layerThk = layerThk
        self.layers = []
        self.intersect()
    def intersect(self):
        zs,layerDic = self.genLayerHeights()
        self.matchFacetZs_bisection(zs)
        for triangle in self.stlModel.triangles:
            for z in triangle.zs:
                seg = intersectTriangleZPlane(triangle,z)
                if seg is not None:
                    layerDic[z].segments.append(seg)
        for layer in layerDic.values():
            self.layers.append(layer)
    def genLayerHeights(self):
        xMin, xMax, yMin, yMax, zMin, zMax = self.stlModel.getBounds()
        zs,layerDic = [],{}
        z = zMin + self.layerThk
        while z < zMax:
            zs.append(z)
            layerDic[z] = Layer(z)
            z += self.layerThk
        return zs,layerDic
    def matchFacetZs_brutal(self,zs):
        for tri in self.stlModel.triangles:
            zMin,zMax = tri.zMinPnt().z,tri.zMaxPnt().z
            for z in zs:
                if z > zMax: break
                if z >= zMin and z <= zMax:
                    tri.zs.append(z)
    def matchFacetZs_bisection(self,zs):
        n = len(zs)
        for tri in self.stlModel.triangles:
            zMin,zMax = tri.zMinPnt().z,tri.zMaxPnt().z
            low,up,mid = 0,n-1,0
            while up-low > 1:
                mid = int((low+up)/2)
                if zs[mid] < zMin: low = mid
                else: up = mid
            start = up
            if zs[low] == zMin:
                start = low
            low,up = 0,n-1
            while up - low > 1:
                mid = int((up+low)/2)
                if zs[mid]<zMax: low=mid
                else: up=mid
            stop = low
            if zs[up] == zMax:
                stop = up
            for i in range(start,stop+1,1):
                tri.zs.append(zs[i])