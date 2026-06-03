from LinkPoint import *
import SliceAlgo
from VtkAdaptor import *
from StlModel import *
from Layer import *
from GeomAlgo import *
from IntersectStl_sweep import *

class TVertex:
    def __init__(self,pnt3d,digits=7):
        self.x = round(pnt3d.x,digits)
        self.y = round(pnt3d.y,digits)
        self.z = round(pnt3d.z,digits)
        self.faces = []
    def toPoint3D(self):
        return Point3D(self.x,self.y,self.z)
    def toTuple(self):
        return (self.x,self.y,self.z)
    def isSmaller(self,other):
        if self.x<other.x:
            return True
        elif self.x==other.x and self.y<other.y:
            return True
        elif self.x==other.x and self.y==other.y and self.z<other.z:
            return True
        return False

class TEdge:
    def __init__(self,tA,tB):
        self.A,self.B = tA,tB
        self.F = None
        self.OE = None
    def toTuple(self):
        if self.A.isSmaller(self.B):
            return (self.A.x,self.A.y,self.A.z,self.B.x,self.B.y,self.B.z)
        else:
            return (self.B.x,self.B.y,self.B.z,self.A.x,self.A.y,self.A.z)
    def intersect(self,z):
        if min(self.A.z,self.B.z)>z or max(self.A.z,self.B.z)<z:
            return None
        elif self.A.z==self.B.z==z:
            return None
        else:
            if z==self.A.z:
                return self.A.toPoint3D()
            else:
                ratio = (z-self.A.z)/(self.B.z-self.A.z)
                vec = self.A.toPoint3D().pointTo(self.B.toPoint3D()).amplified(ratio)
                pnt = self.A.toPoint3D() + vec
                return pnt

class Tface:
    def __init__(self,tA,tB,tC,te1,te2,te3):
        self.A,self.B,self.C = tA,tB,tC
        self.E1,self.E2,self.E3 = te1,te2,te3
        self.used = False
    def zMin(self):
        return min(self.A.z,self.B.z,self.C.z)
    def zMax(self):
        return max(self.A.z,self.B.z,self.C.z)
    def intersect(self,z):
        if self.zMin()>z or self.zMax()<z:
            return None,None,None
        elif self.A.z==self.B.z==self.C.z==z:
            return None,None,None
        else:
            c1 = self.E1.intersect(z)
            c2 = self.E2.intersect(z)
            c3 = self.E3.intersect(z)
            if c1 is None:
                if c2 is not None and c3 is not None:
                    if c2.distance(c3) != 0.0:
                        return Segment(c2,c3),[self.E2,self.E3],None
            elif c2 is None:
                if c1 is not None and c3 is not None:
                    if c1.distance(c3) != 0.0:
                        return Segment(c1,c3),[self.E1,self.E3],None
            elif c3 is None:
                if c1 is not None and c2 is not None:
                    if c1.distance(c2) != 0.0:
                        return Segment(c1,c2),[self.E1,self.E2],None
            elif c1 is not None and c2 is not None and c3 is not None:
                if c1.isIdentical(c2):
                    return Segment(c1,c3),[self.E3],self.B
                elif c1.isIdentical(c3):
                    return Segment(c1,c2),[self.E2],self.A
                elif c2.isIdentical(c3):
                    return Segment(c1,c2),[self.E1],self.C
            return None,None,None

class TModel:
    def __init__(self,stlModel):
        self.vxDic,self.egDic = {},{}
        self.faces = []
        self.stlModel = stlModel
        self.createTModel()
    def createTModel(self):
        for t in self.stlModel.triangles:
            A,B,C = TVertex(t.A),TVertex(t.B),TVertex(t.C)
            if A.toTuple() not in self.vxDic.keys():
                self.vxDic[A.toTuple()] = A
            if B.toTuple() not in self.vxDic.keys():
                self.vxDic[B.toTuple()] = B
            if C.toTuple() not in self.vxDic.keys():
                self.vxDic[C.toTuple()] = C
            tA = self.vxDic[A.toTuple()]
            tB = self.vxDic[B.toTuple()]
            tC = self.vxDic[C.toTuple()]
            e1,e2,e3 = TEdge(tA,tB),TEdge(tB,tC),TEdge(tC,tA)
            f = Tface(tA,tB,tC,e1,e2,e3)
            self.faces.append(f)
            tA.faces.append(f)
            tB.faces.append(f)
            tC.faces.append(f)
            e1.F=e2.F=e3.F=f
            e1tp,e2tp,e3tp = e1.toTuple(),e2.toTuple(),e3.toTuple()
            if e1tp not in self.egDic.keys():
                self.egDic[e1tp] = []
            self.egDic[e1tp].append(e1)
            if e2tp not in self.egDic.keys():
                self.egDic[e2tp] = []
            self.egDic[e2tp].append(e2)
            if e3tp not in self.egDic.keys():
                self.egDic[e3tp] = []
            self.egDic[e3tp].append(e3)
        for edges in self.egDic.values():
                if len(edges) == 2:
                    edges[0].OE,edges[1].OE = edges[1],edges[0]
                else:
                    print('Single edge')

class TopoSlicer:
    def __init__(self,stlModel,layerThk):
        self.stlModel = stlModel
        self.layerThk = layerThk
        self.topoModel = TModel(stlModel)
        self.layers = []
        self.slice()
    def findSeedFace(self,faces):
        for face in faces:
            if face.used == False:
                return face
        return None
    def findNextFace(self,edges,node):
        nextFace = None
        if node is None:
            e0,e1 = edges[0],edges[1]
            if e1.OE is not None and e1.OE.F.used == False:
                nextFace = e1.OE.F
            elif e0.OE is not None and e0.OE.F.used == False:
                nextFace = e0.OE.F
        else:
            e = edges[0]
            if e.OE is not None and e.OE.F.used == False:
                nextFace = e.OE.F
            else:
                for f in node.faces:
                    if f is not e.F and f.used == False:
                        seg,egs,n = f.intersect(node.z)
                        if seg is not None:
                            nextFace = f
                            break
        return nextFace
    def createLayerContours(self,z,faces):
        layer = Layer(z)
        for f in faces:
            f.used = False
        while True:
            f = self.findSeedFace(faces)
            if f is None: break
            contour = Polyline()
            while True:
                seg,edges,node = f.intersect(z)
                f.used = True
                if seg is None: break
                contour.appendSegment(seg)
                if contour.isClosed(): break
                f = self.findNextFace(edges,node)
                if f is None: break
            if contour.count()>0:
                layer.contours.append(contour)
        return layer
    def slice(self):
        self.topoModel.faces.sort(key=lambda t:t.zMin())
        zs = self.genLayerHeight()
        k = 0
        sweep = SweepPlane()
        for z in zs:
            for i in range(len(sweep.triangles)-1,-1,-1):
                if z > sweep.triangles[i].zMax():
                    del sweep.triangles[i]
            for i in range(k,len(self.topoModel.faces)):
                face = self.topoModel.faces[i]
                if z>=face.zMin() and z<=face.zMax():
                    sweep.triangles.append(face)
                elif face.zMin()>z:
                    k = i
                    break
            layer = self.createLayerContours(z,sweep.triangles)
            adjustPolygonDirs(layer.contours)
            self.layers.append(layer)
    def genLayerHeight(self):
        xMin,xMax,yMin,yMax,zMin,zMax = self.stlModel.getBounds()
        zs,z = [],zMin+self.layerThk
        while z<zMax:
            zs.append(z)
            z += self.layerThk
        return zs