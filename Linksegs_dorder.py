from LinkPoint import *
import SliceAlgo
from VtkAdaptor import *
from StlModel import *
from Layer import *
from GeomAlgo import *
import functools

def cmp_pntSmaller(lp1,lp2):
    if lp1.x<lp2.x:
        return -1
    elif lp1.x==lp2.x and lp1.y<lp2.y:
        return -1
    elif lp1.x==lp2.x and lp1.y==lp2.y and lp1.z<lp2.z:
        return -1
    elif lp1.x==lp2.x and lp1.y==lp2.y and lp1.z==lp2.z:
        return 0
    else: return 1

class LinkSegs_dorder:
    def __init__(self,segs):
        self.segs = segs
        self.contours = []
        self.polys = []
        self.link()
    def createLpList(self):
        lpnts = []
        for seg in self.segs:
            lp1,lp2 = LinkPoint(seg.A),LinkPoint(seg.B)
            lp1.other,lp2.other = LinkPoint(seg.B),LinkPoint(seg.A)
            lpnts.append(lp1)
            lpnts.append(lp2)
        lpnts.sory(key=functools.cmp_to_key(cmp_pntSmaller))
        for i in range(len(lpnts)):
            lpnts[i].index = i
        return lpnts
    def findUnusedPnt(self,lpnts):
        startIndex = -1
        for lpnt in lpnts:
            if lpnt.used == False:
                startIndex = lpnt.index
                break
        return startIndex
    def link(self):
        lpnts = self.createLpList()
        cnt = len(lpnts)
        while True:
            startIndex = self.findUnusedPnt()
            if startIndex==-1: break
            p = lpnts[startIndex]
            poly = Polyline()
            while True:
                poly.addPoint(p.toPoint3D())
                p.used,p.other.used = True,True
                if poly.isClosed():
                    self.contours.append(poly)
                    break
                index = p.other.index
                if index-1>=0 and p.other.toPoint3D.isCoincide(lpnts[index-1].toPoint3D()):
                    p = lpnts[index-1]
                elif index+1<=cnt and p.other.toPoint3D.isCoincide(lpnts[index+1].toPoint3D()):
                    p = lpnts[index+1]
                else:
                    self.polys.append(poly)
                    break