from LinkPoint import *
import SliceAlgo
from VtkAdaptor import *
from StlModel import *
from Layer import *
from GeomAlgo import *

class LinkSegs_dlook:
    def __init__(self,segs):
        self.segs = segs
        self.contours = []
        self.polys= []
        self.link()
    def link(self):
        dic = self.creatLpDic()
        while True:
            p = self.findUnusedPnt(dic)
            if p is None: break
            poly = Polyline()
            while True:
                poly.addPoint(p.toPoint3D())
                p.used,p.other.used = True,True
                p = self.findNextPnt(p,dic)
                if poly.isClosed():
                    self.contours.append(poly)
                    break
                if p is None:
                    self.polys.append(poly)
                    break
    def creatLpDic(self):
        dic = {}
        for seg in self.segs:
            lp1,lp2 = LinkPoint(seg.A),LinkPoint(seg.B)
            lp1.other,lp2.other = lp2,lp1
            if (lp1.x,lp1.y) not in dic.keys():
                dic[(lp1.x,lp1.y)] = []
            dic[(lp1.x,lp1.y)].append(lp1)
            if (lp2.x,lp2.y) not in dic.keys():
                dic[(lp2.x,lp2.y)] = []
            dic[(lp2.x,lp2.y)].append(lp2)
        return dic
    def findUnusedPnt(self,dic):
        for pnts in dic.values():
            for pnt in pnts:
                if pnt.used == False:
                    return pnt
        return None
    def findNextPnt(self,p,dic):
        other = p.other
        pnts = dic[(other.x,other.y)]
        difPnt = None
        for pnt in pnts:
            if pnt is not other:
                difPnt = pnt
        return difPnt