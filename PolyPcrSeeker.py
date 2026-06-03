from pyclipper import *
from VtkAdaptor import *
import Utility
from GeomAlgo import *

class PolyPcrSeeker:
    def __init__(self,polys):
        self.polys = Utility.makeListLinear(polys)
        self.seek()
    def seek(self):
        polys = self.polys
        for poly in polys:
            poly.area = math.fabs(poly.getArea())
            poly.parent = None
            poly.childs = []
            poly.depth = 0
            polys.sort(key=lambda t:t.area)
            for i in range(0,len(polys)-1,1):
                for j in range(i+1,len(polys),1):
                    pt = polys[i].startPoint()
                    if pointInPolygon(pt,polys[j]):
                        polys[i].parent = polys[j]
                        polys[j].childs.append(polys[i])
                        break
            for poly in polys:
                self.findPolyDepth(poly)
            polys.sort(key=lambda t:t.depth)
    def findPolyDepth(self,poly):
        crtPoly = poly
        while crtPoly.parent is not None:
            crtPoly = crtPoly.parent
            poly.depth += 1

def SeekPolyPcr(polys):
    return PolyPcrSeeker(polys).polys