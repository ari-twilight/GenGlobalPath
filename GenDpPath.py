from GeomBase import *
import GeomAlgo
from Segment import *
from Line import *
from Polyline import *
from ClipperAdaptor import *
from GenHatch import *
from SplitRegion import *

class GenDpPath:
    def __init__(self,polygons,interval,angle):
        self.polygons,self.interval,self.angle = polygons,interval,angle
        self.splitPolys = []
    def generate(self):
        rotPolys = GeomAlgo.rotatePolygons(self.polygons,-self.angle)
        self.splitPolys = splitRegion(rotPolys)
        ys = self.genScanYs(rotPolys)
        paths = []
        for poly in self.splitPolys:
            segs = genHatches([poly],ys)
            if len(segs)>0:
                path = self.linkLocalHatches(segs)
                paths.append(path)
        return GeomAlgo.rotatePolygons(paths,self.angle)
    def genScanYs(self,polygons):
        ys,yMin,yMax = [],float('inf'),float('-inf')
        for poly in polygons:
            for pt in poly.points:
                yMin,yMax = min(yMin,pt.y),max(yMax,pt.y)
        y = yMin + self.interval
        while y < yMax:
            ys.append(y)
            y += self.interval
        return ys
    def linkLocalHatches(self,segs):
        poly = Polyline()
        for i,seg in enumerate(segs):
            poly.addPoint(seg.A if (i%2==0) else seg.B)
            poly.addPoint(seg.B if (i%2==0) else seg.A)
        return poly

def genDpPath(polygons,interval,angle):
    return GenDpPath(polygons,interval,angle).generate()