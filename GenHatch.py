from GeomBase import *
from GeomAlgo import *
from Segment import *
from Line import *
from Polyline import *
from ClipperAdaptor import *

class SweepPlane:
    def __init__(self):
        self.segs = []
    def intersect(self,y):
        ips = []
        yLine = Line(Point3D(0,y,self.segs[0].A.z),Vector3D(1,0,0))
        for seg in self.segs:
            if seg.A.y == y:
                ips.append(seg.A.clone())
            elif seg.B.y == y:
                ips.append(seg.B.clone())
            else:
                ip = intersect(yLine,seg)
                if ip is not None:
                    ips.append(ip)
        ips.sort(key=lambda t:t.x)
        i = len(ips) - 1
        while i>0:
            if ips[i].distanceSquare(ips[i-1])==0:
                del ips[i]
                del ips[i-1]
                i = i-2
            else:
                i = i-1
        return ips

def calHatchPoints(polygons,ys):
    segs = []
    for poly in polygons:
        for i in range(poly.count()-1):
            seg = Segment(poly.point(i),poly.point(i+1))
            seg.yMin = min(seg.A.y,seg.B.y)
            seg.yMax = max(seg.A.y, seg.B.y)
            segs.append(seg)
    segs.sort(key=lambda t:t.yMin)
    k,sweep = 0,SweepPlane()
    ipses = []
    for y in ys:
        for i in range(len(sweep.segs)-1,-1,-1):
            if sweep.segs[i].yMax < y:
                del sweep.segs[i]
        for i in range(k,len(segs)):
            if segs[i].yMin < y and segs[i].yMax >= y:
                sweep.segs.append(segs[i])
            elif segs[i].yMin >= y:
                k=i
                break
        if len(sweep.segs)>0:
            ips = sweep.intersect(y)
            ipses.append(ips)
    return ipses

def genSweepHatches(polygons,interval,angle):
    mt = Matrix3D.createRotateMatrix('Z',-angle)
    mb = Matrix3D.createRotateMatrix('Z',angle)
    rotpolys = []
    for poly in polygons:
        rotpolys.append(poly.multiplied(mt))
    yMin,yMax = float('inf'),float('-inf')
    for poly in rotpolys :
        for pt in poly.points:
            yMin,yMax = min(yMin,pt.y),max(yMax,pt.y)
    ys = []
    y = yMin + interval
    while y < yMax:
        ys.append(y)
        y += interval
    segs = genHatches(rotpolys,ys)
    for seg in segs:
        seg.multiply(mb)
    return segs

def genHatches(polygons,ys):
    segs = []
    ipses = calHatchPoints(polygons,ys)
    for ips in ipses:
        for i in range(0,len(ips),2):
            seg = Segment(ips[i],ips[i+1])
            segs.append(seg)
    return segs

def genClipHatches(polygons,interval,angle):
    xMin,xMax = float('inf'),float('-inf')
    yMin,yMax = float('inf'),float('-inf')
    z = polygons[0].points[0].z
    for poly in polygons:
        for pt in poly.points:
            xMin,xMax = min(xMin,pt.x),max(xMax,pt.x)
            yMin,yMax = min(yMin,pt.y),max(yMax,pt.y)
    v = Vector3D(math.cos(angle),math.sin(angle))
    n = Vector3D(math.cos(angle+math.pi/2),math.sin(angle+math.pi/2))
    O = Point3D((xMin+xMax)/2,(yMin+yMax)/2,z)
    R = math.sqrt((xMax-xMin)**2+(yMax-yMin)**2)/2
    P1 = O-n.amplified(R)
    parallels = []
    for i in range(0,int(2*R/interval)+1,1):
        Q = P1 + n.amplified(interval*i)
        seg = Polyline()
        seg.addPoint(Q-v.amplified(R))
        seg.addPoint(Q+v.amplified(R))
        parallels.append(seg)
    hatchSegs = []
    ca = ClipperAdaptor()
    clipper = Pyclipper()
    clipper.AddPaths(ca.toPaths(polygons),PT_CLIP,True)
    clipper.AddPaths(ca.toPaths(parallels),PT_SUBJECT,False)
    sln = clipper.Execute2(CT_INTERSECTION)
    for child in sln.childs:
        if len(child.Contour)>0:
            poly = ca.toPoly(child.Contour,z,False)
            seg = Segment(poly.startPoint(),poly.endPoint())
            hatchSegs.append(seg)
    return hatchSegs