from GeomBase import *
from Line import *
from Ray import *
from Segment import *
from Plane import *
from Polyline import *
from Triangle import *
def nearZero(x):
    return True if math.fabs(x) < epsilon else False
def distance(obj1,obj2):
    if isinstance(obj1,Point3D) and isinstance(obj2,Line):
        P,Q,V = obj2.P,obj1,obj2.V
        t = P.pointTo(Q).dotProduct(V)
        R = P+V.amplified(t)
        return Q.distance(R)
    elif isinstance(obj1,Point3D) and isinstance(obj2,Ray):
        P,Q,V = obj2.P,obj1,obj2.V
        t = P.pointTo(Q).dotProduct(V)
        if t>=0:
            R = P+V.amplified(t)
            return Q.distance(R)
        return Q.distance(P)
    elif isinstance(obj1,Point3D) and isinstance(obj2,Segment):
        Q,P,P1,V = obj1,obj2.A,obj2.B,obj2.direction().normalized()
        L = obj2.length()
        t = P.pointTo(Q).dotProduct(V)
        if t<0: return Q.distance(P)
        elif t>=L: return Q.distance(P1)
        else:
            R = P + V.amplified(t)
            return Q.distance(R)
    elif isinstance(obj1,Point3D) and isinstance(obj2,Plane):
        P,Q,N = obj2.P,obj1,obj2.N
        angle = N.getAngle(P.pointTo(Q))
        return P.distance(Q)*math.cos(angle)
    elif isinstance(obj1,Line) and isinstance(obj2,Plane):
        return distance(obj1.P,obj2) if obj1.V.dotProduct(Plane.N)==0 else 0
    elif isinstance(obj1,Ray) and isinstance(obj2,Plane):
        P = intersect(obj1,obj2)
        return distance(obj1.P,obj2) if P is not None else 0
    elif isinstance(obj1,Segment) and isinstance(obj2,Plane):
        P = intersect(obj1, obj2)
        return min(distance(obj1.A,obj2),distance(obj1.B,obj2)) if P is not None else 0
    pass
def intersectLineLine(line1:Line,line2:Line):
    P1,V1,P2,V2 = line1.P,line1.V,line2.P,line2.V
    P1P2 = P1.pointTo(P2)
    deno = V1.dy*V2.dx-V1.dx*V2.dy
    if deno != 0:
        t1 = -(-P1P2.dy*V2.dx+P1P2.dx*V2.dy)/deno
        t2 = -(-P1P2.dy*V1.dx+P1P2.dx*V1.dy)/deno
        return P1+V1.amplified(t1),t1,t2
    else:
        deno = V1.dz*V2.dy-V1.dy*V2.dz
        if deno!=0:
            t1 = -(-P1P2.dz*V2.dy+P1P2.dy*V2.dz)/deno
            t2 = -(-P1P2.dz*V1.dy+P1P2.dy*V1.dz)/deno
            return P1+V1.amplified(t1),t1,t2
    return None,0,0
def intersectSegmentPlane(seg:Segment,plane:Plane):
    A,B,P,N = seg.A,seg.B,plane.P,plane.N
    V = A.pointTo(B)
    PA = P.pointTo(A)
    if V.dotProduct(N) == 0: return None
    else:
        t = -(PA.dotProduct(N))/V.dotProduct(N)
        if t>=0 and t<=1: return A+V.amplified(t)
    return None
def intersect(obj1,obj2):
    if isinstance(obj1,Line) and isinstance(obj2,Line):
        P,t1,t2 = intersectLineLine(obj1,obj2)
        return P
    elif isinstance(obj1,Segment) and isinstance(obj2,Segment):
        line1,line2 = Line(obj1.A,obj1.direction()),Line(obj2.A,obj2.direction())
        P,t1,t2 = intersectLineLine(line1,line2)
        if P is not None:
            if t1>=0 and t1<=obj1.length() and t2>=0 and t2<=obj2.length():
                return P
        return None
    elif isinstance(obj1,Line) and isinstance(obj2,Segment):
        line = Line(obj2.A,obj2.direction())
        P,t1,t2 = intersectLineLine(obj1,line)
        return P if P is not None and t2>=0 and t2<=obj2.length() else None
    elif isinstance(obj1,Line) and isinstance(obj2,Ray):
        line = Line(obj2.P,obj2.V)
        P,t1,t2 = intersectLineLine(obj1,line)
        return P if P is not None and t2 >= 0 else None
    elif isinstance(obj1,Ray) and isinstance(obj2,Ray):
        line1,line2 = Line(obj1.P,obj1.V),Line(obj2.P,obj2.V)
        P, t1, t2 = intersectLineLine(line1, line2)
        return P if P is not None and t1>=0 and t2 >= 0 else None
    elif isinstance(obj1,Ray) and isinstance(obj2,Segment):
        line1, line2 = Line(obj1.P, obj1.V), Line(obj2.A, obj2.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        return P if P is not None and t1 >= 0 and t2 >= 0 and t2<=obj2.length() else None
    elif isinstance(obj1,Line) and isinstance(obj2,Plane):
        P0,V,P1,N = obj1.P,obj1.V,obj2.P,obj2.N
        dotPro = V.dotProduct(N)
        if dotPro != 0:
            t = P0.pointTo(P1).dotProduct(N)/dotPro
            return P0+V.amplified(t)
        return None
    elif isinstance(obj1,Ray) and isinstance(obj2,Plane):
        P0,V,P1,N = obj1.P,obj1.V,obj2.P,obj2.N
        dotPro = V.dotProduct(N)
        if dotPro != 0:
            t = P0.pointTo(P1).dotProduct(N)/dotPro
            return P0 + V.amplified(t) if t>=0 else None
        return None
    elif isinstance(obj1,Segment) and isinstance(obj2,Plane):
        return intersectSegmentPlane(obj1,obj2)
    pass
def pointOnRay(p:Point3D,ray:Ray):
    v = ray.P.pointTo(p)
    return True if v.dotProduct(ray.V)>=0 and v.crossProduct(ray.V).isZeroVector() else False
def pointInPolygon(p:Point3D,polygon:Polyline):
    passCount = 0
    ray = Ray(p,Vector3D(1,0,0))
    segments = [] #返回-1：点在多边形上
    for i in range(polygon.count()-1):
        seg = Segment(polygon.point(i),polygon.point(i+1))
        segments.append(seg)
    for seg in segments:
        line1,line2 = Line(ray.P,ray.V),Line(seg.A,seg.direction())
        P,t1,t2 = intersectLineLine(line1, line2)
        if P is not None:
            if nearZero(t1): return -1
            elif seg.A.y!=p.y and seg.B.y!=p.y and t1>0 and t2>0 and t2<seg.length():
                passCount+=1
    upSegments,downSegments = [],[]
    for seg in segments:
        if seg.A.isIdentical(ray.P) or seg.B.isIdentical(ray.P):
            return -1
        elif pointOnRay(seg.A,ray)^pointOnRay(seg.B,ray):
            if seg.A.y>=p.y and seg.B.y>=p.y: upSegments.append(seg)
            elif seg.A.y<=p.y and seg.B.y<=p.y: downSegments.append(seg)
    passCount+=min(len(upSegments),len(downSegments))
    if passCount%2 == 1:
        return 1 #点在多边形内部
    return 0 #否则点在多边形外部
def intersectTrianglePlane(triangle,plane):
    AB = Segment(triangle.A,triangle.B)
    AC = Segment(triangle.A,triangle.C)
    BC = Segment(triangle.B,triangle.C)
    c1 = intersectSegmentPlane(AB,plane)
    c2 = intersectSegmentPlane(AC,plane)
    c3 = intersectSegmentPlane(BC,plane)
    if c1 is None:
        if c2 is not None and c3 is not None:
            if c2.distance(c3)!=0.0:
                return Segment(c2,c3)
    elif c2 is None:
        if c1 is not None and c3 is not None:
            if c1.distance(c3)!=0:
                return Segment(c1,c3)
    elif c3 is None:
        if c1 is not None and c2 is not None:
            if c1.distance(c2)!=0:
                return Segment(c1,c2)
    elif c1 is not None and c2 is not None and c3 is not None:
        return Segment(c1,c3) if c1.isIdentical(c2) else Segment(c1,c2)
    return None
def intersectTriangleZPlane(triangle,z):
    if triangle.zMinPnt().z>z:
        return None
    if triangle.zMaxPnt().z<z:
        return None
    return intersectTrianglePlane(triangle,Plane.zPlane(z))
def adjustPolygonDirs(polygons):
    for i in range(len(polygons)):
        pt = polygons[i].startPoint()
        insideCount = 0
        for j in range(len(polygons)):
            if j == i: continue
            restPoly = polygons[j]
            if 1 == pointInPolygon(pt,restPoly):
                insideCount += 1
        if insideCount%2 == 0:
                polygons[i].makeCCW()
        else: polygons[i].makeCW()
def rotatePolygons(polygons,angle,center=None):
    dx = 0 if center is None else center.x
    dy = 0 if center is None else center.y
    mt = Matrix3D.createTranslateMatrix(-dx,-dy,0)
    mr = Matrix3D.createRotateMatrix('Z',angle)
    mb = Matrix3D.createTranslateMatrix(dx,dy,0)
    m = mt * mr * mb
    newPoly = []
    for poly in polygons:
        newPoly.append(poly.multiplied(m))
    return newPoly