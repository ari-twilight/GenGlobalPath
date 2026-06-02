from GeomBase import *
from Line import *
class Plane:
    def __init__(self,P,N):
        self.P = P.clone()
        self.N = N.clone().normalized()
    def __str__(self):
        return "Plane\n%s\n%s"%(str(self.P),str(self.N))
    def toFormula(self):
        A,B,C = self.N.dx,self.N.dy,self.N.dz
        D = -self.P.x*self.N.dx - self.P.y*self.N.dy - self.P.z*self.N.dz
        return A,B,C,D
    @staticmethod
    def zPlane(z):
        return Plane(Point3D(0,0,z),Vector3D(0,0,1.0))
    def intersect(self,other):
        dir = self.N.crossProduct(other.N)
        if dir.isZeroVector():
            return None
        else:
            x,y,z = 0,0,0
            A1,B1,C1,D1 = self.toFormula()
            A2,B2,C2,D2 = other.toFormula()
            if B2*C1-B1*C2 != 0:
                y = -(-C2*D1+C1*D2)/(B2*C1-B1*C2)
                z = -(B2*D1-B1*D2)/(B2*C1-B1*C2)
            elif A2*C1-A1*C2 != 0:
                x = -(-C2*D1+C1*D2)/(A2*C1-A1*C2)
                z = -(A2*D1-A1*D2)/(A2*C1-A1*C2)
            elif A2*B1-A1*B2 != 0:
                x = -(-B2*D1+B1*D2)/(A2*B1-A1*B2)
                y = -(A2*D1-A1*D2)/(A2*B1-A1*B2)
            else: return None
            return Line(Point3D(x,y,z),dir.normalized())