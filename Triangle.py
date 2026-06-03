from GeomBase import *
class Triangle:
    def __init__(self,A,B,C,N=Vector3D(0,0,0)):
        self.A,self.B,self.C,self.N = A.clone(),B.clone(),C.clone(),N.clone()
        self.zs=[]
    def __str__(self):
        return 'Triangle:%s,%s,%s,%s' % (self.A, self.B, self.C,self.N)
    def zMinPnt(self):
        z = min(self.A.z,self.B.z,self.C.z)
        if z == self.A.z:
            return self.A
        elif z == self.B.z:
            return self.B
        elif z == self.C.z:
            return self.C
    def zMaxPnt(self):
        z = max(self.A.z, self.B.z, self.C.z)
        if z == self.A.z:
            return self.A
        elif z == self.B.z:
            return self.B
        elif z == self.C.z:
            return self.C

    def calcNormal(self):
        # 1. 算出从 A 到 B 的向量
        v1 = self.A.pointTo(self.B)
        # 2. 算出从 A 到 C 的向量
        v2 = self.A.pointTo(self.C)
        # 3. 两个向量叉乘，得到垂直于这个面的向量
        normal = v1.crossProduct(v2)
        # 4. 将其归一化（变成长度为1的单位向量）
        normal.normalize()
        self.N = normal
        return self.N