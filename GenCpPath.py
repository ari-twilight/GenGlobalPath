from ClipperAdaptor import *
from GeomAlgo import *

class GenCpPath:
    def __init__(self,boundaries,interval,shellThk):
        self.boundaries = boundaries
        self.interval = interval
        self.shellThk = shellThk
        self.arcTolerance = 0.005
        self.jointType = JT_SQUARE
        self.offsetPolyses = []
        self.paths = []
        self.offset()
        self.linkLocalOffsets()
    def offset(self):
        ca = ClipperAdaptor()
        ca.arcTolerance = self.arcTolerance
        delta = self.interval/2
        polys = ca.offset(self.boundaries,-delta,self.jointType)
        self.offsetPolyses.append(polys)
        while math.fabs(delta)<self.shellThk:
            delta += self.interval
            polys = ca.offset(self.boundaries,-delta,self.jointType)
            if polys is None or len(polys) == 0: break
            self.offsetPolyses.append(polys)
    def linkToParent(self,child):
        parent = child.parent
        pt = child.startPoint()
        dMin,iAtdMin = float('inf'),0
        for i in range(parent.count()):
            d = pt.distanceSquare(parent.point(i))
            if d < dMin:
                dMin,iAtdMin = d,i
        newPoly = Polyline()
        for i in range(iAtdMin+1):
            newPoly.addPoint(parent.point(i).clone())
        newPoly.endPoint().w = 2
        for i in range(child.count()):
            newPoly.addPoint(child.point(i).clone())
        newPoly.endPoint().w = 2
        for i in range(iAtdMin,parent.count(),1):
            newPoly.addPoint(parent.point(i).clone())
        return newPoly

    def findParent(self):
        # 从第 1 圈开始遍历（因为第 0 圈是最外围，没有爹）
        for i in range(1, len(self.offsetPolyses)):
            childs = self.offsetPolyses[i]  # 当前圈（儿子候选群）
            parents = self.offsetPolyses[i - 1]  # 上一圈（爸爸候选群）

            for child in childs:
                # 拿儿子的第一个点作为“基因样本”
                pt = child.startPoint()
                # 去上一圈里挨个测试，看这个点落在谁的肚子里
                for parent in parents:
                    # 调用你之前在 GeomAlgo.py 中写的点在多边形内算法
                    if pointInPolygon(pt, parent):
                        child.parent = parent  # 滴血认亲成功！
                        break  # 找到了亲爹，立刻跳出循环，去帮下一个儿子找爹
    def linkLocalOffsets(self):
        self.findParent()
        for i in range(len(self.offsetPolyses)-1,0,-1):
            childs = self.offsetPolyses[i]
            for j in range(len(childs)-1,-1,-1):
                child = childs[j]
                newPoly = self.linkToParent(child)
                parent = child.parent
                parent.points = newPoly.points
                del childs[j]
        for path in self.offsetPolyses[0]:
            self.paths.append(path)
        self.offsetPolyses.clear()

def genCpPath(boundaries,interval,shellThk):
    return GenCpPath(boundaries,interval,shellThk).paths