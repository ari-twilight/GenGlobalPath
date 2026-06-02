from GeomBase import *
class Polyline:
    def __init__(self):
        self.points = []
    def __str__(self):
        if self.count()>0:
            return 'Polyline\nPoint number:%s\nStart%s\nEnd%s\n'%(self.count(),str(self.startPoint()),str(self.endPoint()))
        else: return 'Polyline\nPoint number:0\n'
    def clone(self):
        poly = Polyline()
        for pt in self.points:
            poly.addPoint(pt.clone())
        return poly
    def count(self):
        return len(self.points)
    def addPoint(self,pt):
        self.points.append(pt)
    def addTuple(self,tuple):
        self.points.append(Point3D(tuple[0],tuple[1],tuple[2]))
    def raddPoint(self,pt):
        self.points.insert(0,pt)
    def removePoint(self,index):
        return self.points.pop(index)
    def point(self,index):
        return self.points[index]
    def startPoint(self):
        return self.points[0]
    def endPoint(self):
        return self.points[-1]
    def isClosed(self):
        if self.count() <= 2: return False
        return self.startPoint().isCoincide(self.endPoint())
    def reverse(self):
        sz = self.count()
        for i in range(int(sz/2)):
            self.points[i],self.points[sz-1-i] = self.points[sz-1-i],self.points[i]
    def getArea(self):
        area = 0.0
        for i in range(self.count()-1):
            area += 0.5 * (self.points[i].x*self.points[i+1].y-self.points[i].y*self.points[i+1].x)
        return area
    def makeCCW(self):
        if self.getArea() < 0: self.reverse()
    def makeCW(self):
        if self.getArea() > 0: self.reverse()
    def isCCW(self):
        return True if self.getArea()>0 else False
    def translate(self,vec):
        for i in range(len(self.points)):
            self.points[i].translate(vec)
    def appendSegment(self,seg):
        if self.count() == 0:
            self.points.append(seg.A)
            self.points.append(seg.B)
        else:
            if seg.A.isCoincide(self.endPoint()): self.addPoint(seg.B)
            elif seg.B.isCoincide(self.endPoint()): self.addPoint(seg.A)
            elif seg.A.isCoincide(self.startPoint()): self.raddPoint(seg.B)
            elif seg.B.isCoincide(self.startPoint()): self.raddPoint(seg.A)
            else: return False
        return True
    def multiply(self,m):
        for pt in self.points:
            pt.multiply(m)
    def multiplied(self,m):
        poly = Polyline()
        for pt in self.points:
            poly.addPoint(pt*m)
        return poly
    @staticmethod
    def writePolyline(path,polyline):
        f = None
        try:
            f = open(path,'w')
            f.write('%s\n'%polyline.count())
            for pt in polyline.points:
                txt = '%s,%s,%s\n'%(pt.x,pt.y,pt.z)
                f.write(txt)
        except Exception as ex:
            print(ex)
        finally:
            if f: f.close()
    @staticmethod
    def readPolyline(path):
        f = None
        try:
            f,poly = open(path,'r'),Polyline()
            number = int(f.readline())
            for i in range(number):
                txt = f.readline()
                txts = txt.split(',')
                x,y,z = float(txts[0]),float(txts[1]),float(txts[2])
                poly.addPoint(Point3D(x,y,z))
            return poly
        except Exception as ex:
            print(ex)
        finally:
            if f: f.close()