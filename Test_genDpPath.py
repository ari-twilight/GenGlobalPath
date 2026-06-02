from GenDpPath import *
from SliceAlgo import *
import time
from Utility import *

interval = 3
angle = 0
pathses = []
layers = readSlcFile("D:\\bunny at 1.5mm.slc")
start = time.perf_counter()
for i in range(len(layers)):
    print('dp,layer:%d/%d'%(i+1,len(layers)))
    theta = degToRad(angle) if (i%2==1) else degToRad(angle+180)
    paths = genDpPath(layers[i].contours,interval,theta)
    pathses.append(paths)
end = time.perf_counter()
print("GenDpPath time: %f CPU seconds"%(end-start))
va = VtkAdaptor()
for layer in layers:
    for contour in layer.contours:
        va.drawPolyline(contour).GetProperty().SetColor(0,0,0)
for paths in pathses:
    for path in paths:
        va.drawPolyline(path).GetProperty().SetColor(1,0,0)
va.display()