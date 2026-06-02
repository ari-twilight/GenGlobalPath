from GenCpPath import *
from SliceAlgo import *
import time

interval = 0.4
shellThk = 2.0
pathses = []
layers = readSlcFile("D:\\bunny at 1.5mm.slc")
start = time.perf_counter()
for i in range(len(layers)):
    print('cp,layer:%d/%d'%(i+1,len(layers)))
    if len(layers[i].contours) > 0:
        paths = genCpPath(layers[i].contours, interval, shellThk)
        pathses.append(paths)
    else:
        pathses.append([])
end = time.perf_counter()
print("GenCpPath time:%f CPU seconds"%(end-start))
va = VtkAdaptor()
for layer in layers:
    for contour in layer.contours:
        va.drawPolyline(contour).GetProperty().SetColor(0,0,0)
for paths in pathses:
    for path in paths:
        va.drawPolyline(path).GetProperty().SetColor(1,0,0)
va.display()