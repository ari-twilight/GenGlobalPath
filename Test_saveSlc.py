from SliceAlgo import *

modelName = "hechai_3"
layerThk = 2
src = vtk.vtkSTLReader()
src.SetFileName("D:\STL\%s.stl"%modelName)
stlModel = StlModel()
stlModel.extractFromVtkStlReader(src)
layers = Slice_topo(stlModel,layerThk)
writeSlcFile(layers,"D:\%s at %smm.slc"%(modelName,layerThk))
print("SLC file has been generasted and saved")