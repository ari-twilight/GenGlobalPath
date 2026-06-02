from GenGlobalPath import *
from SliceAlgo import *
import time

# from Utility import * # 确保你的 VtkAdaptor 等工具都正常导入

bead_width = 3  # 焊道宽度 (线宽 3mm，通常用于大型金属沉积)
overlap_rate = 0.5  # 搭接率 50%
global_paths = []  # 改个名字，因为里面装的是单条全局路径
layers = readSlcFile("D:\\bunny at 1.5mm.slc")

print("🚀 开始执行全局连续轨迹规划...")
start = time.perf_counter()
for i, layer in enumerate(layers):
    print(f'处理进度: 层 {i + 1}/{len(layers)}')

    if len(layer.contours) > 0:
        # paths 现在是一个列表 (比如 [左耳路径, 右耳路径])
        paths = continuousPathPlanner(layer.contours, bead_width, overlap_rate)
        # 用 extend 把这个列表里的线全塞进大名单里
        global_paths.extend(paths)
end = time.perf_counter()
print(f"🎉 GenGlobalPath 运算完成! 耗时: {end - start:.4f} 秒")

# 启动 3D 渲染
print("启动 3D 可视化窗口...")
va = VtkAdaptor()

# 绘制最外层原始轮廓 (黑色)
for layer in layers:
    for contour in layer.contours:
        va.drawPolyline(contour).GetProperty().SetColor(0, 0, 0)  # 黑色

# 绘制生成的全局一笔画路径 (红色)
for path in global_paths:
    if path is not None:
        # 因为每一层只有一条线，所以不需要内层循环了！
        va.drawPolyline(path).GetProperty().SetColor(1, 0, 0)  # 红色

va.display()