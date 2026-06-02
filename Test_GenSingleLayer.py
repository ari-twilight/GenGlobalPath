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
        # 1. 提前偷看这一层的真实高度！
        target_z = layer.contours[0].points[0].z

        # 2. 调用算法算出路径 (此时它可能还是坏的，Z=0)
        paths = continuousPathPlanner(layer.contours, bead_width, overlap_rate)

        # 3. 🎯 拦截行动：在装进大名单之前，强行把每一个点的 Z 换掉！
        for path in paths:
            # 遍历这条线里的每一个点
            for j in range(path.count()):
                old_pt = path.point(j)
                # 凭空造一个全新的 3D 点，X和Y照抄，Z强制使用 target_z
                # 并直接覆盖掉原来的点
                path.points[j] = Point3D(old_pt.x, old_pt.y, target_z)

        # 4. 把修复好的路径塞进大名单
        global_paths.extend(paths)
end = time.perf_counter()
print(f"🎉 GenGlobalPath 运算完成! 耗时: {end - start:.4f} 秒")
print("启动 3D 单层可视化窗口...")
va = VtkAdaptor()

# 🎯 设定你想看的那一层的索引
target_layer_idx = 40

# 1. 确保索引没有越界
if 0 <= target_layer_idx < len(layers):

    # 2. 只画这一层的黑色原始轮廓
    for contour in layers[target_layer_idx].contours:
        va.drawPolyline(contour).GetProperty().SetColor(0, 0, 0)

    # 3. 只画这一层的红色全局一笔画路径
    path = global_paths[target_layer_idx]
    if path is not None:
        va.drawPolyline(path).GetProperty().SetColor(1, 0, 0)

else:
    print("指定的层数不存在！")

va.display()