from GenGlobalPath import *  # 确保这里导入的是上一节最新版的代码
from SliceAlgo import *
from Utility import *  # ⚠️ 关键修复：必须导入，VtkAdaptor 依赖它
import time

bead_width = 3  # 焊道宽度 (线宽 3mm，通常用于大型金属沉积)
overlap_rate = 0.5  # 搭接率 50%

# 你可以在这里自定义孤岛断开的极限距离 (可选)
# 默认底层逻辑是 bead_width * 1.5 = 4.5mm。距离超过这个值的区域会被视为孤岛，智能断开。
# custom_max_link_dist = 4.5

global_paths = []  # 存放全图所有生成的真实物理路径
layers = readSlcFile("D:\\bunny at 1.5mm.slc")

print("🚀 开始执行全局连续轨迹规划...")
start = time.perf_counter()

for i, layer in enumerate(layers):
    print(f'处理进度: 层 {i + 1}/{len(layers)}')

    if len(layer.contours) > 0:
        # 调用最终版的接口：
        # 它现在自带 Z 高度修复，不再会坠毁到地板上；
        # 遇到兔子身体返回 1 条路径，遇到兔子双耳会智能断开返回 2 条路径！
        paths = continuousPathPlanner(
            layer.contours,
            bead_width,
            overlap_rate
            # ,max_global_link_distance=custom_max_link_dist # 若想调整断开敏感度，可传此参数
        )

        # extend 会把这一层返回的 1条 或 多条 路径，全部平铺塞进总名单里
        global_paths.extend(paths)

end = time.perf_counter()
print(f"🎉 GenGlobalPath 运算完成! 耗时: {end - start:.4f} 秒")

# ==========================================
# 启动 3D 渲染
# ==========================================
print("启动 3D 可视化窗口...")
va = VtkAdaptor()

# 1. 绘制最外层原始轮廓 (黑色)
for layer in layers:
    for contour in layer.contours:
        if contour is not None and contour.count() > 0:
            va.drawPolyline(contour).GetProperty().SetColor(0, 0, 0)  # 黑色

# 2. 绘制生成的全局一笔画路径 (红色)
for path in global_paths:
    # 现在的 path 都是已经带上正确 Z 高度的完美 3D 路径
    if path is not None and path.count() > 0:
        va.drawPolyline(path).GetProperty().SetColor(1, 0, 0)  # 红色

va.display()