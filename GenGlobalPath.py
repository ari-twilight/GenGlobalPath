import math
from GeomBase import *
from ClipperAdaptor import *
from Polyline import *
from Segment import *
from GeomAlgo import *
from GenHatch import calHatchPoints


class PathGroup:
    """路径元素分组类：存放一个区域内的外轮廓、中介轮廓、填充线段"""

    def __init__(self, outer_contour, intermediate_contour):
        self.outer_contour = outer_contour  # 外轮廓 (用于最终打印的边界)
        self.intermediate_contour = intermediate_contour  # 中介轮廓 (用于剪切平行线和引导Zigzag)
        self.clipped_lines = []  # 被中介轮廓剪切后的平行线
        self.contour_points = []  # 中介轮廓上的高密度插值点
        self.composite_zigzag = Polyline()  # 局部连接生成的复合 Zigzag
        self.local_continuous_path = Polyline()  # 复合 Zigzag 与外轮廓缝合后的局部一笔画


class ContinuousPathPlanner:
    def __init__(self, polygons, bead_width, overlap_rate):
        self.polygons = polygons
        self.bead_width = bead_width
        self.overlap_rate = overlap_rate
        self.groups = []
        self.global_path = Polyline()

    def execute(self):
        """算法主控引擎：严格执行论文的四个步骤"""
        print("步骤 1: 生成路径元素...")
        self.generate_path_elements()

        print("步骤 2: 组合路径元素...")
        self.combine_path_elements()

        print("步骤 3: 局部连接 (缝合组内的一笔画)...")
        self.local_connection()

        print("步骤 4: 全局连接 (串联所有孤岛)...")
        self.global_connection()

        return self.global_path

    # ==========================================
    # 步骤 1: 路径元素生成
    # ==========================================
    def generate_path_elements(self):
        ca = ClipperAdaptor()

        # 1. 获取全局 Y 轴包围盒 (为生成铺满全图的平行线做准备)
        self.yMin, self.yMax = float('inf'), float('-inf')
        for poly in self.polygons:
            for pt in poly.points:
                self.yMin, self.yMax = min(self.yMin, pt.y), max(self.yMax, pt.y)

        # 2. 生成外轮廓和中介轮廓
        for poly in self.polygons:
            # 外轮廓：偏移 0mm (如果是真实加工，这里可以填入收缩补偿值)
            outer_contours = ca.offset([poly], 0)
            if not outer_contours: continue
            outer_contour = outer_contours[0]

            # 中介轮廓：向内收缩 (线宽 * (1 - 搭接率))
            offset_dist = self.bead_width * (1 - self.overlap_rate)
            # 注意：向内偏置在 Clipper 中距离为负数
            intermediate_contours = ca.offset([poly], -offset_dist)

            if intermediate_contours:
                intermediate_contour = intermediate_contours[0]
                group = PathGroup(outer_contour, intermediate_contour)
                self.groups.append(group)

        # 3. 生成并剪切平行线
        self.all_clipped_lines = self._generate_and_clip_parallel_lines()

    def _generate_and_clip_parallel_lines(self):
        """生成全局平行线，并用所有的中介轮廓去剪切它们"""
        ys = []
        y = self.yMin + self.bead_width
        while y < self.yMax:
            ys.append(y)
            y += self.bead_width

        # 收集所有的中介轮廓作为“剪刀/饼干模具”
        intermediate_polys = [g.intermediate_contour for g in self.groups]

        # 利用你之前写的 calHatchPoints 扫描线引擎，直接求出落在中介轮廓内的交点
        ipses = calHatchPoints(intermediate_polys, ys)

        clipped_lines = []
        vx = Vector3D(1, 0, 0)
        shrink_dist = self.bead_width * 0.1  # 两端稍微向内收缩，留出搭接余量

        for ips in ipses:
            for i in range(0, len(ips), 2):
                if i + 1 < len(ips):
                    p1, p2 = ips[i], ips[i + 1]
                    # 向内收缩端点
                    p1 = p1 + vx.amplified(shrink_dist)
                    p2 = p2 - vx.amplified(shrink_dist)
                    if p1.x < p2.x:  # 确保线段合法
                        clipped_lines.append(Segment(p1, p2))

        return clipped_lines

    # ==========================================
    # 步骤 2: 路径元素组合
    # ==========================================
    def combine_path_elements(self):
        for group in self.groups:
            # 1. 对中介轮廓进行高密度插值 (距离必须小于线宽)
            group.contour_points = self._interpolate_contour(group.intermediate_contour, self.bead_width * 0.5)

            # 2. 将属于该中介轮廓的平行线认领回家
            for line in self.all_clipped_lines:
                # 如果线段的中点落在该中介轮廓内部，说明它属于这个组
                mid_pt = Point3D((line.A.x + line.B.x) / 2, (line.A.y + line.B.y) / 2, line.A.z)
                # 调用 GeomAlgo 中的点包含测试
                if pointInPolygon(mid_pt, group.intermediate_contour):
                    group.clipped_lines.append(line)

    def _interpolate_contour(self, poly, step):
        """将轮廓打碎成密集的点，用于引导前向探测"""
        points = []
        for i in range(poly.count() - 1):
            p1, p2 = poly.point(i), poly.point(i + 1)
            dist = math.sqrt(p1.distanceSquare(p2))
            num_steps = max(1, int(dist / step))
            for j in range(num_steps):
                ratio = j / num_steps
                new_x = p1.x + (p2.x - p1.x) * ratio
                new_y = p1.y + (p2.y - p1.y) * ratio
                points.append(Point3D(new_x, new_y, p1.z))
        return points

    # ==========================================
    # 步骤 3: 局部连接
    # ==========================================
    def local_connection(self):
        for group in self.groups:
            group.composite_zigzag = self._build_composite_zigzag(group.contour_points, group.clipped_lines)
            group.local_continuous_path = self._merge_loops(group.outer_contour, group.composite_zigzag)

    def _build_composite_zigzag(self, contour_points, clipped_lines):
        """修复版：贴边桥接 (Contour-Bridged Zigzag)"""
        composite_path = Polyline()
        if not clipped_lines:
            # 如果这块区域太小没有填充线，直接把轮廓画一圈返回
            for pt in contour_points:
                composite_path.addPoint(pt.clone())
            if composite_path.count() > 0:
                composite_path.addPoint(composite_path.startPoint().clone())
            return composite_path

        # 1. 构建闭合的轮廓环，用于提供“贴边走的桥梁”
        dense_poly = Polyline()
        for pt in contour_points:
            dense_poly.addPoint(pt.clone())
        if dense_poly.count() > 0:
            dense_poly.addPoint(dense_poly.startPoint().clone())

        # --- 内部辅助函数：计算两点之间沿着轮廓的最短路径 ---
        def get_path_along_contour(p_from, p_to):
            idx_from, idx_to = 0, 0
            min_d_from, min_d_to = float('inf'), float('inf')

            # 找到起点在轮廓上的最近索引
            for i in range(dense_poly.count() - 1):
                d = dense_poly.point(i).distanceSquare(p_from)
                if d < min_d_from:
                    min_d_from, idx_from = d, i

            # 找到终点在轮廓上的最近索引
            for i in range(dense_poly.count() - 1):
                d = dense_poly.point(i).distanceSquare(p_to)
                if d < min_d_to:
                    min_d_to, idx_to = d, i

            path = []
            n = dense_poly.count() - 1
            if n <= 0: return path

            # 比较顺时针走和逆时针走，哪个步数少就选哪个
            steps_forward = (idx_to - idx_from) % n
            steps_backward = (idx_from - idx_to) % n

            if steps_forward <= steps_backward:
                for i in range(steps_forward + 1):
                    idx = (idx_from + i) % n
                    path.append(dense_poly.point(idx).clone())
            else:
                for i in range(steps_backward + 1):
                    idx = (idx_from - i) % n
                    path.append(dense_poly.point(idx).clone())
            return path

        # ----------------------------------------------------

        # 2. 将填充线按 Y 坐标从下到上排序
        clipped_lines.sort(key=lambda seg: (seg.A.y + seg.B.y) / 2.0)

        # 3. 开始织网 (Zigzag 缝合)
        for i in range(len(clipped_lines)):
            line = clipped_lines[i]

            # 确保认清哪边是左，哪边是右
            p_left = line.B if line.A.x > line.B.x else line.A
            p_right = line.A if line.A.x > line.B.x else line.B

            # 奇偶交替法则：偶数行从左往右走，奇数行从右往左走
            p_in, p_out = (p_left, p_right) if i % 2 == 0 else (p_right, p_left)

            # 如果不是第一条线，需要找一条从【上一条线的出口】沿着边缘走到【这条线的入口】的桥
            if i > 0:
                prev_line = clipped_lines[i - 1]
                prev_left = prev_line.B if prev_line.A.x > prev_line.B.x else prev_line.A
                prev_right = prev_line.A if prev_line.A.x > prev_line.B.x else prev_line.B
                prev_out = prev_right if (i - 1) % 2 == 0 else prev_left

                bridge_path = get_path_along_contour(prev_out, p_in)
                for pt in bridge_path:
                    composite_path.addPoint(pt)
            else:
                composite_path.addPoint(p_in.clone())

            # 存入这条直线的出口点
            composite_path.addPoint(p_out.clone())

        # 4. 完美闭环：从最后一条线的出口，沿着轮廓桥接回第一条线的入口
        first_line = clipped_lines[0]
        first_in = first_line.A if first_line.A.x < first_line.B.x else first_line.B
        last_line = clipped_lines[-1]
        last_left = last_line.B if last_line.A.x > last_line.B.x else last_line.A
        last_right = last_line.A if last_line.A.x > last_line.B.x else last_line.B
        last_out = last_right if (len(clipped_lines) - 1) % 2 == 0 else last_left

        final_bridge = get_path_along_contour(last_out, first_in)
        for pt in final_bridge:
            composite_path.addPoint(pt)

        return composite_path

    def _merge_loops(self, loop1, loop2):
        """核心缝合手术：将两个封闭多边形在距离最近处打断并首尾相连"""
        if loop1.count() < 2 or loop2.count() < 2:
            return loop1  # 兜底保护

        # 1. 寻找最近的搭桥点
        min_dist = float('inf')
        idx1, idx2 = 0, 0
        for i in range(loop1.count() - 1):  # 忽略最后一个闭合点
            for j in range(loop2.count() - 1):
                d = loop1.point(i).distanceSquare(loop2.point(j))
                if d < min_dist:
                    min_dist, idx1, idx2 = d, i, j

        # 2. 做外科手术拼接 (大鱼吃小鱼)
        new_poly = Polyline()

        # 走 loop1 的前半段 (0 到 idx1)
        for i in range(idx1 + 1):
            new_poly.addPoint(loop1.point(i).clone())

        # 跳到 loop2，把 loop2 完整走一圈 (idx2 走到尾，再从头走到 idx2)
        for i in range(idx2, loop2.count() - 1):
            new_poly.addPoint(loop2.point(i).clone())
        for i in range(0, idx2 + 1):
            new_poly.addPoint(loop2.point(i).clone())

        # 跳回 loop1，走完剩下的路
        for i in range(idx1 + 1, loop1.count()):
            new_poly.addPoint(loop1.point(i).clone())

        return new_poly

        # ==========================================
        # 步骤 4: 全局连接 (支持孤岛断裂)
        # ==========================================
    def global_connection(self):
            self.global_paths = []  # ⚠️ 注意：这里变成了一个列表，存放一层里的多条“一笔画”
            if len(self.groups) == 0: return

            # 依然根据 X 坐标排序，优化搜索
            for g in self.groups:
                x_min = min(pt.x for pt in g.local_continuous_path.points)
                g.sort_key = x_min
            self.groups.sort(key=lambda g: g.sort_key)

            unconnected_groups = self.groups[:]  # 复制一份待连接的组

            # 设定一个极限跨越距离 (例如：线宽的 1.5 倍)
            # 如果两个岛屿的距离超过这个值，绝对不能连！
            max_jump_dist_sq = (self.bead_width * 1.5) ** 2

            while unconnected_groups:
                # 拿出一个孤岛作为当前主路径
                current_path = unconnected_groups.pop(0).local_continuous_path

                merged_something = True
                while merged_something and unconnected_groups:
                    merged_something = False
                    best_idx = -1
                    min_dist = float('inf')

                    # 在剩下的组里，找离当前路径最近的组
                    for i, candidate_group in enumerate(unconnected_groups):
                        dist = self._min_distance_between_paths(current_path, candidate_group.local_continuous_path)
                        if dist < min_dist:
                            min_dist = dist
                            best_idx = i

                    # 🛑 核心判决：最近的组，距离够近吗？
                    if best_idx != -1 and min_dist <= max_jump_dist_sq:
                        # 距离够近 (比如只是被中介轮廓切开的同一个零件的裂缝)，缝合它们！
                        next_group = unconnected_groups.pop(best_idx)
                        current_path = self._merge_loops(current_path, next_group.local_continuous_path)
                        merged_something = True

                # 当找不到任何足够近的组可以缝合时，这条路径彻底宣告完成
                # (比如左耳已经缝合完了内部所有细节，接下来离它最近的是右耳，但右耳太远了)
                self.global_paths.append(current_path)

    def _min_distance_between_paths(self, path1, path2):
            """计算两条路径之间的最短平方距离"""
            min_dist = float('inf')
            for i in range(path1.count()):
                p1 = path1.point(i)
                for j in range(path2.count()):
                    d = p1.distanceSquare(path2.point(j))
                    if d < min_dist:
                        min_dist = d
            return min_dist


def continuousPathPlanner(polygons, bead_width, overlap_rate=0.5):
    """
    全局连续轨迹规划接口函数 (终极高度强制修复版)
    """
    if len(polygons) == 0:
        return []

    # 1. 偷看原始轮廓，记住这一层真正的 Z 高度
    target_z = polygons[0].points[0].z

    # 2. 执行算法（此时底层可能会把 Z 降维成 0）
    planner = ContinuousPathPlanner(polygons, bead_width, overlap_rate)
    planner.execute()

    # 3. ⭐️ 终极修复：放弃修改原路径，强制克隆并重建 3D 路径
    fixed_paths = []
    for path in planner.global_paths:
        new_path = Polyline()  # 造一条全新的空路径

        # 遍历原来的 2D 点，抽出 X 和 Y，强行注入 target_z 组合成新的 3D 点
        for pt in path.points:
            # 绝对无视它原来的 z，直接用 Target_Z 实例化新点
            new_3d_point = Point3D(pt.x, pt.y, target_z)
            new_path.addPoint(new_3d_point)

        fixed_paths.append(new_path)

    # 返回强制带上 3D 高度的新路径！
    return fixed_paths