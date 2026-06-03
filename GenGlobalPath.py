import math
import heapq
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from GeomBase import *
from ClipperAdaptor import *
from Polyline import *
from Segment import *
from GeomAlgo import *
from GenHatch import calHatchPoints
from GenDpPath import genDpPath

def _clone_point(pt):
    return Point3D(pt.x, pt.y, pt.z, pt.w)

def _ensure_closed(poly):
    new_poly = poly.clone()
    if new_poly.count() > 0 and not new_poly.isClosed():
        new_poly.addPoint(new_poly.startPoint().clone())
    return new_poly

def _clean_closed_polygon(poly):
    cleaned = Polyline()
    for pt in poly.points:
        if cleaned.count() == 0 or not cleaned.endPoint().isCoincide(pt):
            cleaned.addPoint(pt.clone())
    if cleaned.count() > 1 and cleaned.startPoint().isCoincide(cleaned.endPoint()):
        cleaned.points.pop()
    if cleaned.count() > 0:
        cleaned.addPoint(cleaned.startPoint().clone())
    return cleaned

def _polygon_depth(poly, polygons):
    sample = poly.startPoint()
    depth = 0
    for other in polygons:
        if other is poly:
            continue
        if pointInPolygon(sample, other) == 1:
            depth += 1
    return depth

def _normalize_polygon_directions(polygons):
    normalized = []
    for poly in polygons:
        if poly.count() < 3:
            continue
        cleaned = _clean_closed_polygon(_ensure_closed(poly))
        if cleaned.count() >= 4 and abs(cleaned.getArea()) > 1e-9:
            normalized.append(cleaned)
    bboxes = [_bbox(poly) for poly in normalized]
    for i, poly in enumerate(normalized):
        poly.poly_index = i
        sample = poly.startPoint()
        depth = 0
        for j, other in enumerate(normalized):
            if i == j or not _point_in_bbox(sample, bboxes[j]):
                continue
            if pointInPolygon(sample, other) == 1:
                depth += 1
        poly.depth = depth
        if depth % 2 == 0:
            poly.makeCCW()
        else:
            poly.makeCW()
    for i, poly in enumerate(normalized):
        poly.parent_index = None
        if poly.depth == 0:
            continue
        sample = poly.startPoint()
        best_parent = None
        best_area = float("inf")
        for j, other in enumerate(normalized):
            if i == j or getattr(other, "depth", 0) != poly.depth - 1:
                continue
            if not _point_in_bbox(sample, bboxes[j]):
                continue
            if pointInPolygon(sample, other) == 1:
                area = abs(other.getArea())
                if area < best_area:
                    best_parent = j
                    best_area = area
        poly.parent_index = best_parent
    return normalized

def _bbox(poly):
    x_min = y_min = float("inf")
    x_max = y_max = float("-inf")
    for pt in poly.points:
        x_min = min(x_min, pt.x)
        y_min = min(y_min, pt.y)
        x_max = max(x_max, pt.x)
        y_max = max(y_max, pt.y)
    return x_min, y_min, x_max, y_max

def _point_in_bbox(pt, box, padding=0.0):
    return (
        box[0] - padding <= pt.x <= box[2] + padding
        and box[1] - padding <= pt.y <= box[3] + padding
    )

def _bbox_overlap(a, b, padding=0.0):
    return not (
        a[2] + padding < b[0]
        or b[2] + padding < a[0]
        or a[3] + padding < b[1]
        or b[3] + padding < a[1]
    )

def _point_distance_square(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return dx * dx + dy * dy + dz * dz

def _bbox_distance_square(a, b):
    if a[2] < b[0]:
        dx = b[0] - a[2]
    elif b[2] < a[0]:
        dx = a[0] - b[2]
    else:
        dx = 0.0

    if a[3] < b[1]:
        dy = b[1] - a[3]
    elif b[3] < a[1]:
        dy = a[1] - b[3]
    else:
        dy = 0.0
    return dx * dx + dy * dy

def _point_segment_distance_square(pt, a, b):
    ab = a.pointTo(b)
    length2 = ab.lengthSquare()
    if length2 == 0:
        return _point_distance_square(pt, a)
    ap = a.pointTo(pt)
    t = ap.dotProduct(ab) / length2
    if t < 0:
        return _point_distance_square(pt, a)
    if t > 1:
        return _point_distance_square(pt, b)
    foot = Point3D(a.x + ab.dx * t, a.y + ab.dy * t, a.z + ab.dz * t)
    return _point_distance_square(pt, foot)

def _polyline_length(poly):
    length = 0.0
    for i in range(poly.count() - 1):
        length += poly.point(i).distance(poly.point(i + 1))
    return length

def _path_from_closed_poly(poly, start_index, reverse=False):
    path = Polyline()
    n = poly.count() - 1 if poly.isClosed() else poly.count()
    if n <= 0:
        return path
    indexes = []
    if reverse:
        for i in range(n + 1):
            indexes.append((start_index - i) % n)
    else:
        for i in range(n + 1):
            indexes.append((start_index + i) % n)
    for idx in indexes:
        path.addPoint(poly.point(idx).clone())
    return path

def _open_contour_path(poly, start_index, end_index, reverse=False):
    points = []
    n = poly.count() - 1 if poly.isClosed() else poly.count()
    if n <= 0:
        return points
    idx = start_index % n
    end_index = end_index % n
    while True:
        points.append(poly.point(idx).clone())
        if idx == end_index:
            break
        idx = (idx - 1) % n if reverse else (idx + 1) % n
    return points

def _nearest_index(poly, pt):
    best_i = 0
    best_d = float("inf")
    n = poly.count() - 1 if poly.isClosed() else poly.count()
    for i in range(n):
        d = _point_distance_square(poly.point(i), pt)
        if d < best_d:
            best_i = i
            best_d = d
    return best_i, best_d

def _append_points(target, points, skip_duplicate=True):
    for pt in points:
        if skip_duplicate and target.count() > 0 and target.endPoint().isCoincide(pt):
            continue
        target.addPoint(pt.clone())

def _closed_count(poly):
    return poly.count() - 1 if poly.isClosed() else poly.count()

def _edge_pair_cost(points1, n1, points2, n2, i, j):
    a0 = points1[i]
    a1 = points1[(i + 1) % n1]
    b0 = points2[j]
    b1 = points2[(j + 1) % n2]
    reverse_cost = _point_distance_square(a0, b0) + _point_distance_square(a1, b1)
    forward_cost = _point_distance_square(a0, b1) + _point_distance_square(a1, b0)
    if reverse_cost <= forward_cost:
        return reverse_cost, True
    return forward_cost, False

def _edge_pair_connectors(points1, n1, points2, n2, i, j, reverse2):
    a0 = points1[i]
    a1 = points1[(i + 1) % n1]
    b0 = points2[j]
    b1 = points2[(j + 1) % n2]
    if reverse2:
        return a0, b0, b1, a1
    return a0, b1, b0, a1

def _build_kd_tree(items, depth=0):
    if not items:
        return None
    axis = depth % 2
    items.sort(key=lambda item: item[axis])
    mid = len(items) // 2
    return (
        items[mid],
        axis,
        _build_kd_tree(items[:mid], depth + 1),
        _build_kd_tree(items[mid + 1 :], depth + 1),
    )

def _query_kd_tree(node, point, k, heap):
    if node is None:
        return
    item, axis, left, right = node
    dx = point.x - item[0]
    dy = point.y - item[1]
    dist = dx * dx + dy * dy
    entry = (-dist, item[2])
    if len(heap) < k:
        heapq.heappush(heap, entry)
    elif dist < -heap[0][0]:
        heapq.heapreplace(heap, entry)

    diff = dx if axis == 0 else dy
    near, far = (left, right) if diff <= 0 else (right, left)
    _query_kd_tree(near, point, k, heap)
    if len(heap) < k or diff * diff < -heap[0][0]:
        _query_kd_tree(far, point, k, heap)

def _nearest_vertex_indexes(tree, point, k):
    heap = []
    _query_kd_tree(tree, point, k, heap)
    return [idx for _, idx in heap]

def _valid_edge_pair(points1, n1, points2, n2, i, j, reverse2, bridge_validator):
    if bridge_validator is None:
        return True
    c1_start, c1_end, c2_start, c2_end = _edge_pair_connectors(
        points1, n1, points2, n2, i, j, reverse2
    )
    return bridge_validator(c1_start, c1_end) and bridge_validator(c2_start, c2_end)

def _find_best_splice_fast(path1, path2, n1, n2, bridge_validator=None):
    points1 = path1.points
    points2 = path2.points
    tree = _build_kd_tree([(points1[i].x, points1[i].y, i) for i in range(n1)])
    candidate_pairs = set()
    nearest_count = min(24, n1)

    for j in range(n2):
        for i in _nearest_vertex_indexes(tree, points2[j], nearest_count):
            candidate_pairs.add((i, j))
            candidate_pairs.add(((i - 1) % n1, j))
            candidate_pairs.add((i, (j - 1) % n2))
            candidate_pairs.add(((i - 1) % n1, (j - 1) % n2))

    if not candidate_pairs:
        return None

    ranked = []
    for i, j in candidate_pairs:
        cost, reverse2 = _edge_pair_cost(points1, n1, points2, n2, i, j)
        ranked.append((cost, i, j, reverse2))
    ranked.sort(key=lambda item: item[0])

    best = None
    validation_limit = 60 if bridge_validator is not None else len(ranked)
    for cost, i, j, reverse2 in ranked[:validation_limit]:
        if _valid_edge_pair(points1, n1, points2, n2, i, j, reverse2, bridge_validator):
            best = (cost, i, j, reverse2)
            break

    if best is None:
        return None

    _, center_i, center_j, _ = best
    for di in range(-12, 13):
        i = (center_i + di) % n1
        for dj in range(-6, 7):
            j = (center_j + dj) % n2
            cost, reverse2 = _edge_pair_cost(points1, n1, points2, n2, i, j)
            if not _valid_edge_pair(points1, n1, points2, n2, i, j, reverse2, bridge_validator):
                continue
            if cost < best[0]:
                best = (cost, i, j, reverse2)
    return best

def _find_best_splice(path1, path2, bridge_validator=None):
    if path1.count() < 2:
        return path2.clone(), 0.0
    if path2.count() < 2:
        return path1.clone(), 0.0

    n1 = _closed_count(path1)
    n2 = _closed_count(path2)
    best = (float("inf"), 0, 0, False)
    fast_limit = 2000 if bridge_validator is not None else 30000

    if n1 * n2 > fast_limit:
        best = _find_best_splice_fast(path1, path2, n1, n2, bridge_validator)
        if best is None:
            return None, float("inf")
    elif best[0] == float("inf"):
        points1 = path1.points
        points2 = path2.points
        for i in range(n1):
            for j in range(n2):
                cost, reverse2 = _edge_pair_cost(points1, n1, points2, n2, i, j)
                if not _valid_edge_pair(points1, n1, points2, n2, i, j, reverse2, bridge_validator):
                    continue
                if cost < best[0]:
                    best = (cost, i, j, reverse2)

    if best[0] == float("inf"):
        return None, float("inf")

    _, i1, i2, reverse2 = best
    merged = _merge_closed_paths_at_edges(path1, path2, i1, i2, reverse2)
    return merged, best[0]

def _merge_closed_paths_at_edges(path1, path2, edge1_index, edge2_index, reverse2):
    n1 = path1.count() - 1 if path1.isClosed() else path1.count()
    n2 = path2.count() - 1 if path2.isClosed() else path2.count()
    merged = Polyline()

    a_start = (edge1_index + 1) % n1
    a_end = edge1_index % n1
    _append_points(merged, _open_contour_path(path1, a_start, a_end, False))

    if reverse2:
        b_start = edge2_index % n2
        b_end = (edge2_index + 1) % n2
        _append_points(merged, _open_contour_path(path2, b_start, b_end, True))
    else:
        b_start = (edge2_index + 1) % n2
        b_end = edge2_index % n2
        _append_points(merged, _open_contour_path(path2, b_start, b_end, False))

    if merged.count() > 0 and not merged.isClosed():
        merged.addPoint(merged.startPoint().clone())
    return merged

def _splice_closed_paths(path1, path2, bridge_validator=None):
    merged, _ = _find_best_splice(path1, path2, bridge_validator)
    if merged is None:
        return None
    if merged.count() > 0 and not merged.isClosed():
        merged.addPoint(merged.startPoint().clone())
    return merged

def _polyline_from_points(points, closed=True):
    poly = Polyline()
    for pt in points:
        poly.addPoint(pt.clone())
    if closed and poly.count() > 0 and not poly.isClosed():
        poly.addPoint(poly.startPoint().clone())
    return poly

def _contour_distance(poly, start_index, end_index, reverse=False):
    n = poly.count() - 1 if poly.isClosed() else poly.count()
    if n <= 0:
        return 0.0
    distance = 0.0
    idx = start_index
    while idx != end_index:
        next_idx = (idx - 1) % n if reverse else (idx + 1) % n
        distance += poly.point(idx).distance(poly.point(next_idx))
        idx = next_idx
    return distance

def _contour_bridge(poly, start_pt, end_pt):
    n = poly.count() - 1 if poly.isClosed() else poly.count()
    if n <= 0:
        return []

    start_index, _ = _nearest_index(poly, start_pt)
    end_index, _ = _nearest_index(poly, end_pt)
    forward = _contour_distance(poly, start_index, end_index, False)
    backward = _contour_distance(poly, start_index, end_index, True)
    reverse = backward < forward

    bridge = []
    idx = start_index
    while True:
        bridge.append(poly.point(idx).clone())
        if idx == end_index:
            break
        idx = (idx - 1) % n if reverse else (idx + 1) % n
    return bridge

def _bbox_from_segment(seg):
    return (
        min(seg.A.x, seg.B.x),
        min(seg.A.y, seg.B.y),
        max(seg.A.x, seg.B.x),
        max(seg.A.y, seg.B.y),
    )

def _sort_paths_for_display(paths):
    return sorted(
        paths,
        key=lambda poly: (
            getattr(poly, "depth", 0),
            _bbox(poly)[0],
            _bbox(poly)[1],
            abs(poly.getArea()),
        ),
    )

def _polyline_reversed(poly):
    reversed_poly = poly.clone()
    reversed_poly.reverse()
    return reversed_poly

def _model_bbox(polygons):
    boxes = [_bbox(poly) for poly in polygons if poly.count() > 0]
    if not boxes:
        return 0.0, 0.0, 0.0, 0.0
    return (
        min(box[0] for box in boxes),
        min(box[1] for box in boxes),
        max(box[2] for box in boxes),
        max(box[3] for box in boxes),
    )

def estimateWaamParameters(polygons, overlap_rate=0.5):
    bbox = _model_bbox(polygons)
    width = bbox[2] - bbox[0]
    height = bbox[3] - bbox[1]
    min_size = max(1e-6, min(width, height))

    # Full-scale WAAM often uses a 1.5-2.5mm centerline spacing.  This model is
    # much smaller, so scale spacing by part size and keep it in a practical
    # preview range.
    centerline_spacing = min(max(min_size / 60.0, 0.35), 2.5)
    bead_width = centerline_spacing / max(1e-6, 1.0 - overlap_rate)
    shell_thickness = bead_width
    return bead_width, overlap_rate, shell_thickness


class PathGroup:
    def __init__(self, source_poly, outer_contour, intermediate_contour):
        self.source_poly = source_poly
        self.outer_contour = outer_contour
        self.intermediate_contour = intermediate_contour
        self.intermediate_points = []
        self.clipped_lines = []
        self.composite_zigzag = Polyline()
        self.local_continuous_path = Polyline()
        self.bbox = _bbox(intermediate_contour)


class ContinuousPathPlanner:
    def __init__(
        self,
        polygons,
        bead_width,
        overlap_rate,
        outer_offset=0.0,
        interpolation_step=None,
        max_global_link_distance=None,
        global_candidate_count=1,
        safe_mode=True,
        shell_thickness=None,
        global_link=True,
    ):
        self.raw_polygons = polygons
        if bead_width is None:
            bead_width, overlap_rate, estimated_shell = estimateWaamParameters(
                polygons, overlap_rate
            )
            if shell_thickness is None:
                shell_thickness = estimated_shell
        self.bead_width = bead_width
        self.overlap_rate = overlap_rate
        self.outer_offset = outer_offset
        self.interpolation_step = (
            bead_width * 0.5 if interpolation_step is None else interpolation_step
        )
        self.max_global_link_distance = (
            bead_width * 1.5
            if max_global_link_distance is None
            else max_global_link_distance
        )
        self.global_candidate_count = max(1, int(global_candidate_count))
        self.safe_mode = safe_mode
        default_shell = bead_width
        self.shell_thickness = default_shell if shell_thickness is None else max(0.0, shell_thickness)
        self.global_link = global_link
        self.ca = ClipperAdaptor()
        self.polygons = []
        self.polygon_bboxes = []
        self.groups = []
        self.all_clipped_lines = []
        self.global_paths = []

    def execute(self):
        if self.safe_mode:
            return self.execute_safe()
        self.preprocess()
        self.generate_path_elements()
        self.combine_path_elements()
        self.local_connection()
        self.global_connection()
        return self.global_paths

    def execute_safe(self):
        self.preprocess()
        paths = []

        path_interval = max(
            self.bead_width * (1.0 - self.overlap_rate),
            self.bead_width * 0.25,
        )
        shell_thickness = self.shell_thickness

        delta = path_interval * 0.5
        while abs(delta) <= shell_thickness + 1e-9:
            offset_polys = self._offset_region(-delta)
            if not offset_polys:
                break
            for poly in _sort_paths_for_display(offset_polys):
                if poly.count() > 1:
                    paths.append(_ensure_closed(poly))
            delta += path_interval

        residual_delta = shell_thickness + path_interval * 0.5
        residual_polys = self._offset_region(-residual_delta)
        fill_paths = self._generate_partitioned_fill(residual_polys, path_interval)
        fill_paths = self._smooth_fill_paths(fill_paths, residual_polys, path_interval)
        for path in fill_paths:
            if path.count() > 1:
                paths.append(path)
        if self.global_link and len(paths) > 1:
            paths = [self._link_paths_one_stroke(paths, path_interval)]
        self.global_paths = paths
        return self.global_paths

    def preprocess(self):
        self.polygons = _normalize_polygon_directions(self.raw_polygons)
        self.polygon_bboxes = [_bbox(poly) for poly in self.polygons]

    def _point_material_state(self, pt):
        inside_count = 0
        for poly, box in zip(self.polygons, self.polygon_bboxes):
            if not _point_in_bbox(pt, box, 1e-6):
                continue
            state = pointInPolygon(pt, poly)
            if state == -1:
                return True
            if state == 1:
                inside_count += 1
        return inside_count % 2 == 1

    def _segment_in_material(self, p1, p2):
        length = p1.distance(p2)
        if length <= 1e-7:
            return True
        sample_step = max(self.bead_width * 0.5, 0.75)
        sample_count = min(8, max(1, int(math.ceil(length / sample_step))))
        for i in range(1, sample_count):
            t = float(i) / sample_count
            pt = Point3D(
                p1.x + (p2.x - p1.x) * t,
                p1.y + (p2.y - p1.y) * t,
                p1.z + (p2.z - p1.z) * t,
            )
            if not self._point_material_state(pt):
                return False
        return True

    def generate_path_elements(self):
        if not self.polygons:
            return

        for poly in self.polygons:
            outer = self._offset_single(poly, self.outer_offset)
            intermediate_delta = self.bead_width * (1.0 - self.overlap_rate)
            if getattr(poly, "depth", 0) % 2 == 0:
                intermediate_delta = -intermediate_delta
            intermediate = self._offset_single(poly, intermediate_delta)
            if outer is None or intermediate is None or intermediate.count() < 3:
                continue
            if getattr(poly, "depth", 0) % 2 == 0:
                outer.makeCCW()
                intermediate.makeCW()
            else:
                outer.makeCW()
                intermediate.makeCW()
            self.groups.append(PathGroup(poly, outer, intermediate))

        self.all_clipped_lines = self._generate_and_clip_parallel_lines()

    def _offset_region(self, delta):
        try:
            result = self.ca.offset(self.polygons, delta)
        except Exception:
            return []
        cleaned = []
        for poly in result:
            fixed = _clean_closed_polygon(_ensure_closed(poly))
            if fixed.count() >= 4 and abs(fixed.getArea()) > 1e-9:
                cleaned.append(fixed)
        return _normalize_polygon_directions(cleaned)

    def _generate_partitioned_fill(self, polygons, interval):
        if not polygons:
            return []
        bbox = _model_bbox(polygons)
        if bbox is None:
            return []

        y_min, y_max = bbox[1], bbox[3]
        ys = []
        y = y_min + interval * 0.5
        while y < y_max - interval * 0.25:
            ys.append(y)
            y += interval
        if not ys:
            return []

        try:
            ipses = calHatchPoints(polygons, ys)
        except Exception:
            return []

        rows = []
        for ips in ipses:
            if len(ips) < 2:
                continue
            ips.sort(key=lambda pt: pt.x)
            segments = []
            for i in range(0, len(ips) - 1, 2):
                left = ips[i].clone()
                right = ips[i + 1].clone()
                if left.distance(right) >= interval * 0.25:
                    segments.append((left, right))
            if segments:
                rows.append((segments[0][0].y, segments))

        if not rows:
            return []

        rows.sort(key=lambda item: item[0])
        poly_bboxes = [_bbox(poly) for poly in polygons]
        finished = []
        active = []
        max_row_gap = 3
        side_dx_limit = max(interval * 10.0, self.bead_width * 5.0)
        side_link_limit = max(interval * 14.0, self.bead_width * 6.0)

        for row_index, (_, segments) in enumerate(rows):
            next_active = []
            assigned_paths = set()
            assigned_segments = set()

            for seg_index, (left, right) in enumerate(segments):
                best = None
                for active_index, item in enumerate(active):
                    if active_index in assigned_paths:
                        continue
                    row_gap = row_index - item["row_index"]
                    if row_gap < 1 or row_gap > max_row_gap:
                        continue
                    side_options = [(item["side"], False)]
                    if item["path"].count() == 2:
                        opposite_side = "left" if item["side"] == "right" else "right"
                        side_options.append((opposite_side, True))

                    for side, reverse_active in side_options:
                        if side == "right":
                            entry, exit_pt, next_side = right, left, "left"
                        else:
                            entry, exit_pt, next_side = left, right, "right"
                        end_pt = (
                            item["path"].startPoint()
                            if reverse_active
                            else item["path"].endPoint()
                        )
                        dx = abs(end_pt.x - entry.x)
                        dist = end_pt.distance(entry)
                        if dx > side_dx_limit or dist > side_link_limit:
                            continue
                        if best is not None and dist >= best[0]:
                            continue
                        transition = self._same_side_transition(
                            end_pt, entry, side, polygons, poly_bboxes, interval
                        )
                        if transition:
                            transition_len = self._points_path_length(
                                [end_pt] + transition
                            )
                            best = (
                                transition_len,
                                active_index,
                                transition,
                                exit_pt,
                                next_side,
                                reverse_active,
                            )

                if best is None:
                    continue

                _, active_index, transition, exit_pt, next_side, reverse_active = best
                path = active[active_index]["path"]
                if reverse_active:
                    path.reverse()
                for pt in transition:
                    if not path.endPoint().isCoincide(pt):
                        path.addPoint(pt.clone())
                path.addPoint(exit_pt.clone())
                next_active.append(
                    {"path": path, "row_index": row_index, "side": next_side}
                )
                assigned_paths.add(active_index)
                assigned_segments.add(seg_index)

            for active_index, item in enumerate(active):
                if active_index in assigned_paths:
                    continue
                if row_index - item["row_index"] < max_row_gap:
                    next_active.append(item)
                elif item["path"].count() > 1:
                    finished.append(item["path"])

            for seg_index, (left, right) in enumerate(segments):
                if seg_index in assigned_segments:
                    continue
                path = Polyline()
                path.addPoint(left)
                path.addPoint(right)
                next_active.append({"path": path, "row_index": row_index, "side": "right"})

            active = next_active

        for item in active:
            if item["path"].count() > 1:
                finished.append(item["path"])
        return finished

    def _point_in_given_polygons(self, pt, polygons, bboxes):
        inside_count = 0
        for poly, box in zip(polygons, bboxes):
            if not _point_in_bbox(pt, box, 1e-6):
                continue
            state = pointInPolygon(pt, poly)
            if state == -1:
                return True
            if state == 1:
                inside_count += 1
        return inside_count % 2 == 1

    def _segment_in_given_polygons(self, p1, p2, polygons, bboxes, sample_step):
        length = p1.distance(p2)
        if length <= 1e-7:
            return True
        sample_count = min(8, max(2, int(math.ceil(length / max(sample_step, 1e-6)))))
        for i in range(1, sample_count):
            t = float(i) / sample_count
            pt = Point3D(
                p1.x + (p2.x - p1.x) * t,
                p1.y + (p2.y - p1.y) * t,
                p1.z + (p2.z - p1.z) * t,
            )
            if not self._point_in_given_polygons(pt, polygons, bboxes):
                return False
        return True

    def _points_path_length(self, points):
        length = 0.0
        for a, b in zip(points, points[1:]):
            length += a.distance(b)
        return length

    def _transition_is_safe(self, points, polygons, bboxes, sample_step):
        for a, b in zip(points, points[1:]):
            if not self._segment_in_given_polygons(a, b, polygons, bboxes, sample_step):
                return False
        return True

    def _same_side_transition(self, start, entry, side, polygons, bboxes, interval):
        direct = [entry.clone()]
        mid = start.middle(entry)
        if self._point_in_given_polygons(mid, polygons, bboxes):
            return direct

        # The visible zigzag should connect the two same-side hatch endpoints
        # directly.  For endpoints that lie exactly on an offset boundary, test a
        # slightly inset copy to avoid rejecting valid boundary-following links.
        inward = max(interval * 0.35, self.bead_width * 0.15)
        if side == "right":
            dx = -inward
        else:
            dx = inward
        inset_start = Point3D(start.x + dx, start.y, start.z)
        inset_entry = Point3D(entry.x + dx, entry.y, entry.z)
        inset_mid = inset_start.middle(inset_entry)
        if self._point_in_given_polygons(inset_mid, polygons, bboxes):
            return direct
        return []

    def _curve_points_in_given_polygons(self, points, polygons, bboxes):
        if not points:
            return True
        return self._point_in_given_polygons(points[len(points) // 2], polygons, bboxes)

    def _smooth_fill_paths(self, paths, polygons, interval):
        if not paths or not polygons:
            return paths
        bboxes = [_bbox(poly) for poly in polygons]
        radius = max(interval * 0.45, self.bead_width * 0.2)
        smoothed = []
        for path in paths:
            if path is None or path.count() < 3 or path.isClosed():
                smoothed.append(path)
                continue
            smoothed.append(
                self._smooth_open_path_corners(path, polygons, bboxes, radius, interval)
            )
        return smoothed

    def _smooth_open_path_corners(self, path, polygons, bboxes, radius, interval):
        result = Polyline()
        result.addPoint(path.startPoint().clone())
        min_turn = math.radians(35.0)
        min_cut = max(interval * 0.08, 1e-6)

        for i in range(1, path.count() - 1):
            prev_pt = path.point(i - 1)
            corner = path.point(i)
            next_pt = path.point(i + 1)

            if (
                getattr(prev_pt, "w", 0) == 2
                or getattr(corner, "w", 0) == 2
                or getattr(next_pt, "w", 0) == 2
            ):
                result.addPoint(corner.clone())
                continue

            in_vec = prev_pt.pointTo(corner)
            out_vec = corner.pointTo(next_pt)
            in_len = in_vec.length()
            out_len = out_vec.length()
            if in_len <= 1e-7 or out_len <= 1e-7:
                result.addPoint(corner.clone())
                continue

            turn = in_vec.getAngle(out_vec)
            if turn < min_turn:
                result.addPoint(corner.clone())
                continue

            cut = min(radius, in_len * 0.35, out_len * 0.35)
            if cut < min_cut:
                result.addPoint(corner.clone())
                continue

            in_unit = in_vec.normalized()
            out_unit = out_vec.normalized()
            entry = corner - in_unit.amplified(cut)
            exit_pt = corner + out_unit.amplified(cut)
            curve = [entry]
            for t in (0.25, 0.5, 0.75):
                mt = 1.0 - t
                curve.append(
                    Point3D(
                        mt * mt * entry.x + 2.0 * mt * t * corner.x + t * t * exit_pt.x,
                        mt * mt * entry.y + 2.0 * mt * t * corner.y + t * t * exit_pt.y,
                        mt * mt * entry.z + 2.0 * mt * t * corner.z + t * t * exit_pt.z,
                    )
                )
            curve.append(exit_pt)

            if not self._curve_points_in_given_polygons(curve, polygons, bboxes):
                result.addPoint(corner.clone())
                continue

            for pt in curve:
                if not result.endPoint().isCoincide(pt):
                    result.addPoint(pt.clone())

        result.addPoint(path.endPoint().clone())
        return result

    def _path_points_from_entry(self, path, entry_index=0, reverse=False):
        if path.isClosed():
            return _path_from_closed_poly(path, entry_index, reverse).points
        points = list(reversed(path.points)) if reverse else path.points
        return [pt.clone() for pt in points]

    def _best_entry_options(self, current_pt, path):
        n = _closed_count(path)
        if n <= 0:
            return []
        if path.isClosed():
            best_i = 0
            best_d = float("inf")
            for i in range(n):
                d = _point_distance_square(current_pt, path.point(i))
                if d < best_d:
                    best_i = i
                    best_d = d
            return [(best_d, best_i, False), (best_d, best_i, True)]
        start_d = _point_distance_square(current_pt, path.startPoint())
        end_d = _point_distance_square(current_pt, path.endPoint())
        return [(start_d, 0, False), (end_d, n - 1, True)]

    def _route_between_points(self, start_pt, end_pt, spacing):
        if self._segment_in_material(start_pt, end_pt):
            return [end_pt.clone()]

        bbox = _model_bbox(self.polygons)
        margin = max(spacing * 2.0, self.bead_width)
        x_min = bbox[0] - margin
        y_min = bbox[1] - margin
        x_max = bbox[2] + margin
        y_max = bbox[3] + margin
        step = max(spacing * 1.5, 0.35)

        def to_node(pt):
            return (
                int(round((pt.x - x_min) / step)),
                int(round((pt.y - y_min) / step)),
            )

        def to_point(node):
            return Point3D(x_min + node[0] * step, y_min + node[1] * step, start_pt.z)

        max_ix = int(math.ceil((x_max - x_min) / step))
        max_iy = int(math.ceil((y_max - y_min) / step))
        start = to_node(start_pt)
        goal = to_node(end_pt)

        def in_bounds(node):
            return 0 <= node[0] <= max_ix and 0 <= node[1] <= max_iy

        def walkable(node):
            return in_bounds(node) and self._point_material_state(to_point(node))

        if not walkable(start) or not walkable(goal):
            return []

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        max_expansions = 5000
        expansions = 0

        while open_heap and expansions < max_expansions:
            _, current = heapq.heappop(open_heap)
            expansions += 1
            if current == goal:
                nodes = [current]
                while nodes[-1] in came_from:
                    nodes.append(came_from[nodes[-1]])
                nodes.reverse()
                route = [to_point(node) for node in nodes[1:]]
                route.append(end_pt.clone())
                return route

            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not walkable(nxt):
                    continue
                tentative = g_score[current] + 1.0
                if tentative >= g_score.get(nxt, float("inf")):
                    continue
                came_from[nxt] = current
                g_score[nxt] = tentative
                heuristic = abs(nxt[0] - goal[0]) + abs(nxt[1] - goal[1])
                heapq.heappush(open_heap, (tentative + heuristic, nxt))
        return []

    def _link_paths_one_stroke(self, paths, spacing):
        candidates = [path for path in paths if path is not None and path.count() > 1]
        if not candidates:
            return Polyline()
        candidates.sort(
            key=lambda path: (
                0 if path.isClosed() else 1,
                _bbox(path)[0],
                _bbox(path)[1],
                -abs(path.getArea()) if path.isClosed() else 0.0,
            )
        )

        global_path = Polyline()
        first = candidates.pop(0)
        _append_points(global_path, self._path_points_from_entry(first, 0, False))
        last_closed_path = first if first.isClosed() else None

        while candidates:
            current_pt = global_path.endPoint()
            ranked = []
            for idx, path in enumerate(candidates):
                for dist, entry_index, reverse in self._best_entry_options(current_pt, path):
                    ranked.append((dist, idx, entry_index, reverse))
            ranked.sort(key=lambda item: item[0])

            _, idx, entry_index, reverse = ranked[0]
            path = candidates[idx]
            chosen = (idx, entry_index, reverse)
            entry_pt = path.point(entry_index)

            if last_closed_path is not None:
                exit_index, _ = _nearest_index(last_closed_path, entry_pt)
                exit_pt = last_closed_path.point(exit_index)
                bridge = _contour_bridge(last_closed_path, current_pt, exit_pt)
                _append_points(global_path, bridge)
                current_pt = global_path.endPoint()

            travel_pt = entry_pt.clone()
            travel_pt.w = 2
            chosen_route = [travel_pt]

            idx, entry_index, reverse = chosen
            next_path = candidates.pop(idx)
            _append_points(global_path, chosen_route)
            _append_points(
                global_path,
                self._path_points_from_entry(next_path, entry_index, reverse),
                skip_duplicate=False,
            )
            last_closed_path = next_path if next_path.isClosed() else None
        return global_path

    def _offset_single(self, poly, delta):
        try:
            result = self.ca.offset([poly], delta)
        except Exception:
            result = []
        if not result:
            return poly.clone() if abs(delta) <= 1e-6 else None
        result.sort(key=lambda item: abs(item.getArea()), reverse=True)
        return _ensure_closed(result[0])

    def _generate_and_clip_parallel_lines(self):
        if not self.groups:
            return []

        y_min = min(group.bbox[1] for group in self.groups)
        y_max = max(group.bbox[3] for group in self.groups)
        ys = []
        y = y_min + self.bead_width
        while y < y_max:
            ys.append(y)
            y += self.bead_width

        ipses = calHatchPoints([group.intermediate_contour for group in self.groups], ys)
        shrink_dist = max(0.0, self.bead_width * self.overlap_rate * 0.2)
        x_axis = Vector3D(1, 0, 0)
        clipped = []

        for ips in ipses:
            for i in range(0, len(ips) - 1, 2):
                p1 = ips[i] + x_axis.amplified(shrink_dist)
                p2 = ips[i + 1] - x_axis.amplified(shrink_dist)
                if p1.distanceSquare(p2) > 1e-12:
                    if p1.x > p2.x:
                        p1, p2 = p2, p1
                    clipped.append(Segment(p1, p2))
        return clipped

    def combine_path_elements(self):
        for group in self.groups:
            group.intermediate_points = self._interpolate_contour(
                group.intermediate_contour, self.interpolation_step
            )
            if getattr(group.source_poly, "depth", 0) % 2 == 1:
                continue
            for line in self.all_clipped_lines:
                mid = line.A.middle(line.B)
                if not _bbox_overlap(group.bbox, _bbox_from_segment(line), self.bead_width):
                    continue
                if self._point_material_state(mid) and pointInPolygon(mid, group.intermediate_contour) == 1:
                    group.clipped_lines.append(line)

    def _interpolate_contour(self, poly, step):
        points = []
        step = max(step, 1e-6)
        for i in range(poly.count() - 1):
            p1 = poly.point(i)
            p2 = poly.point(i + 1)
            dist = p1.distance(p2)
            count = max(1, int(math.ceil(dist / step)))
            for j in range(count):
                t = float(j) / count
                points.append(
                    Point3D(
                        p1.x + (p2.x - p1.x) * t,
                        p1.y + (p2.y - p1.y) * t,
                        p1.z + (p2.z - p1.z) * t,
                    )
                )
        if points:
            points.append(points[0].clone())
        return points

    def local_connection(self):
        for group in self.groups:
            group.composite_zigzag = self._build_composite_zigzag(group)
            group.local_continuous_path = self._merge_outer_and_zigzag(group)

    def _build_composite_zigzag(self, group):
        path = Polyline()
        spans = self._sorted_group_spans(group)
        dense_contour = _polyline_from_points(group.intermediate_points, True)

        if not spans:
            return dense_contour

        first_in = None
        prev_out = None
        for i, span in enumerate(spans):
            left, right = self._left_right_points(span)
            if i % 2 == 0:
                p_in, p_out = left, right
            else:
                p_in, p_out = right, left

            if first_in is None:
                first_in = p_in.clone()
                path.addPoint(first_in.clone())
            else:
                bridge = _contour_bridge(dense_contour, prev_out, p_in)
                _append_points(path, bridge)
                if not path.endPoint().isCoincide(p_in):
                    path.addPoint(p_in.clone())
            if not path.endPoint().isCoincide(p_out):
                path.addPoint(p_out.clone())
            prev_out = p_out.clone()
        if first_in is not None and prev_out is not None:
            bridge = _contour_bridge(dense_contour, prev_out, first_in)
            _append_points(path, bridge)
            if not path.endPoint().isCoincide(first_in):
                path.addPoint(first_in.clone())
        return path

    def _sorted_group_spans(self, group):
        seen = set()
        spans = []
        for span in group.clipped_lines:
            key = (
                round(min(span.A.x, span.B.x), 7),
                round(max(span.A.x, span.B.x), 7),
                round((span.A.y + span.B.y) * 0.5, 7),
            )
            if key in seen:
                continue
            seen.add(key)
            spans.append(span)
        spans.sort(key=lambda seg: ((seg.A.y + seg.B.y) * 0.5, min(seg.A.x, seg.B.x)))
        return spans

    def _left_right_points(self, span):
        if span.A.x <= span.B.x:
            return span.A, span.B
        return span.B, span.A

    def _merge_outer_and_zigzag(self, group):
        outer = _ensure_closed(group.outer_contour)
        zigzag = _ensure_closed(group.composite_zigzag)
        if zigzag.count() < 2:
            return outer
        merged = _splice_closed_paths(outer, zigzag)
        return merged if merged is not None else outer

    def global_connection(self):
        candidates = [g for g in self.groups if g.local_continuous_path.count() >= 2]
        if not candidates:
            return

        even_groups = [
            group
            for group in candidates
            if getattr(group.source_poly, "depth", 0) % 2 == 0
        ]
        even_groups.sort(
            key=lambda group: (
                getattr(group.source_poly, "depth", 0),
                group.bbox[0],
                group.bbox[1],
            )
        )
        odd_groups = [
            group
            for group in candidates
            if getattr(group.source_poly, "depth", 0) % 2 == 1
        ]
        odd_groups.sort(
            key=lambda group: (
                getattr(group.source_poly, "depth", 0),
                group.bbox[0],
                group.bbox[1],
            )
        )

        for group in even_groups:
            self.global_paths.append(group.local_continuous_path)
        for group in odd_groups:
            self.global_paths.append(group.local_continuous_path)

    def _nearest_adjacent_pair_distance(self, path1, path2):
        best = float("inf")
        n1 = path1.count() - 1 if path1.isClosed() else path1.count()
        n2 = path2.count() - 1 if path2.isClosed() else path2.count()
        for i in range(n1):
            p1 = path1.point(i)
            for j in range(n2):
                p2 = path2.point(j)
                d = p1.distanceSquare(p2)
                if d < best:
                    best = d
        return best


def continuousPathPlanner(
    polygons,
    bead_width=None,
    overlap_rate=0.5,
    outer_offset=0.0,
    interpolation_step=None,
    max_global_link_distance=None,
    global_candidate_count=1,
    safe_mode=True,
    shell_thickness=None,
    global_link=True,
):
    if len(polygons) == 0:
        return []
    if bead_width is not None and bead_width <= 0:
        raise ValueError("bead_width must be positive")
    if overlap_rate < 0 or overlap_rate >= 1:
        raise ValueError("overlap_rate must satisfy 0 <= overlap_rate < 1")

    target_z = polygons[0].points[0].z if polygons[0].count() > 0 else 0.0
    planner = ContinuousPathPlanner(
        polygons,
        bead_width,
        overlap_rate,
        outer_offset=outer_offset,
        interpolation_step=interpolation_step,
        max_global_link_distance=max_global_link_distance,
        global_candidate_count=global_candidate_count,
        safe_mode=safe_mode,
        shell_thickness=shell_thickness,
        global_link=global_link,
    )
    paths = planner.execute()

    fixed_paths = []
    for path in paths:
        fixed = Polyline()
        for pt in path.points:
            fixed.addPoint(Point3D(pt.x, pt.y, target_z, pt.w))
        if fixed.count() > 1:
            fixed_paths.append(fixed)
    return fixed_paths

def _plan_layer_worker(args):
    index, contours, bead_width, overlap_rate, shell_thickness = args
    if not contours:
        return index, []
    return (
        index,
        continuousPathPlanner(
            contours,
            bead_width,
            overlap_rate,
            shell_thickness=shell_thickness,
        ),
    )

def continuousLayerPathPlanner(
    layers,
    bead_width=None,
    overlap_rate=0.5,
    max_workers=None,
    parallel=True,
    shell_thickness=None,
):
    jobs = [
        (index, layer.contours, bead_width, overlap_rate, shell_thickness)
        for index, layer in enumerate(layers)
    ]
    if not parallel or len(jobs) <= 1:
        return [_plan_layer_worker(job)[1] for job in jobs]

    if max_workers is None:
        cpu_count = os.cpu_count() or 1
        max_workers = max(1, min(cpu_count - 1, 6, len(jobs)))
    if max_workers <= 1:
        return [_plan_layer_worker(job)[1] for job in jobs]

    results = [None] * len(jobs)
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(_plan_layer_worker, job) for job in jobs]
        for future in as_completed(futures):
            index, paths = future.result()
            results[index] = paths
    return results
