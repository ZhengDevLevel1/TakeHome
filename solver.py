#!/usr/bin/env python3
"""
2D Rectangle Packing within Polygon Boundary
Solves the furniture placement problem:
- Place rectangular items inside a polygon boundary
- Items must be axis-aligned (0° or 90° rotation)
- Items must not overlap with each other, boundary edges, or door zones
- Fridge door-opening side must be clear
- Prefer wall-snapping placements
"""

import json
import math
import sys
import os
from typing import List, Tuple, Dict, Optional

Point = Tuple[float, float]
Rect = Tuple[float, float, float, float]  # (cx, cy, width, height)


def read_input(file_path: str) -> dict:
    with open(file_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def distance(p1: Point, p2: Point) -> float:
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def point_in_polygon(point: Point, polygon: List[Point]) -> bool:
    """Ray casting algorithm"""
    x, y = point
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def segments_intersect(p1, p2, p3, p4) -> bool:
    """Check if segment p1-p2 intersects segment p3-p4 (excluding touching at endpoints)"""
    def orient(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if abs(val) < 1e-9:
            return 0
        return 1 if val > 0 else 2

    def on_segment(p, q, r):
        return (min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9 and
                min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9)

    o1 = orient(p1, p2, p3)
    o2 = orient(p1, p2, p4)
    o3 = orient(p3, p4, p1)
    o4 = orient(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and on_segment(p1, p3, p2):
        return True
    if o2 == 0 and on_segment(p1, p4, p2):
        return True
    if o3 == 0 and on_segment(p3, p1, p4):
        return True
    if o4 == 0 and on_segment(p3, p2, p4):
        return True

    return False


def rect_corners(cx: float, cy: float, w: float, h: float) -> List[Point]:
    hw, hh = w / 2, h / 2
    return [
        (cx - hw, cy - hh),
        (cx + hw, cy - hh),
        (cx + hw, cy + hh),
        (cx - hw, cy + hh)
    ]


def rect_edges(cx: float, cy: float, w: float, h: float):
    corners = rect_corners(cx, cy, w, h)
    edges = []
    for i in range(4):
        edges.append((corners[i], corners[(i + 1) % 4]))
    return edges


def rect_inside_polygon(cx: float, cy: float, w: float, h: float, polygon: List[Point]) -> bool:
    """Check if axis-aligned rectangle is fully inside polygon"""
    corners = rect_corners(cx, cy, w, h)
    for corner in corners:
        if not point_in_polygon(corner, polygon):
            return False

    n = len(polygon)
    for c1, c2 in rect_edges(cx, cy, w, h):
        for j in range(n):
            p1 = polygon[j]
            p2 = polygon[(j + 1) % n]
            if segments_intersect(c1, c2, p1, p2):
                return False
    return True


def rects_overlap(cx1, cy1, w1, h1, cx2, cy2, w2, h2) -> bool:
    return (abs(cx1 - cx2) * 2 < w1 + w2 - 1e-6 and
            abs(cy1 - cy2) * 2 < h1 + h2 - 1e-6)


def get_polygon_edges(polygon: List[Point]):
    n = len(polygon)
    edges = []
    for i in range(n):
        edges.append((polygon[i], polygon[(i + 1) % n]))
    return edges


def edge_direction(p1: Point, p2: Point) -> Tuple[float, float]:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = distance(p1, p2)
    if length < 1e-12:
        return (0, 0)
    return (dx / length, dy / length)


def edge_normal(p1: Point, p2: Point) -> Tuple[float, float]:
    """Get inward-pointing normal of edge (assumes CCW polygon)"""
    dx, dy = edge_direction(p1, p2)
    return (-dy, dx)


def compute_inward_normal(polygon: List[Point]) -> Tuple[float, float]:
    """Compute the average inward normal direction for a polygon"""
    centroid = polygon_centroid(polygon)
    edges = get_polygon_edges(polygon)
    in_x, in_y = 0, 0
    for p1, p2 in edges:
        mid_x = (p1[0] + p2[0]) / 2
        mid_y = (p1[1] + p2[1]) / 2
        to_center_x = centroid[0] - mid_x
        to_center_y = centroid[1] - mid_y
        length = math.sqrt(to_center_x**2 + to_center_y**2)
        if length > 1e-12:
            in_x += to_center_x / length
            in_y += to_center_y / length
    length = math.sqrt(in_x**2 + in_y**2)
    if length > 1e-12:
        return (in_x / length, in_y / length)
    return (0, -1)


def polygon_centroid(points: List[Point]) -> Point:
    n = len(points)
    area = 0
    cx, cy = 0, 0
    for i in range(n):
        j = (i + 1) % n
        cross = points[i][0] * points[j][1] - points[j][0] * points[i][1]
        area += cross
        cx += (points[i][0] + points[j][0]) * cross
        cy += (points[i][1] + points[j][1]) * cross
    area /= 2
    if abs(area) < 1e-12:
        return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)
    cx /= (6 * area)
    cy /= (6 * area)
    return (cx, cy)


def compute_door_zone(door: List[Point], polygon: List[Point], is_inward: bool) -> List[Point]:
    """Compute the door obstruction zone (rectangle around door for inward-opening)"""
    if not is_inward:
        return []

    p1, p2 = door[0], door[1]
    door_width = distance(p1, p2)

    inward = compute_inward_normal(polygon)

    corners = [
        (p1[0], p1[1]),
        (p2[0], p2[1]),
        (p2[0] + inward[0] * door_width, p2[1] + inward[1] * door_width),
        (p1[0] + inward[0] * door_width, p1[1] + inward[1] * door_width)
    ]
    return corners


def point_in_rect(px, py, cx, cy, w, h) -> bool:
    return abs(px - cx) * 2 <= w + 1e-6 and abs(py - cy) * 2 <= h + 1e-6


def rect_intersects_polygon_rect(cx, cy, w, h, poly_rect: List[Point]) -> bool:
    """Check if rectangle overlaps with a polygon (simplified as rect-rect or point-in-rect)"""
    if not poly_rect:
        return False

    corners = rect_corners(cx, cy, w, h)
    for corner in corners:
        if point_in_polygon(corner, poly_rect):
            return True

    r_corners = poly_rect
    for rc in r_corners:
        if point_in_rect(rc[0], rc[1], cx, cy, w, h):
            return True

    n = len(poly_rect)
    rect_edges_list = rect_edges(cx, cy, w, h)
    for re1, re2 in rect_edges_list:
        for j in range(n):
            pe1 = poly_rect[j]
            pe2 = poly_rect[(j + 1) % n]
            if segments_intersect(re1, re2, pe1, pe2):
                return True

    return False


def wall_snap_positions(polygon: List[Point], w: float, h: float, step: float = 50,
                        door_zone: List[Point] = None) -> List[Tuple[Point, float]]:
    """Generate candidate positions snapped to walls with both orientations.
    Returns list of ((cx, cy), rotation_degrees)"""
    positions = []
    edges = get_polygon_edges(polygon)
    offset = min(w, h) / 2 + 10  # Small offset from wall

    for p1, p2 in edges:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        edge_len = distance(p1, p2)
        if edge_len < 1e-6:
            continue

        nx, ny = edge_normal(p1, p2)

        num_steps = max(1, int(edge_len / step))
        for s in range(num_steps + 1):
            t = s / num_steps
            base_x = p1[0] + dx * t
            base_y = p1[1] + dy * t

            for rot, rw, rh in [(0, w, h), (90, h, w)]:
                cx = base_x + nx * (rw / 2 + 5)
                cy = base_y + ny * (rh / 2 + 5)

                if door_zone and rect_intersects_polygon_rect(cx, cy, rw, rh, door_zone):
                    continue

                positions.append(((cx, cy), rot))

    return positions


def random_interior_positions(polygon: List[Point], w: float, h: float, count: int = 100,
                               door_zone: List[Point] = None) -> List[Tuple[Point, float]]:
    """Generate random positions inside polygon"""
    positions = []
    all_x = [p[0] for p in polygon]
    all_y = [p[1] for p in polygon]
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    import random
    random.seed(42)

    attempts = 0
    while len(positions) < count and attempts < count * 20:
        attempts += 1
        cx = random.uniform(min_x + w / 2, max_x - w / 2)
        cy = random.uniform(min_y + h / 2, max_y - h / 2)

        for rot, rw, rh in [(0, w, h), (90, h, w)]:
            if rect_inside_polygon(cx, cy, rw, rh, polygon):
                if not (door_zone and rect_intersects_polygon_rect(cx, cy, rw, rh, door_zone)):
                    positions.append(((cx, cy), rot))

    return positions


def item_type_key(name: str) -> str:
    """Extract item type from name like 'shelf-1' -> 'shelf'"""
    parts = name.split('-')
    return parts[0]


def compute_door_clearance(door: List[Point]) -> float:
    """Compute the door opening clearance needed for fridge.
    The fridge door-opening side is one of its length sides.
    We need to ensure nothing is placed on the opening side."""
    return 0


class FurniturePlacer:
    def __init__(self, boundary, door, is_inward, items):
        self.boundary = boundary
        self.door = door
        self.is_inward = is_inward
        self.items = items
        self.placed = []

        self.door_zone = compute_door_zone(door, boundary, is_inward)

        self.polygon_edges = get_polygon_edges(boundary)

        self.centroid = polygon_centroid(boundary)

        all_x = [p[0] for p in boundary]
        all_y = [p[1] for p in boundary]
        self.bounds = (min(all_x), min(all_y), max(all_x), max(all_y))

    def is_valid_placement(self, cx, cy, w, h, exclude_idx=-1) -> bool:
        if not rect_inside_polygon(cx, cy, w, h, self.boundary):
            return False

        if self.door_zone and rect_intersects_polygon_rect(cx, cy, w, h, self.door_zone):
            return False

        for i, (px, py, pw, ph, _, _) in enumerate(self.placed):
            if i == exclude_idx:
                continue
            if rects_overlap(cx, cy, w, h, px, py, pw, ph):
                return False

        return True

    def count_wall_contacts(self, cx, cy, w, h) -> int:
        """Count how many sides of the rectangle touch a boundary wall"""
        contacts = 0
        corners = rect_corners(cx, cy, w, h)

        for c1, c2 in rect_edges(cx, cy, w, h):
            for p1, p2 in self.polygon_edges:
                if distance_to_segment_midpoint(c1, c2, p1, p2) < 20:
                    contacts += 1
                    break

        return contacts

    def count_item_contacts(self, cx, cy, w, h, exclude_idx=-1) -> int:
        """Count how many sides touch other placed items"""
        contacts = 0
        margin = 5

        for i, (px, py, pw, ph, _, _) in enumerate(self.placed):
            if i == exclude_idx:
                continue
            if (abs(cx - px) * 2 < w + pw + margin * 2 and
                abs(cy - py) * 2 < h + ph + margin * 2):
                contacts += 1

        return contacts

    def generate_candidates(self, w, h) -> List[Tuple[Point, float]]:
        candidates = []

        wall_positions = wall_snap_positions(self.boundary, w, h, step=30,
                                              door_zone=self.door_zone)
        candidates.extend(wall_positions)

        random_positions = random_interior_positions(self.boundary, w, h, count=200,
                                                      door_zone=self.door_zone)
        candidates.extend(random_positions)

        return candidates

    def score_placement(self, cx, cy, w, h, item_name) -> float:
        score = 0

        wall_contacts = self.count_wall_contacts(cx, cy, w, h)
        score += wall_contacts * 100

        item_contacts = self.count_item_contacts(cx, cy, w, h)
        score += item_contacts * 50

        dist_to_centroid = distance((cx, cy), self.centroid)
        score -= dist_to_centroid * 0.01

        item_type = item_type_key(item_name)
        if item_type == 'fridge':
            score += 1000

        return score

    def compute_fridge_door_zone(self, cx, cy, w, h, rotation) -> List[Tuple[float, float]]:
        """For fridge, the door-opening side (length side) must be clear.
        Returns the area that must be kept clear."""
        clearance = 300

        if rotation == 0:
            return [
                (cx - w / 2, cy - h / 2 - clearance),
                (cx + w / 2, cy - h / 2),
            ]
        else:
            return [
                (cx - w / 2 - clearance, cy - h / 2),
                (cx - w / 2, cy + h / 2),
            ]

    def check_fridge_clearance(self, cx, cy, w, h, rotation, exclude_idx=-1) -> bool:
        clearance = 300

        if rotation == 0:
            zone_y_min = cy - h / 2 - clearance
            zone_y_max = cy - h / 2
            zone_x_min = cx - w / 2
            zone_x_max = cx + w / 2
        else:
            zone_y_min = cy - h / 2
            zone_y_max = cy + h / 2
            zone_x_min = cx - w / 2 - clearance
            zone_x_max = cx - w / 2

        for i, (px, py, pw, ph, _, _) in enumerate(self.placed):
            if i == exclude_idx:
                continue
            if (px + pw / 2 > zone_x_min and px - pw / 2 < zone_x_max and
                py + ph / 2 > zone_y_min and py - ph / 2 < zone_y_max):
                return False

        return True

    def place_items(self) -> bool:
        sorted_items = sorted(self.items.items(),
                              key=lambda x: x[1][0] * x[1][1],
                              reverse=True)

        for name, (length, width) in sorted_items:
            item_type = item_type_key(name)
            best_pos = None
            best_score = -float('inf')

            candidates = self.generate_candidates(length, width)

            for (cx, cy), rot in candidates:
                w, h = (length, width) if rot == 0 else (width, length)

                if not self.is_valid_placement(cx, cy, w, h):
                    continue

                if item_type == 'fridge':
                    if not self.check_fridge_clearance(cx, cy, w, h, rot):
                        continue

                score = self.score_placement(cx, cy, w, h, name)

                if score > best_score:
                    best_score = score
                    best_pos = (cx, cy, w, h, rot, name)

            if best_pos is None:
                print(f"WARNING: Could not place {name} ({length}x{width})")
                return False

            cx, cy, w, h, rot, _ = best_pos
            self.placed.append((cx, cy, w, h, rot, name))
            print(f"  Placed {name}: center=({cx:.1f}, {cy:.1f}), size={w:.0f}x{h:.0f}, rotation={rot}°")

        return True


def distance_to_segment_midpoint(c1, c2, p1, p2) -> float:
    """Approximate distance between two segments by checking midpoint distances"""
    m1 = ((c1[0] + c2[0]) / 2, (c1[1] + c2[1]) / 2)
    m2 = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

    d = distance(m1, m2)

    for pt in [c1, c2]:
        d = min(d, point_to_segment_distance(pt, p1, p2))
    for pt in [p1, p2]:
        d = min(d, point_to_segment_distance(pt, c1, c2))

    return d


def point_to_segment_distance(point, seg_start, seg_end) -> float:
    px, py = point
    sx, sy = seg_start
    ex, ey = seg_end
    dx = ex - sx
    dy = ey - sy
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return distance(point, seg_start)
    t = max(0, min(1, ((px - sx) * dx + (py - sy) * dy) / length_sq))
    proj_x = sx + t * dx
    proj_y = sy + t * dy
    return distance(point, (proj_x, proj_y))


def solve(file_path: str):
    data = read_input(file_path)

    boundary = [tuple(p) for p in data['boundary']]
    door = [tuple(p) for p in data['door']]
    is_inward = data.get('isOpenInward', False)
    items = data['algoToPlace']

    print(f"\n{'='*60}")
    print(f"Solving: {os.path.basename(file_path)}")
    print(f"{'='*60}")
    print(f"Boundary: {len(boundary)} vertices")
    print(f"Door: {door}, inward={is_inward}")
    print(f"Items to place: {len(items)}")

    if is_inward:
        dw = distance(door[0], door[1])
        print(f"Door width: {dw:.1f}, inward zone: {dw:.1f}x{dw:.1f}")

    placer = FurniturePlacer(boundary, door, is_inward, items)
    success = placer.place_items()

    print(f"\nResult: {'SUCCESS' if success else 'FAILED'}")

    if success:
        print("\nPlacements:")
        for cx, cy, w, h, rot, name in placer.placed:
            print(f"  {name}: center=({cx:.2f}, {cy:.2f}), size=({w:.0f}, {h:.0f}), rotation={rot}°")

    return success, placer.placed


def format_output(success: bool, placements: List) -> dict:
    output = {
        "feasible": success,
        "placements": []
    }
    for cx, cy, w, h, rot, name in placements:
        output["placements"].append({
            "name": name,
            "center": [round(cx, 2), round(cy, 2)],
            "width": round(w, 2),
            "height": round(h, 2),
            "rotation": rot
        })
    return output


if __name__ == '__main__':
    base_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(base_dir, '输出示例')
    os.makedirs(output_dir, exist_ok=True)

    all_results = []
    for i in range(1, 5):
        file_path = os.path.join(base_dir, '输入示例', f'example{i}.json')
        if os.path.exists(file_path):
            success, placements = solve(file_path)
            result = format_output(success, placements)
            all_results.append((file_path, result))

            output_path = os.path.join(output_dir, f'output{i}.json')
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)
            print(f"Output saved to {output_path}")

    print(f"\n{'='*60}")
    print("Summary")
    print(f"{'='*60}")
    for path, result in all_results:
        status = "FEASIBLE" if result["feasible"] else "NOT FEASIBLE"
        print(f"  {os.path.basename(path)}: {status} ({len(result['placements'])} items)")
