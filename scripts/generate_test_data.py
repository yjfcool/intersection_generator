#!/usr/bin/env python3
"""
Generate realistic intersection test data JSON files.

Generates various intersection types with proper lane geometry, connections,
obstacles, road edges, and rough areas. Uses projected coordinates (meters)
centered around origin (0,0).

Usage:
    python3 scripts/generate_test_data.py
"""

import json
import math
import os
import random
from typing import List, Tuple, Dict, Any

# Fixed seed for reproducibility
random.seed(42)

# Constants
ARM_LENGTH_MIN = 30.0
ARM_LENGTH_MAX = 50.0
INTERSECTION_RADIUS_MIN = 12.0
INTERSECTION_RADIUS_MAX = 20.0
LANE_WIDTH_MIN = 2.8
LANE_WIDTH_MAX = 3.5
LANE_COUNT_MIN = 2
LANE_COUNT_MAX = 5
MAX_ANGLE_OFFSET_DEG = 60.0

# Output directory relative to project root
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "test", "integration", "data")


# ============================================================================
# Utility / Helper Functions
# ============================================================================

def rand_lane_count() -> int:
    """Random lane count between 2 and 5."""
    return random.randint(LANE_COUNT_MIN, LANE_COUNT_MAX)


def rand_lane_width() -> float:
    """Random lane width between 2.8m and 3.5m."""
    return round(random.uniform(LANE_WIDTH_MIN, LANE_WIDTH_MAX), 2)


def rand_arm_length() -> float:
    """Random arm length between 30m and 50m."""
    return round(random.uniform(ARM_LENGTH_MIN, ARM_LENGTH_MAX), 1)


def rand_intersection_radius() -> float:
    """Random intersection radius between 12m and 20m."""
    return round(random.uniform(INTERSECTION_RADIUS_MIN, INTERSECTION_RADIUS_MAX), 1)


def deg_to_rad(deg: float) -> float:
    return math.radians(deg)


def rotate_point(x: float, y: float, angle_rad: float) -> Tuple[float, float]:
    """Rotate point (x,y) around origin by angle_rad."""
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    rx = x * cos_a - y * sin_a
    ry = x * sin_a + y * cos_a
    return round(rx, 3), round(ry, 3)


def offset_point(x: float, y: float, dx: float, dy: float) -> Tuple[float, float]:
    """Offset a point."""
    return round(x + dx, 3), round(y + dy, 3)


def make_linestring(coords: List[Tuple[float, float]]) -> Dict:
    """Create a GeoJSON LineString geometry."""
    return {"type": "LineString", "coordinates": [[c[0], c[1]] for c in coords]}


def make_polygon(coords: List[Tuple[float, float]]) -> Dict:
    """Create a GeoJSON Polygon geometry (single ring, auto-close)."""
    ring = [[c[0], c[1]] for c in coords]
    if ring[0] != ring[-1]:
        ring.append(ring[0])
    return {"type": "Polygon", "coordinates": [ring]}


def generate_lane_widths(n_lanes: int) -> List[float]:
    """Generate a list of lane widths."""
    return [rand_lane_width() for _ in range(n_lanes)]


def compute_lane_offsets(lane_widths: List[float]) -> List[float]:
    """
    Compute centerline offsets from the inner edge (median side).
    Lane 0 is innermost (closest to median).
    Returns the center offset for each lane measured from the inner boundary.
    """
    offsets = []
    cumulative = 0.0
    for w in lane_widths:
        offsets.append(cumulative + w / 2.0)
        cumulative += w
    return offsets


def compute_edge_offsets(lane_widths: List[float]) -> List[float]:
    """
    Compute edge line offsets from inner edge.
    For N lanes, returns 2*N offsets: [lane0_left, lane0_right, lane1_left, lane1_right, ...]
    lane_i_left = sum(widths[0:i])
    lane_i_right = sum(widths[0:i+1])
    """
    offsets = []
    cumulative = 0.0
    for w in lane_widths:
        offsets.append(cumulative)
        cumulative += w
        offsets.append(cumulative)
    return offsets


def total_width(lane_widths: List[float]) -> float:
    return sum(lane_widths)


# ============================================================================
# Arm / Lane Group Generation
# ============================================================================

class ArmConfig:
    """Configuration for one road arm approaching the intersection."""

    def __init__(self, direction_name: str, angle_deg: float,
                 enter_lanes: int = None, exit_lanes: int = None,
                 arm_length: float = None, intersection_radius: float = None,
                 angle_offset_deg: float = 0.0):
        self.direction_name = direction_name
        self.angle_deg = angle_deg  # angle from positive X axis
        self.angle_offset_deg = angle_offset_deg  # slight misalignment
        self.enter_lanes = enter_lanes or rand_lane_count()
        self.exit_lanes = exit_lanes or rand_lane_count()
        self.enter_widths = generate_lane_widths(self.enter_lanes)
        self.exit_widths = generate_lane_widths(self.exit_lanes)
        self.arm_length = arm_length or rand_arm_length()
        self.radius = intersection_radius or rand_intersection_radius()


def generate_arm_geometry(arm: ArmConfig) -> Dict[str, Any]:
    """
    Generate centerlines, edgelines, and road edges for one arm.

    Convention (right-hand traffic):
    - The arm extends along a direction given by arm.angle_deg from origin.
    - arm direction points AWAY from intersection center.
    - Vehicles ENTERING the intersection travel OPPOSITE to arm direction.
    - For right-hand traffic, entering vehicles are on their RIGHT side of the road.
    - From the arm's away-direction perspective:
        Enter lanes are to the LEFT (positive perpendicular) — driver's right when approaching
        Exit lanes are to the RIGHT (negative perpendicular) — driver's right when departing
    - Enter lines: far endpoint -> near endpoint (towards intersection)
    - Exit lines: near endpoint -> far endpoint (away from intersection)

    Returns dict with keys: enter_centerlines, exit_centerlines,
                           enter_edgelines, exit_edgelines,
                           enter_road_edges, exit_road_edges
    """
    angle_rad = deg_to_rad(arm.angle_deg + arm.angle_offset_deg)
    # Direction vector (unit) pointing AWAY from intersection
    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)
    # Perpendicular (to the left of direction)
    px = -dy  # perpendicular pointing left
    py = dx

    near_dist = arm.radius
    far_dist = arm.arm_length

    # Median is at offset 0 in perpendicular direction
    # For right-hand traffic rule:
    #   The arm direction points AWAY from intersection.
    #   A vehicle ENTERING the intersection travels OPPOSITE to the arm direction.
    #   From the entering driver's perspective, they should be on the RIGHT side of the road.
    #   Driver's right = LEFT of the arm's away-direction (positive perpendicular).
    # Therefore:
    #   Enter lanes are to the LEFT of arm away-direction (positive perpendicular)
    #   Exit lanes are to the RIGHT of arm away-direction (negative perpendicular)

    result = {
        "enter_centerlines": [],
        "exit_centerlines": [],
        "enter_edgelines": [],
        "exit_edgelines": [],
        "enter_road_edges": [],
        "exit_road_edges": [],
    }

    dir_abbr = arm.direction_name

    # --- Enter lanes (left side of away-direction = positive perpendicular) ---
    enter_offsets = compute_lane_offsets(arm.enter_widths)
    enter_edge_offsets = compute_edge_offsets(arm.enter_widths)

    for i, offset in enumerate(enter_offsets):
        # Offset in positive perpendicular direction (left of away = right of approaching driver)
        perp_offset = offset  # positive = left of away-direction = driver's right
        # Near point (close to intersection)
        near_x = dx * near_dist + px * perp_offset
        near_y = dy * near_dist + py * perp_offset
        # Far point
        far_x = dx * far_dist + px * perp_offset
        far_y = dy * far_dist + py * perp_offset

        cl_id = f"cl_{dir_abbr}{i+1}"
        # Enter: far -> near (towards intersection)
        geom = make_linestring([(round(far_x, 3), round(far_y, 3)),
                                (round(near_x, 3), round(near_y, 3))])
        result["enter_centerlines"].append({
            "id": cl_id,
            "geom": geom,
            "attrs": {"lane_type": "driving"}
        })

    # Enter edgelines
    for i, offset in enumerate(enter_edge_offsets):
        perp_offset = offset  # positive = left of away = driver's right
        near_x = dx * near_dist + px * perp_offset
        near_y = dy * near_dist + py * perp_offset
        far_x = dx * far_dist + px * perp_offset
        far_y = dy * far_dist + py * perp_offset

        lane_idx = i // 2
        side = "l" if i % 2 == 0 else "r"
        el_id = f"el_{dir_abbr}{lane_idx+1}_{side}"
        # Same direction as enter centerlines: far -> near
        geom = make_linestring([(round(far_x, 3), round(far_y, 3)),
                                (round(near_x, 3), round(near_y, 3))])
        result["enter_edgelines"].append({
            "id": el_id,
            "geom": geom,
            "attrs": {}
        })

    # Enter road edges (innermost = median side, outermost = far from median)
    # Median edge at offset 0; outer edge at +total_width (positive perp direction)
    for edge_side, perp_offset in [("inner", 0.0), ("outer", total_width(arm.enter_widths))]:
        near_x = dx * near_dist + px * perp_offset
        near_y = dy * near_dist + py * perp_offset
        far_x = dx * far_dist + px * perp_offset
        far_y = dy * far_dist + py * perp_offset

        re_id = f"re_{dir_abbr}_enter_{edge_side}"
        geom = make_linestring([(round(far_x, 3), round(far_y, 3)),
                                (round(near_x, 3), round(near_y, 3))])
        result["enter_road_edges"].append({
            "id": re_id,
            "geom": geom,
            "attrs": {}
        })

    # --- Exit lanes (right side of away-direction = negative perpendicular) ---
    exit_offsets = compute_lane_offsets(arm.exit_widths)
    exit_edge_offsets = compute_edge_offsets(arm.exit_widths)

    for i, offset in enumerate(exit_offsets):
        perp_offset = -(offset)  # negative = right of away-direction = driver's right when exiting
        near_x = dx * near_dist + px * perp_offset
        near_y = dy * near_dist + py * perp_offset
        far_x = dx * far_dist + px * perp_offset
        far_y = dy * far_dist + py * perp_offset

        cl_id = f"cl_x{dir_abbr}{i+1}"
        # Exit: near -> far (away from intersection)
        geom = make_linestring([(round(near_x, 3), round(near_y, 3)),
                                (round(far_x, 3), round(far_y, 3))])
        result["exit_centerlines"].append({
            "id": cl_id,
            "geom": geom,
            "attrs": {"lane_type": "driving"}
        })

    # Exit edgelines
    for i, offset in enumerate(exit_edge_offsets):
        perp_offset = -(offset)  # negative = right of away-direction
        near_x = dx * near_dist + px * perp_offset
        near_y = dy * near_dist + py * perp_offset
        far_x = dx * far_dist + px * perp_offset
        far_y = dy * far_dist + py * perp_offset

        lane_idx = i // 2
        side = "l" if i % 2 == 0 else "r"
        el_id = f"el_x{dir_abbr}{lane_idx+1}_{side}"
        # Exit direction: near -> far
        geom = make_linestring([(round(near_x, 3), round(near_y, 3)),
                                (round(far_x, 3), round(far_y, 3))])
        result["exit_edgelines"].append({
            "id": el_id,
            "geom": geom,
            "attrs": {}
        })

    # Exit road edges
    # Median edge at offset 0; outer edge at -total_width (negative perp direction)
    for edge_side, perp_offset in [("inner", 0.0), ("outer", -(total_width(arm.exit_widths)))]:
        near_x = dx * near_dist + px * perp_offset
        near_y = dy * near_dist + py * perp_offset
        far_x = dx * far_dist + px * perp_offset
        far_y = dy * far_dist + py * perp_offset

        re_id = f"re_{dir_abbr}_exit_{edge_side}"
        geom = make_linestring([(round(near_x, 3), round(near_y, 3)),
                                (round(far_x, 3), round(far_y, 3))])
        result["exit_road_edges"].append({
            "id": re_id,
            "geom": geom,
            "attrs": {}
        })

    return result



# ============================================================================
# Connection Generation
# ============================================================================

def determine_turn_type(enter_angle_deg: float, exit_angle_deg: float) -> str:
    """
    Determine turn type based on angle difference.
    Enter arm points TOWARD intersection, exit arm points AWAY from intersection.
    The angular difference determines the turn type.
    """
    # Normalize the difference: how much do you turn from enter direction to exit direction
    # Enter direction towards center = enter_angle_deg + 180
    enter_toward = (enter_angle_deg + 180) % 360
    # Exit direction away from center = exit_angle_deg
    diff = (exit_angle_deg - enter_toward + 360) % 360

    if diff < 30 or diff > 330:
        return "straight"
    elif 30 <= diff <= 150:
        return "left"
    elif 150 < diff < 210:
        return "uturn"
    else:  # 210 <= diff <= 330
        return "right"


def generate_connections(arms: List[ArmConfig], arm_geoms: List[Dict]) -> List[Dict]:
    """
    Generate turn connections between enter and exit groups.
    Creates realistic connections with various turn types.
    """
    connections = []
    conn_counter = 0

    enter_arms = [(i, arm) for i, arm in enumerate(arms)]
    exit_arms = [(i, arm) for i, arm in enumerate(arms)]

    for enter_idx, enter_arm in enter_arms:
        enter_geom = arm_geoms[enter_idx]
        enter_cls = enter_geom["enter_centerlines"]

        for exit_idx, exit_arm in exit_arms:
            if enter_idx == exit_idx:
                # U-turn connections (only from outermost enter lane)
                if random.random() < 0.3:  # 30% chance of u-turn
                    exit_geom = arm_geoms[exit_idx]
                    exit_cls = exit_geom["exit_centerlines"]
                    # U-turn from innermost enter lane to innermost exit lane
                    conn_counter += 1
                    connections.append({
                        "id": f"conn_{conn_counter:03d}",
                        "enter_group_id": f"lg_enter_{enter_arm.direction_name}",
                        "exit_group_id": f"lg_exit_{exit_arm.direction_name}",
                        "enter_line_id": enter_cls[0]["id"],
                        "exit_line_id": exit_cls[0]["id"],
                        "turn_type": "uturn"
                    })
                continue

            exit_geom = arm_geoms[exit_idx]
            exit_cls = exit_geom["exit_centerlines"]
            turn_type = determine_turn_type(enter_arm.angle_deg, exit_arm.angle_deg)

            if turn_type == "straight":
                # 1:1 mapping for straight connections
                n_conns = min(len(enter_cls), len(exit_cls))
                for k in range(n_conns):
                    conn_counter += 1
                    connections.append({
                        "id": f"conn_{conn_counter:03d}",
                        "enter_group_id": f"lg_enter_{enter_arm.direction_name}",
                        "exit_group_id": f"lg_exit_{exit_arm.direction_name}",
                        "enter_line_id": enter_cls[k]["id"],
                        "exit_line_id": exit_cls[k]["id"],
                        "turn_type": "straight"
                    })
            elif turn_type == "left":
                # Left turn: typically from inner lanes to first exit lane
                n_left = min(2, len(enter_cls))
                for k in range(n_left):
                    exit_lane_idx = min(k, len(exit_cls) - 1)
                    conn_counter += 1
                    connections.append({
                        "id": f"conn_{conn_counter:03d}",
                        "enter_group_id": f"lg_enter_{enter_arm.direction_name}",
                        "exit_group_id": f"lg_exit_{exit_arm.direction_name}",
                        "enter_line_id": enter_cls[k]["id"],
                        "exit_line_id": exit_cls[exit_lane_idx]["id"],
                        "turn_type": "left"
                    })
            elif turn_type == "right":
                # Right turn: typically from outer lanes
                n_right = min(2, len(enter_cls))
                for k in range(n_right):
                    enter_lane_idx = len(enter_cls) - 1 - k
                    exit_lane_idx = min(k, len(exit_cls) - 1)
                    conn_counter += 1
                    connections.append({
                        "id": f"conn_{conn_counter:03d}",
                        "enter_group_id": f"lg_enter_{enter_arm.direction_name}",
                        "exit_group_id": f"lg_exit_{exit_arm.direction_name}",
                        "enter_line_id": enter_cls[enter_lane_idx]["id"],
                        "exit_line_id": exit_cls[exit_lane_idx]["id"],
                        "turn_type": "right"
                    })

    return connections


# ============================================================================
# Obstacle Generation
# ============================================================================

def generate_obstacles(n_obstacles: int, intersection_radius: float) -> List[Dict]:
    """Generate polygon obstacles within the intersection area."""
    obstacles = []
    for i in range(n_obstacles):
        # Random position within intersection area
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(2.0, intersection_radius * 0.6)
        cx = dist * math.cos(angle)
        cy = dist * math.sin(angle)

        # Random polygon (small convex shape)
        n_vertices = random.randint(4, 6)
        size = random.uniform(1.0, 3.0)
        vertices = []
        for j in range(n_vertices):
            v_angle = 2 * math.pi * j / n_vertices + random.uniform(-0.2, 0.2)
            r = size * random.uniform(0.7, 1.3)
            vx = cx + r * math.cos(v_angle)
            vy = cy + r * math.sin(v_angle)
            vertices.append((round(vx, 3), round(vy, 3)))

        obstacles.append({
            "id": f"obs_{i+1}",
            "geom_type": "polygon",
            "geom": make_polygon(vertices),
            "attrs": {"type": random.choice(["median_island", "traffic_island", "bollard_cluster"])}
        })
    return obstacles


# ============================================================================
# Rough Area Generation
# ============================================================================

def generate_rough_area(arms: List[ArmConfig], intersection_radius: float) -> Dict:
    """Generate a rough area polygon encompassing the intersection."""
    # Create a polygon that covers the intersection area
    n_points = max(len(arms) * 2, 8)
    points = []
    for i in range(n_points):
        angle = 2 * math.pi * i / n_points
        r = intersection_radius * 1.2
        points.append((round(r * math.cos(angle), 3), round(r * math.sin(angle), 3)))
    return {
        "id": "rough_01",
        "geom": make_polygon(points),
        "attrs": {}
    }


# ============================================================================
# Full Intersection Assembly
# ============================================================================

def build_intersection(scenario_id: str, arms: List[ArmConfig],
                       n_obstacles: int = 2,
                       ref_lon: float = 116.395,
                       ref_lat: float = 39.911) -> Dict:
    """
    Build a complete intersection JSON structure from arm configurations.
    """
    # Generate geometry for each arm
    arm_geoms = [generate_arm_geometry(arm) for arm in arms]

    # Average intersection radius
    avg_radius = sum(a.radius for a in arms) / len(arms)

    # Assemble lane_groups
    lane_groups = []
    all_centerlines = []
    all_edgelines = []
    all_road_edges = []

    for i, (arm, geom) in enumerate(zip(arms, arm_geoms)):
        # Enter group
        enter_cl_ids = [cl["id"] for cl in geom["enter_centerlines"]]
        enter_el_ids = [el["id"] for el in geom["enter_edgelines"]]
        lane_groups.append({
            "id": f"lg_enter_{arm.direction_name}",
            "type": "enter",
            "centerline_ids": enter_cl_ids,
            "edgeline_ids": enter_el_ids
        })

        # Exit group
        exit_cl_ids = [cl["id"] for cl in geom["exit_centerlines"]]
        exit_el_ids = [el["id"] for el in geom["exit_edgelines"]]
        lane_groups.append({
            "id": f"lg_exit_{arm.direction_name}",
            "type": "exit",
            "centerline_ids": exit_cl_ids,
            "edgeline_ids": exit_el_ids
        })

        # Collect all geometries
        all_centerlines.extend(geom["enter_centerlines"])
        all_centerlines.extend(geom["exit_centerlines"])
        all_edgelines.extend(geom["enter_edgelines"])
        all_edgelines.extend(geom["exit_edgelines"])
        all_road_edges.extend(geom["enter_road_edges"])
        all_road_edges.extend(geom["exit_road_edges"])

    # Generate connections
    connections = generate_connections(arms, arm_geoms)

    # Generate obstacles
    obstacles = generate_obstacles(n_obstacles, avg_radius)

    # Generate rough area
    rough_area = generate_rough_area(arms, avg_radius)

    return {
        "scenario_id": scenario_id,
        "ref_point": {"lon": round(ref_lon, 4), "lat": round(ref_lat, 4)},
        "coord_type": "projected",
        "lane_groups": lane_groups,
        "centerlines": all_centerlines,
        "edgelines": all_edgelines,
        "connections": connections,
        "obstacles": obstacles,
        "stop_lines": [],
        "road_edges": all_road_edges,
        "rough_area": rough_area
    }



# ============================================================================
# Scenario Generators
# ============================================================================

def generate_cross_4way() -> Dict:
    """Standard 4-way cross intersection (十字型)."""
    # Apply slight angle offsets to simulate imperfect alignment
    offset_s = random.uniform(-5, 5)
    offset_n = random.uniform(-5, 5)
    radius = rand_intersection_radius()

    arms = [
        ArmConfig("south", 270.0, angle_offset_deg=offset_s, intersection_radius=radius),
        ArmConfig("north", 90.0, angle_offset_deg=offset_n, intersection_radius=radius),
        ArmConfig("west", 180.0, angle_offset_deg=random.uniform(-3, 3), intersection_radius=radius),
        ArmConfig("east", 0.0, angle_offset_deg=random.uniform(-3, 3), intersection_radius=radius),
    ]
    return build_intersection("cross_4way_01", arms, n_obstacles=2,
                              ref_lon=116.391, ref_lat=39.907)


def generate_hash_intersection() -> Dict:
    """
    Hash/grid intersection (井字型).
    An offset grid with 4+ arms - essentially a pair of offset cross intersections.
    We model this as a 4-arm intersection with wider spacing and slight offsets.
    """
    radius = rand_intersection_radius()
    # The hash shape has arms at roughly 90 degree intervals but with positional offsets
    arms = [
        ArmConfig("south", 265.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-8, 0)),
        ArmConfig("north", 85.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(0, 8)),
        ArmConfig("west", 175.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-5, 5)),
        ArmConfig("east", 5.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-5, 5)),
        ArmConfig("sw", 225.0, enter_lanes=2, exit_lanes=2,
                  intersection_radius=radius,
                  angle_offset_deg=random.uniform(-5, 5)),
    ]
    return build_intersection("hash_intersection_01", arms, n_obstacles=3,
                              ref_lon=116.393, ref_lat=39.909)


def generate_t_intersection() -> Dict:
    """T-intersection (T字型) - 3 arms."""
    radius = rand_intersection_radius()
    arms = [
        ArmConfig("south", 270.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-3, 3)),
        ArmConfig("west", 180.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-5, 5)),
        ArmConfig("east", 0.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-5, 5)),
    ]
    return build_intersection("t_intersection_01", arms, n_obstacles=1,
                              ref_lon=116.392, ref_lat=39.908)


def generate_y_intersection() -> Dict:
    """Y-intersection (Y字型) - 3 arms at roughly 120 degree angles."""
    radius = rand_intersection_radius()
    base_angles = [270.0, 30.0, 150.0]  # roughly 120 apart
    arms = [
        ArmConfig("south", base_angles[0], intersection_radius=radius,
                  angle_offset_deg=random.uniform(-10, 10)),
        ArmConfig("ne", base_angles[1], intersection_radius=radius,
                  angle_offset_deg=random.uniform(-10, 10)),
        ArmConfig("nw", base_angles[2], intersection_radius=radius,
                  angle_offset_deg=random.uniform(-10, 10)),
    ]
    return build_intersection("y_intersection_01", arms, n_obstacles=1,
                              ref_lon=116.394, ref_lat=39.910)


def generate_h_intersection() -> Dict:
    """
    H-intersection (H字型) - two parallel roads connected by a perpendicular road.
    Modeled as 4 arms: 2 on the left parallel road, 2 on the right, connected in the middle.
    """
    radius = rand_intersection_radius()
    arms = [
        ArmConfig("north", 90.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-3, 3)),
        ArmConfig("south", 270.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-3, 3)),
        ArmConfig("east", 0.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-3, 3)),
        ArmConfig("west", 180.0, intersection_radius=radius,
                  angle_offset_deg=random.uniform(-3, 3)),
    ]
    # Simulate H shape: the east-west arms are the connecting road (fewer lanes)
    arms[2].enter_lanes = 2
    arms[2].exit_lanes = 2
    arms[2].enter_widths = generate_lane_widths(2)
    arms[2].exit_widths = generate_lane_widths(2)
    arms[3].enter_lanes = 2
    arms[3].exit_lanes = 2
    arms[3].enter_widths = generate_lane_widths(2)
    arms[3].exit_widths = generate_lane_widths(2)

    return build_intersection("h_intersection_01", arms, n_obstacles=2,
                              ref_lon=116.395, ref_lat=39.911)


def generate_tree_intersection() -> Dict:
    """
    Tree/木 intersection (木字型) - main road with 3 side roads branching off.
    Main road runs north-south, with 3 side roads at various angles.
    """
    radius = rand_intersection_radius()
    arms = [
        # Main road (north-south)
        ArmConfig("north", 90.0, enter_lanes=3, exit_lanes=3,
                  intersection_radius=radius),
        ArmConfig("south", 270.0, enter_lanes=3, exit_lanes=3,
                  intersection_radius=radius),
        # Side roads (smaller)
        ArmConfig("east", 0.0, enter_lanes=2, exit_lanes=2,
                  intersection_radius=radius,
                  angle_offset_deg=random.uniform(-10, 10)),
        ArmConfig("sw", 225.0, enter_lanes=2, exit_lanes=2,
                  intersection_radius=radius,
                  angle_offset_deg=random.uniform(-5, 5)),
        ArmConfig("se", 315.0, enter_lanes=2, exit_lanes=2,
                  intersection_radius=radius,
                  angle_offset_deg=random.uniform(-5, 5)),
    ]
    return build_intersection("tree_intersection_01", arms, n_obstacles=2,
                              ref_lon=116.396, ref_lat=39.912)


def generate_star_intersection() -> Dict:
    """Star/米 intersection (米字型) - 6-8 arms radiating from center."""
    radius = rand_intersection_radius()
    n_arms = random.choice([6, 7, 8])
    angle_step = 360.0 / n_arms
    arm_names = ["a", "b", "c", "d", "e", "f", "g", "h"]

    arms = []
    for i in range(n_arms):
        angle = i * angle_step
        arms.append(ArmConfig(
            arm_names[i], angle,
            enter_lanes=random.randint(2, 3),
            exit_lanes=random.randint(2, 3),
            intersection_radius=radius,
            angle_offset_deg=random.uniform(-5, 5)
        ))
    return build_intersection("star_intersection_01", arms, n_obstacles=3,
                              ref_lon=116.397, ref_lat=39.913)


def generate_roundabout() -> Dict:
    """
    Roundabout (环岛) - circular intersection with 4 arms.
    The central island is a large circular obstacle.
    """
    radius = random.uniform(15.0, 20.0)
    arms = [
        ArmConfig("south", 270.0, enter_lanes=2, exit_lanes=2,
                  intersection_radius=radius),
        ArmConfig("north", 90.0, enter_lanes=2, exit_lanes=2,
                  intersection_radius=radius),
        ArmConfig("west", 180.0, enter_lanes=2, exit_lanes=2,
                  intersection_radius=radius),
        ArmConfig("east", 0.0, enter_lanes=2, exit_lanes=2,
                  intersection_radius=radius),
    ]

    data = build_intersection("roundabout_01", arms, n_obstacles=0,
                              ref_lon=116.398, ref_lat=39.914)

    # Add circular central island as the main obstacle
    island_radius = radius * 0.5
    n_pts = 16
    island_pts = []
    for i in range(n_pts):
        angle = 2 * math.pi * i / n_pts
        island_pts.append((round(island_radius * math.cos(angle), 3),
                           round(island_radius * math.sin(angle), 3)))

    data["obstacles"] = [{
        "id": "obs_central_island",
        "geom_type": "polygon",
        "geom": make_polygon(island_pts),
        "attrs": {"type": "roundabout_island"}
    }]

    # Update rough area to be circular
    rough_pts = []
    rough_r = radius * 1.3
    for i in range(20):
        angle = 2 * math.pi * i / 20
        rough_pts.append((round(rough_r * math.cos(angle), 3),
                          round(rough_r * math.sin(angle), 3)))
    data["rough_area"] = {
        "id": "rough_01",
        "geom": make_polygon(rough_pts),
        "attrs": {"intersection_type": "roundabout"}
    }

    return data



# ============================================================================
# Output / Main
# ============================================================================

def write_json(data: Dict, filename: str) -> None:
    """Write intersection data to a JSON file."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    filepath = os.path.join(OUTPUT_DIR, filename)
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"  Written: {filepath}")


def main():
    """Generate all test intersection scenarios."""
    print("Generating intersection test data...")
    print(f"Output directory: {OUTPUT_DIR}")
    print()

    scenarios = [
        ("cross_4way.json", generate_cross_4way),
        ("hash_intersection.json", generate_hash_intersection),
        ("t_intersection.json", generate_t_intersection),
        ("y_intersection.json", generate_y_intersection),
        ("h_intersection.json", generate_h_intersection),
        ("tree_intersection.json", generate_tree_intersection),
        ("star_intersection.json", generate_star_intersection),
        ("roundabout.json", generate_roundabout),
    ]

    for filename, generator in scenarios:
        print(f"Generating {filename}...")
        # Reset seed per scenario for independent reproducibility
        # (main seed 42 was set at module level, giving consistent random state flow)
        data = generator()

        # Print summary
        n_groups = len(data["lane_groups"])
        n_cls = len(data["centerlines"])
        n_conns = len(data["connections"])
        n_obs = len(data["obstacles"])
        print(f"    Lane groups: {n_groups}, Centerlines: {n_cls}, "
              f"Connections: {n_conns}, Obstacles: {n_obs}")

        write_json(data, filename)

    print()
    print("Done! All test data files generated successfully.")


if __name__ == "__main__":
    main()
