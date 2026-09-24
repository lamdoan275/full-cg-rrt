#!/usr/bin/env python3
"""Generate a benchmark map shaped like the supplied S-corridor/U-trap sketch.

The map keeps the same resolution and origin convention as maze_19/map_extra:
  - a closed rectangular room;
  - two offset quarter-circle walls forming an S-shaped central passage;
  - an upward-open U-shaped dead-end around the goal.

Generated files:
  sim_env/worlds/map_s_trap.world
  sim_env/maps/map_s_trap/map_s_trap.pgm
  sim_env/maps/map_s_trap/map_s_trap.yaml
"""

import math
import os
import sys

# Reuse the tested Gazebo/PGM helpers and map conventions from map_extra.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_map_extra as base  # noqa: E402


MAP_NAME = "map_s_trap"
WALL_THICK = 0.15
WALL_HEIGHT = 2.5
# Passage sizing for the TurtleBot3 Waffle.  Every corridor the robot actually
# drives through -- the central S passage, the mouth of the goal trap, and the
# band between the trap arms and the top wall -- is built from one rule: the
# robot footprint keeps ROBOT_TO_COSTMAP_CLEARANCE free of the 0.40 m global
# costmap inflation on *both* sides.
ROBOT_WIDTH = 0.31
COSTMAP_INFLATION = 0.40
ROBOT_TO_COSTMAP_CLEARANCE = 0.05
# Segmented curved walls protrude slightly farther inward after rasterization.
# This calibrated allowance preserves a full 0.05 m map cell at the tightest
# point of the curve, not only in the continuous-geometry calculation.
CURVE_RASTER_ALLOWANCE = 0.03
# wall centreline -> path centreline
PASSAGE_HALF_GAP = (0.5 * WALL_THICK + 0.5 * ROBOT_WIDTH +
                    COSTMAP_INFLATION + ROBOT_TO_COSTMAP_CLEARANCE)
# wall *surface* -> path centreline: the clearance a distance transform of the
# generated .pgm has to report everywhere along a driveable corridor.
PASSAGE_HALF_FREE = PASSAGE_HALF_GAP - 0.5 * WALL_THICK + CURVE_RASTER_ALLOWANCE
S_PASSAGE_HALF_GAP = PASSAGE_HALF_GAP + CURVE_RASTER_ALLOWANCE

ROOM_HALF_X = 4.60
ROOM_BOTTOM_Y = -4.60
MAP_W = MAP_H = 384
RESOLUTION = 0.05
ORIGIN = (-10.0, -10.0)

ROOM_FLOOR_SURFACE = ROOM_BOTTOM_Y + 0.5 * WALL_THICK
# Height of the pocket the robot spawns in, from the bottom wall surface up to
# the lower S wall surface.  Lowering it pulls the whole S structure -- and
# with it the goal trap -- further down towards the robot.  The floor is
# 2 * PASSAGE_HALF_FREE, at which point the robot's 0.05 m margin to the
# inflation on each side is all that is left.
SPAWN_POCKET_H = 1.96
# The robot starts balanced at the centre of that pocket, the same distance
# from the inflated bottom wall below as from the inflated S wall above.
FLOOD_SEED = (0.0, ROOM_FLOOR_SURFACE + 0.5 * SPAWN_POCKET_H)

# Centreline of the S passage, as a cubic Bezier with horizontal end tangents.
S_BEZIER = ((-1.55, 0.60), (-0.45, 0.60), (-0.35, -1.00), (0.85, -1.00))
# Lowest S wall surface before shifting: at t=1 the curve normal is vertical,
# so the lower offset wall sits one half-gap below the last control point.
_S_LOWEST = S_BEZIER[3][1] - S_PASSAGE_HALF_GAP - 0.5 * WALL_THICK
# Drop the whole S structure until it closes the spawn pocket to
# SPAWN_POCKET_H.  Pulling the two hooks down towards the robot is what leaves
# the goal trap standing clear of them, with a full-width corridor under the
# bowl, so the robot can circle right around the trap and enter its mouth.
S_SHIFT_Y = (ROOM_FLOOR_SURFACE + SPAWN_POCKET_H) - _S_LOWEST
# Top surface of the upper S wall, which the goal trap has to stand clear of.
S_TOP_SURFACE = (S_BEZIER[0][1] + S_SHIFT_Y + S_PASSAGE_HALF_GAP +
                 0.5 * WALL_THICK)

# Goal trap: the curved U from map_extra -- a true semicircular bowl closed at
# the bottom, with two straight vertical arms rising from its widest points.
# Scale the complete U around its existing geometric centre so changing size
# does not move the trap away from the intended location.
U_BASE_RADIUS = 1.25
U_BASE_ARM_LEN = 1.00
U_SCALE = 0.80
U_RADIUS = U_BASE_RADIUS * U_SCALE
U_ARM_LEN = U_BASE_ARM_LEN * U_SCALE
U_ARC_SEGMENTS = 30
# Additional free space between the bottom of the U and the upper S wall.
U_EXTRA_LOWER_CLEARANCE = 0.30
# Preserve the pre-scale bounding-box centre of the U. The old placement was
# derived from the S-wall clearance, so compute that reference centre first and
# then recover the arc centre after scaling the radius and arms.
_U_REFERENCE_BOTTOM_Y = (S_TOP_SURFACE + 2.0 * PASSAGE_HALF_FREE +
                         0.5 * WALL_THICK + U_EXTRA_LOWER_CLEARANCE)
U_CENTER_Y = (_U_REFERENCE_BOTTOM_Y + U_BASE_RADIUS +
              0.5 * (U_BASE_ARM_LEN - U_BASE_RADIUS))
U_ARC_Y = U_CENTER_Y + 0.5 * (U_RADIUS - U_ARM_LEN)
U_MOUTH_Y = U_ARC_Y + U_ARM_LEN
# Vertical arms, so the trap's highest point is simply the top of the arm box.
U_APEX_Y = U_MOUTH_Y
# Put the goal as deep in the bowl as the clearance rule allows.  Inside a
# circular bowl the clearance is U_RADIUS - |y - U_ARC_Y| - WALL_THICK / 2.
GOAL = (0.0, U_ARC_Y - (U_RADIUS - 0.5 * WALL_THICK - PASSAGE_HALF_FREE))

# Raise the ceiling until the band the robot crosses above the trap arms is
# exactly as wide as every other passage on this map.
ROOM_TOP_Y = U_APEX_Y + 2.0 * PASSAGE_HALF_FREE + 0.5 * WALL_THICK
ROOM_CENTER_Y = 0.5 * (ROOM_TOP_Y + ROOM_BOTTOM_Y)
ROOM_LEN_X = 2.0 * ROOM_HALF_X + WALL_THICK
ROOM_LEN_Y = (ROOM_TOP_Y - ROOM_BOTTOM_Y) + WALL_THICK


def segment(cx1, cy1, cx2, cy2, name):
    """Return one wall box whose long axis connects two world points."""
    dx, dy = cx2 - cx1, cy2 - cy1
    length = math.hypot(dx, dy)
    return ((cx1 + cx2) / 2.0, (cy1 + cy2) / 2.0, length + 0.04,
            WALL_THICK, math.atan2(dy, dx), name)


def arc_segments(cx, cy, radius, start_deg, end_deg, count, name):
    """Approximate an arc with overlapping boxes following its tangent."""
    step = math.radians(end_deg - start_deg) / count
    # Slight overlap keeps the occupancy grid and Gazebo wall watertight.
    length = 2.0 * radius * math.sin(abs(step) / 2.0) * 1.25
    boxes = []
    for i in range(count):
        theta = math.radians(start_deg) + (i + 0.5) * step
        boxes.append((cx + radius * math.cos(theta),
                      cy + radius * math.sin(theta),
                      length, WALL_THICK, theta + math.pi / 2.0,
                      "%s_%02d" % (name, i)))
    return boxes


def bezier_parallel_walls(points, half_gap, count):
    """Create two constant-offset walls around a cubic Bezier centreline.

    Offset is measured along the local normal, so the free curved gap does not
    widen or narrow through the bend.  Both end tangents are horizontal, which
    lets the four straight arms meet the curves without a corner.
    """
    p0, p1, p2, p3 = points

    def point_and_tangent(t):
        u = 1.0 - t
        x = (u**3 * p0[0] + 3*u*u*t * p1[0] +
             3*u*t*t * p2[0] + t**3 * p3[0])
        y = (u**3 * p0[1] + 3*u*u*t * p1[1] +
             3*u*t*t * p2[1] + t**3 * p3[1])
        dx = (3*u*u * (p1[0] - p0[0]) +
              6*u*t * (p2[0] - p1[0]) +
              3*t*t * (p3[0] - p2[0]))
        dy = (3*u*u * (p1[1] - p0[1]) +
              6*u*t * (p2[1] - p1[1]) +
              3*t*t * (p3[1] - p2[1]))
        norm = math.hypot(dx, dy)
        return x, y, -dy / norm, dx / norm

    upper_points, lower_points = [], []
    for i in range(count + 1):
        x, y, nx, ny = point_and_tangent(i / float(count))
        upper_points.append((x + half_gap * nx, y + half_gap * ny))
        lower_points.append((x - half_gap * nx, y - half_gap * ny))

    def boxes(polyline, prefix):
        return [segment(*polyline[i], *polyline[i + 1], "%s_%02d" % (prefix, i))
                for i in range(len(polyline) - 1)]

    return upper_points, lower_points, boxes(upper_points, "S_upper_curve"), boxes(lower_points, "S_lower_curve")


def build_boxes():
    outer = [
        (0.0, ROOM_BOTTOM_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_bottom"),
        (0.0, ROOM_TOP_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_top"),
        (-ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_left"),
        (ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_right"),
    ]

    # Two opposing hooks, matching the reference image.  The left hook has two
    # arms attached to the left wall; the right hook has two arms attached to
    # the right wall.  Their inner Bezier curves are constant normal offsets.
    bezier = tuple((px, py + S_SHIFT_Y) for px, py in S_BEZIER)
    upper_pts, lower_pts, upper_curve, lower_curve = bezier_parallel_walls(
        bezier, half_gap=S_PASSAGE_HALF_GAP, count=28)

    upper_start, upper_end = upper_pts[0], upper_pts[-1]
    lower_start, lower_end = lower_pts[0], lower_pts[-1]
    s_walls = [
        # Left-facing hook: upper/lower arms plus the lower offset curve.
        segment(-ROOM_HALF_X + WALL_THICK / 2.0, lower_start[1],
                lower_start[0], lower_start[1], "Left_hook_upper_arm"),
        *lower_curve,
        segment(-ROOM_HALF_X + WALL_THICK / 2.0, lower_end[1],
                lower_end[0], lower_end[1], "Left_hook_lower_arm"),
        # Right-facing hook: upper/lower arms plus the upper offset curve.
        segment(upper_start[0], upper_start[1],
                ROOM_HALF_X - WALL_THICK / 2.0, upper_start[1], "Right_hook_upper_arm"),
        *upper_curve,
        segment(upper_end[0], upper_end[1],
                ROOM_HALF_X - WALL_THICK / 2.0, upper_end[1], "Right_hook_lower_arm"),
    ]

    # Goal trap: map_extra's curved U -- a semicircular bowl closed at the
    # bottom, with two straight vertical arms continuing tangentially from its
    # widest points up to the mouth.
    arm_cy = U_ARC_Y + U_ARM_LEN / 2.0
    u_walls = [
        *arc_segments(0.0, U_ARC_Y, U_RADIUS, 180.0, 360.0, U_ARC_SEGMENTS,
                      "Goal_U_bottom"),
        (-U_RADIUS, arm_cy, U_ARM_LEN, WALL_THICK, math.pi / 2.0,
         "Goal_U_left"),
        (U_RADIUS, arm_cy, U_ARM_LEN, WALL_THICK, math.pi / 2.0,
         "Goal_U_right"),
    ]
    return outer, s_walls, u_walls


def wall_link(name, cx, cy, lx, ly, yaw):
    size = "%g %g %g" % (lx, ly, WALL_HEIGHT)
    half_h = WALL_HEIGHT / 2.0
    return """      <link name='{name}'>
        <collision name='{name}_Collision'>
          <geometry><box><size>{size}</size></box></geometry>
          <pose>0 0 {half_h} 0 -0 0</pose>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='{name}_Visual'>
          <pose>0 0 {half_h} 0 -0 0</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
        </visual>
        <pose>{cx:.6f} {cy:.6f} 0 0 -0 {yaw:.6f}</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
""".format(name=name, size=size, half_h=half_h, cx=cx, cy=cy, yaw=yaw)


def write_world(path, boxes_by_model):
    out = [base.WORLD_HEAD]
    models = [("ground_plane", "0 0 0 0 -0 0", [("link", "0 0 0 0 -0 0")])]
    for model_name, boxes in boxes_by_model:
        out.append("    <model name='%s'>\n      <pose>0 0 0 0 -0 0</pose>\n" % model_name)
        links = []
        for cx, cy, lx, ly, yaw, name in boxes:
            out.append(wall_link(name, cx, cy, lx, ly, yaw))
            links.append((name, "%.6f %.6f 0 0 -0 %.6f" % (cx, cy, yaw)))
        out.append("      <static>1</static>\n    </model>\n")
        models.append((model_name, "0 0 0 0 -0 0", links))
    out.append(base.state_block(models))
    out.append(base.WORLD_TAIL)
    with open(path, "w") as stream:
        stream.write("".join(out))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    map_dir = os.path.join(pkg, "maps", MAP_NAME)
    os.makedirs(map_dir, exist_ok=True)

    outer, s_walls, u_walls = build_boxes()
    all_boxes = outer + s_walls + u_walls

    # Configure the reused rasterizer with this map's geometry and origin.
    base.MAP_W = base.MAP_H = MAP_W
    base.RESOLUTION = RESOLUTION
    base.ORIGIN = ORIGIN
    base.FLOOD_SEED = FLOOD_SEED
    base.UNIT_BOXES = []

    write_world(os.path.join(pkg, "worlds", MAP_NAME + ".world"),
                ((MAP_NAME + "_outer", outer),
                 (MAP_NAME + "_s_walls", s_walls),
                 (MAP_NAME + "_goal_u", u_walls)))
    image = base.rasterize(all_boxes)
    base.write_pgm(os.path.join(map_dir, MAP_NAME + ".pgm"), image)
    base.write_map_yaml(os.path.join(map_dir, MAP_NAME + ".yaml"), MAP_NAME + ".pgm")

    print("world: %s" % os.path.join(pkg, "worlds", MAP_NAME + ".world"))
    print("map:   %s/%s.{pgm,yaml}" % (map_dir, MAP_NAME))
    print("room:  x [%.2f, %.2f], y [%.2f, %.2f]"
          % (-ROOM_HALF_X, ROOM_HALF_X, ROOM_BOTTOM_Y, ROOM_TOP_Y))
    print("start: (%.2f, %.2f), goal: (%.2f, %.2f)"
          % (FLOOD_SEED[0], FLOOD_SEED[1], GOAL[0], GOAL[1]))
    print("S hooks shifted by %+.2f m; corridor under the bowl: %.2f m free"
          % (S_SHIFT_Y,
             (U_ARC_Y - U_RADIUS - 0.5 * WALL_THICK) - S_TOP_SURFACE))
    print("spawn pocket %.2f m tall, robot centred with %.3f m to each side"
          % (SPAWN_POCKET_H, 0.5 * SPAWN_POCKET_H))
    print("required clearance along every corridor: %.3f m "
          "(inflation %.2f + %.2f margin + half footprint %.3f)"
          % (PASSAGE_HALF_FREE - CURVE_RASTER_ALLOWANCE, COSTMAP_INFLATION,
             ROBOT_TO_COSTMAP_CLEARANCE, 0.5 * ROBOT_WIDTH))


if __name__ == "__main__":
    main()
