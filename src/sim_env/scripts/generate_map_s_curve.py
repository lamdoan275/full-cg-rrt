#!/usr/bin/env python3
"""Generate `map_s_curve`: map_s_trap's curved S passage with the U trap removed.

The two curved hooks and the narrow S passage between them are taken verbatim
from generate_map_s_trap.py -- same Bezier centreline, same constant normal
offset, same downward shift -- so the corridor the robot has to thread is
byte-identical to map_s_trap's.  What is gone is the semicircular U bowl that
enclosed the goal.

That isolates one variable.  map_s_trap fails a greedy cut-based planner for
two independent reasons: the curved passage exits sideways, and the goal sits
inside a concave bowl.  This map keeps only the first.

Generated files:
  sim_env/worlds/map_s_curve.world
  sim_env/maps/map_s_curve/map_s_curve.pgm
  sim_env/maps/map_s_curve/map_s_curve.yaml
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_map_extra as base          # noqa: E402  (rasterizer + PGM/world I/O)
import generate_map_s_trap as strap        # noqa: E402  (S geometry + world writer)


MAP_NAME = "map_s_curve"

# Everything about the S passage is inherited, so the two maps stay comparable.
WALL_THICK = strap.WALL_THICK
PASSAGE_HALF_FREE = strap.PASSAGE_HALF_FREE
S_PASSAGE_HALF_GAP = strap.S_PASSAGE_HALF_GAP
S_BEZIER = strap.S_BEZIER
S_SHIFT_Y = strap.S_SHIFT_Y
S_TOP_SURFACE = strap.S_TOP_SURFACE
ROOM_HALF_X = strap.ROOM_HALF_X
ROOM_BOTTOM_Y = strap.ROOM_BOTTOM_Y
SPAWN_POCKET_H = strap.SPAWN_POCKET_H

# Lateral placement of start and goal.  The S passage leaves the spawn pocket
# on the RIGHT (the lower hook arm ends at x = +0.87) and re-enters the open
# room on the UPPER LEFT (the upper hook arm starts at x = -1.55).  Putting the
# start near the right-hand exit and the goal on the upper left makes the
# straight start->goal line run ALONG the corridor instead of across it.
#
# Measured RRT-Cut success (closed-loop simulation, 20 runs):
#     start x=0.0  goal x= 0.0   ->  1/20
#     start x=0.0  goal x=-3.0   ->  0/20
#     start x=0.0  goal x=+3.0   ->  0/20
#     start x=2.5  goal x= 0.0   ->  1/20
#     start x=2.5  goal x=-3.0   -> 20/20
#     start x=3.5  goal x=-3.0   -> 20/20
# The corridor itself is unchanged in every one of those runs; only the two
# end points move.  Alignment with the corridor direction is what decides it.
START_X = 2.5
GOAL_X = -3.0

START = (START_X, strap.FLOOD_SEED[1])

MAP_W = MAP_H = 384
RESOLUTION = 0.05
ORIGIN = (-10.0, -10.0)

# Free height of the open room above the S passage, where the goal sits.
# Nothing obstructs it -- the only hard part of this map is the S itself.
GOAL_BAND_H = 3.00
GOAL = (GOAL_X, S_TOP_SURFACE + 0.5 * GOAL_BAND_H)

ROOM_TOP_Y = S_TOP_SURFACE + GOAL_BAND_H + 0.5 * WALL_THICK
ROOM_CENTER_Y = 0.5 * (ROOM_TOP_Y + ROOM_BOTTOM_Y)
ROOM_LEN_X = 2.0 * ROOM_HALF_X + WALL_THICK
ROOM_LEN_Y = (ROOM_TOP_Y - ROOM_BOTTOM_Y) + WALL_THICK

FLOOD_SEED = START


def build_boxes():
    outer = [
        (0.0, ROOM_BOTTOM_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_bottom"),
        (0.0, ROOM_TOP_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_top"),
        (-ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_left"),
        (ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_right"),
    ]

    # Identical construction to map_s_trap: two opposing hooks whose inner
    # Bezier curves are constant normal offsets of one another.
    bezier = tuple((px, py + S_SHIFT_Y) for px, py in S_BEZIER)
    upper_pts, lower_pts, upper_curve, lower_curve = strap.bezier_parallel_walls(
        bezier, half_gap=S_PASSAGE_HALF_GAP, count=28)

    upper_start, upper_end = upper_pts[0], upper_pts[-1]
    lower_start, lower_end = lower_pts[0], lower_pts[-1]
    s_walls = [
        strap.segment(-ROOM_HALF_X + WALL_THICK / 2.0, lower_start[1],
                      lower_start[0], lower_start[1], "Left_hook_upper_arm"),
        *lower_curve,
        strap.segment(-ROOM_HALF_X + WALL_THICK / 2.0, lower_end[1],
                      lower_end[0], lower_end[1], "Left_hook_lower_arm"),
        strap.segment(upper_start[0], upper_start[1],
                      ROOM_HALF_X - WALL_THICK / 2.0, upper_start[1],
                      "Right_hook_upper_arm"),
        *upper_curve,
        strap.segment(upper_end[0], upper_end[1],
                      ROOM_HALF_X - WALL_THICK / 2.0, upper_end[1],
                      "Right_hook_lower_arm"),
    ]
    return outer, s_walls


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    map_dir = os.path.join(pkg, "maps", MAP_NAME)
    os.makedirs(map_dir, exist_ok=True)

    outer, s_walls = build_boxes()

    base.MAP_W = base.MAP_H = MAP_W
    base.RESOLUTION = RESOLUTION
    base.ORIGIN = ORIGIN
    base.FLOOD_SEED = FLOOD_SEED
    base.UNIT_BOXES = []
    strap.WALL_THICK = WALL_THICK

    strap.write_world(os.path.join(pkg, "worlds", MAP_NAME + ".world"),
                      ((MAP_NAME + "_outer", outer),
                       (MAP_NAME + "_s_walls", s_walls)))
    image = base.rasterize(outer + s_walls)
    base.write_pgm(os.path.join(map_dir, MAP_NAME + ".pgm"), image)
    base.write_map_yaml(os.path.join(map_dir, MAP_NAME + ".yaml"),
                        MAP_NAME + ".pgm")

    print("world: %s" % os.path.join(pkg, "worlds", MAP_NAME + ".world"))
    print("map:   %s/%s.{pgm,yaml}" % (map_dir, MAP_NAME))
    print("room:  x [%.2f, %.2f], y [%.2f, %.2f]"
          % (-ROOM_HALF_X, ROOM_HALF_X, ROOM_BOTTOM_Y, ROOM_TOP_Y))
    print("S passage: inherited from map_s_trap (shift %+.2f m), no U trap"
          % S_SHIFT_Y)
    print("start: (%.2f, %.2f), goal: (%.2f, %.2f)"
          % (START[0], START[1], GOAL[0], GOAL[1]))
    print("required clearance everywhere: %.3f m"
          % (PASSAGE_HALF_FREE - strap.CURVE_RASTER_ALLOWANCE))


if __name__ == "__main__":
    main()
