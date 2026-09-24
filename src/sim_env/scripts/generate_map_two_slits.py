#!/usr/bin/env python3
"""Generate `map_two_slits`: a room whose only obstacles are two narrow slits.

Two full-width walls divide the room into three bands.  Each wall has a single
gap, and the two gaps sit on opposite sides, so the robot has to zig-zag:

    +-----------------------------------+   <- goal band
    |                                   |
    |################        ###########|   <- wall 2, slit on the RIGHT
    |                                   |
    |###########        ################|   <- wall 1, slit on the LEFT
    |                                   |
    +-----------------------------------+   <- start band

Start and goal both sit on x = 0, so the straight line between them crosses
both walls: the same heuristic deception as map_s_trap, but with no concave
pocket anywhere.  That is the point of this map -- it separates "deceptive
straight line" from "trap that encloses the goal".

Every gap and every band is one passage wide, using the same rule as
map_s_trap: the robot footprint keeps ROBOT_TO_COSTMAP_CLEARANCE free of the
0.40 m global costmap inflation on both sides.

Generated files:
  sim_env/worlds/map_two_slits.world
  sim_env/maps/map_two_slits/map_two_slits.pgm
  sim_env/maps/map_two_slits/map_two_slits.yaml
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_map_extra as base          # noqa: E402  (rasterizer + PGM/world I/O)
import generate_map_s_trap as strap        # noqa: E402  (wall_link / write_world)


MAP_NAME = "map_two_slits"
WALL_THICK = 0.15

# Same passage rule as map_s_trap so the two maps stay comparable.
ROBOT_WIDTH = 0.31
COSTMAP_INFLATION = 0.40
ROBOT_TO_COSTMAP_CLEARANCE = 0.05
RASTER_ALLOWANCE = 0.03
PASSAGE_HALF_GAP = (0.5 * WALL_THICK + 0.5 * ROBOT_WIDTH +
                    COSTMAP_INFLATION + ROBOT_TO_COSTMAP_CLEARANCE)
# wall surface -> path centreline; a distance transform of the .pgm must report
# at least this much everywhere the robot drives.
PASSAGE_HALF_FREE = PASSAGE_HALF_GAP - 0.5 * WALL_THICK + RASTER_ALLOWANCE

ROOM_HALF_X = 4.60
ROOM_BOTTOM_Y = -4.60
# Free height of each of the three bands.  Wide enough for the robot to turn
# around, so the only genuinely tight places are the two slits.
BAND_H = 2.20
# Half the free width of a slit; 2 * PASSAGE_HALF_FREE is one passage.
SLIT_HALF_FREE = PASSAGE_HALF_FREE
# Measured RRT-Cut success (closed-loop simulation, 20 runs, start/goal on the
# centre line) against the lateral offset of the slits:
#     +-0.60 -> 19/20   but the gaps then straddle x = 0, so the straight
#                       start->goal line is not blocked at all
#     +-0.80 -> 15/20   straight line still blocked by BOTH walls
#     +-1.00 -> 10/20
#     +-1.20 ->  6/20
#     +-2.40 ->  0/20
# +-0.80 is the largest offset that keeps the heuristic deception intact while
# a greedy cut-based planner can still solve the map.
SLIT1_X = -0.80   # lower wall: gap left of centre
SLIT2_X = 0.80    # upper wall: gap right of centre

# Lateral placement of start and goal.  Measured behaviour of a greedy planner
# on this map depends almost entirely on how far each slit sits from the
# straight start->goal line, so these two numbers -- together with SLIT*_X --
# set the difficulty.  0.0 / 0.0 puts both on the centre line, which makes the
# straight line cross both walls and leaves each slit SLIT*_X off the line.
START_X = 0.0
GOAL_X = 0.0

MAP_W = MAP_H = 384
RESOLUTION = 0.05
ORIGIN = (-10.0, -10.0)

FLOOR_SURFACE = ROOM_BOTTOM_Y + 0.5 * WALL_THICK
WALL1_Y = FLOOR_SURFACE + BAND_H + 0.5 * WALL_THICK
WALL2_Y = WALL1_Y + 0.5 * WALL_THICK + BAND_H + 0.5 * WALL_THICK
CEILING_SURFACE = WALL2_Y + 0.5 * WALL_THICK + BAND_H
ROOM_TOP_Y = CEILING_SURFACE + 0.5 * WALL_THICK

ROOM_CENTER_Y = 0.5 * (ROOM_TOP_Y + ROOM_BOTTOM_Y)
ROOM_LEN_X = 2.0 * ROOM_HALF_X + WALL_THICK
ROOM_LEN_Y = (ROOM_TOP_Y - ROOM_BOTTOM_Y) + WALL_THICK

# Each at the centre of its own band.
START = (START_X, FLOOR_SURFACE + 0.5 * BAND_H)
GOAL = (GOAL_X, WALL2_Y + 0.5 * WALL_THICK + 0.5 * BAND_H)
FLOOD_SEED = START

import math  # noqa: E402


def _line_offset(px, py):
    """Perpendicular distance from a slit centre to the start->goal line."""
    ax, ay = START
    bx, by = GOAL
    dx, dy = bx - ax, by - ay
    return abs(dy * px - dx * py + bx * ay - by * ax) / math.hypot(dx, dy)


def h_wall(x0, x1, y, name):
    """One horizontal wall box spanning x0..x1 at height y."""
    return ((x0 + x1) / 2.0, y, x1 - x0, WALL_THICK, 0.0, name)


def slit_wall(y, slit_x, prefix):
    """A full-width wall with a single gap centred on slit_x."""
    inner_left = -ROOM_HALF_X + 0.5 * WALL_THICK
    inner_right = ROOM_HALF_X - 0.5 * WALL_THICK
    return [
        h_wall(inner_left, slit_x - SLIT_HALF_FREE, y, prefix + "_left"),
        h_wall(slit_x + SLIT_HALF_FREE, inner_right, y, prefix + "_right"),
    ]


def build_boxes():
    outer = [
        (0.0, ROOM_BOTTOM_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_bottom"),
        (0.0, ROOM_TOP_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_top"),
        (-ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_left"),
        (ROOM_HALF_X, ROOM_CENTER_Y, ROOM_LEN_Y, WALL_THICK,
         math.pi / 2.0, "Wall_right"),
    ]
    slits = slit_wall(WALL1_Y, SLIT1_X, "Slit1") + \
        slit_wall(WALL2_Y, SLIT2_X, "Slit2")
    return outer, slits


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    map_dir = os.path.join(pkg, "maps", MAP_NAME)
    os.makedirs(map_dir, exist_ok=True)

    outer, slits = build_boxes()

    base.MAP_W = base.MAP_H = MAP_W
    base.RESOLUTION = RESOLUTION
    base.ORIGIN = ORIGIN
    base.FLOOD_SEED = FLOOD_SEED
    base.UNIT_BOXES = []
    strap.WALL_THICK = WALL_THICK

    strap.write_world(os.path.join(pkg, "worlds", MAP_NAME + ".world"),
                      ((MAP_NAME + "_outer", outer),
                       (MAP_NAME + "_slits", slits)))
    image = base.rasterize(outer + slits)
    base.write_pgm(os.path.join(map_dir, MAP_NAME + ".pgm"), image)
    base.write_map_yaml(os.path.join(map_dir, MAP_NAME + ".yaml"),
                        MAP_NAME + ".pgm")

    print("world: %s" % os.path.join(pkg, "worlds", MAP_NAME + ".world"))
    print("map:   %s/%s.{pgm,yaml}" % (map_dir, MAP_NAME))
    print("room:  x [%.2f, %.2f], y [%.2f, %.2f]"
          % (-ROOM_HALF_X, ROOM_HALF_X, ROOM_BOTTOM_Y, ROOM_TOP_Y))
    print("walls: y = %.2f (slit at x = %+.2f), y = %.2f (slit at x = %+.2f)"
          % (WALL1_Y, SLIT1_X, WALL2_Y, SLIT2_X))
    print("slit free width: %.2f m; band free height: %.2f m"
          % (2.0 * SLIT_HALF_FREE, BAND_H))
    print("start: (%.2f, %.2f), goal: (%.2f, %.2f)"
          % (START[0], START[1], GOAL[0], GOAL[1]))
    print("slit offset from the straight start->goal line: %.2f m / %.2f m"
          % (_line_offset(SLIT1_X, WALL1_Y), _line_offset(SLIT2_X, WALL2_Y)))
    print("required clearance everywhere: %.3f m"
          % (PASSAGE_HALF_FREE - RASTER_ALLOWANCE))


if __name__ == "__main__":
    main()
