#!/usr/bin/env python3
"""
Generate the `map_extra` environment: same room as maze_19, but the central
U-shaped trap is replaced by a *curved* U (a semicircular arc closed at the
bottom, with two straight arms so the mouth stays the same width as maze_19).

Writes, from a single geometry definition so they can never drift apart:
  - sim_env/worlds/map_extra.world          (Gazebo)
  - sim_env/maps/map_extra/map_extra.pgm    (map_server occupancy grid)
  - sim_env/maps/map_extra/map_extra.yaml

Gazebo has no arc primitive, so the curve is approximated by ARC_SEGMENTS
short boxes rotated along the tangent, each slightly longer than the chord
so neighbouring segments overlap and leave no gap for the laser to slip
through.

Usage:  python3 generate_map_extra.py
"""

import math
import os

# ----------------------------------------------------------------------------
# geometry parameters -- tweak these to reshape the map, then re-run
# ----------------------------------------------------------------------------
WALL_THICK = 0.15
WALL_HEIGHT = 2.5

# outer room (kept identical to maze_19 so planner results stay comparable)
ROOM_HALF_X = 6.175
ROOM_HALF_Y = 4.55
ROOM_LEN_X = 12.5
ROOM_LEN_Y = 9.25

# curved U
ARC_CENTER = (0.0, 0.0)   # centre of the circular part
ARC_RADIUS = 1.25         # centreline radius
ARC_START_DEG = 180.0     # sweep start; 180 -> 360 is the bottom half
ARC_END_DEG = 360.0
ARC_SEGMENTS = 30         # more segments -> smoother curve
ARM_TOP_Y = 1.75          # straight arm length (was 1.20 m, then 1.32, 1.35, 1.5, 2.0)

# free-standing boxes along the side walls (same layout as maze_19)
UNIT_BOXES = [
    (-5.41379, 3.95004),
    (-5.41583, -3.92211),
    (5.44079, -3.81584),
    (5.51042, 3.71029),
    (-5.46872, 0.35278),
    (5.68215, -0.012164),
]
UNIT_BOX_SIZE = 1.0

# occupancy grid
MAP_W = MAP_H = 384
RESOLUTION = 0.05
ORIGIN = (-10.0, -10.0)
FLOOD_SEED = (0.0, -3.0)  # robot spawn: must be inside the room

# pixel values expected by map_server with the thresholds written below
PIX_OCCUPIED = 0
PIX_FREE = 254
PIX_UNKNOWN = 205


def build_boxes():
    """Every obstacle as (cx, cy, length_x, length_y, yaw, name).

    `length_x`/`length_y` are the box dimensions in its own frame; `yaw`
    rotates it into the world frame.
    """
    walls = [
        (0.0, -ROOM_HALF_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_bottom"),
        (0.0, ROOM_HALF_Y, ROOM_LEN_X, WALL_THICK, 0.0, "Wall_top"),
        (-ROOM_HALF_X, 0.0, ROOM_LEN_Y, WALL_THICK, math.pi / 2, "Wall_left"),
        (ROOM_HALF_X, 0.0, ROOM_LEN_Y, WALL_THICK, math.pi / 2, "Wall_right"),
    ]

    arc = []
    cx, cy = ARC_CENTER
    sweep = math.radians(ARC_END_DEG - ARC_START_DEG)
    step = sweep / ARC_SEGMENTS
    # chord of one segment; overlap by 25% so the seams stay watertight
    seg_len = 2 * ARC_RADIUS * math.sin(step / 2) * 1.25
    for i in range(ARC_SEGMENTS):
        theta = math.radians(ARC_START_DEG) + (i + 0.5) * step
        x = cx + ARC_RADIUS * math.cos(theta)
        y = cy + ARC_RADIUS * math.sin(theta)
        # the box's long axis follows the tangent
        arc.append((x, y, seg_len, WALL_THICK, theta + math.pi / 2,
                    "Arc_%02d" % i))

    # straight arms from the arc ends up to the mouth
    arm_len = ARM_TOP_Y - cy
    arms = []
    if arm_len > 0:
        arm_cy = cy + arm_len / 2
        for sign, name in ((-1, "Arm_left"), (1, "Arm_right")):
            arms.append((cx + sign * ARC_RADIUS, arm_cy, arm_len, WALL_THICK,
                         math.pi / 2, name))

    return walls, arc + arms


# ----------------------------------------------------------------------------
# Gazebo world
# ----------------------------------------------------------------------------
WORLD_HEAD = """<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""

WORLD_TAIL = """    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0 -1.5 24 -1e-06 1.5356 1.5602</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
"""


def wall_link(name, cx, cy, lx, ly, yaw):
    size = "%g %g %g" % (lx, ly, WALL_HEIGHT)
    half_h = WALL_HEIGHT / 2
    return """      <link name='{name}'>
        <collision name='{name}_Collision'>
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>
          <pose>0 0 {half_h} 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='{name}_Visual'>
          <pose>0 0 {half_h} 0 -0 0</pose>
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>{cx:.6f} {cy:.6f} 0 0 -0 {yaw:.6f}</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
""".format(name=name, size=size, half_h=half_h, cx=cx, cy=cy, yaw=yaw)


def unit_box_model(name, x, y):
    s = UNIT_BOX_SIZE
    return """    <model name='{name}'>
      <pose>{x:.6f} {y:.6f} {hz:g} 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{s:g} {s:g} {s:g}</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{s:g} {s:g} {s:g}</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
""".format(name=name, x=x, y=y, hz=s / 2, s=s)


def state_block(models):
    """The <state> snapshot Gazebo writes on 'Save World As'.

    Every other world in this package carries one, so map_extra does too.
    The wall models sit at the origin, so each of their links' world pose is
    just its own pose; the unit boxes carry their own model pose.
    """
    out = ["""    <state world_name='default'>
      <sim_time>0 0</sim_time>
      <real_time>0 0</real_time>
      <wall_time>0 0</wall_time>
      <iterations>0</iterations>
"""]
    for model_name, model_pose, links in models:
        out.append("      <model name='%s'>\n" % model_name)
        out.append("        <pose>%s</pose>\n"
                   "        <scale>1 1 1</scale>\n" % model_pose)
        for link_name, pose in links:
            out.append("""        <link name='{ln}'>
          <pose>{pose}</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
""".format(ln=link_name, pose=pose))
        out.append("      </model>\n")
    out.append("""      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
""")
    return "".join(out)


def write_world(path, walls, curve):
    out = [WORLD_HEAD]
    # (model name, model pose, [(link name, world pose), ...]) -> <state>
    models = [("ground_plane", "0 0 0 0 -0 0", [("link", "0 0 0 0 -0 0")])]

    for model_name, boxes in (("map_extra", walls), ("U_curved", curve)):
        out.append("    <model name='%s'>\n      <pose>0 0 0 0 -0 0</pose>\n"
                   % model_name)
        links = []
        for cx, cy, lx, ly, yaw, name in boxes:
            out.append(wall_link(name, cx, cy, lx, ly, yaw))
            links.append((name, "%.6f %.6f 0 0 -0 %.6f" % (cx, cy, yaw)))
        out.append("      <static>1</static>\n    </model>\n")
        models.append((model_name, "0 0 0 0 -0 0", links))

    for i, (x, y) in enumerate(UNIT_BOXES):
        name = "unit_box" if i == 0 else "unit_box_%d" % (i - 1)
        out.append(unit_box_model(name, x, y))
        pose = "%.6f %.6f %g 0 -0 0" % (x, y, UNIT_BOX_SIZE / 2)
        models.append((name, pose, [("link", pose)]))

    out.append(state_block(models))
    out.append(WORLD_TAIL)
    with open(path, "w") as f:
        f.write("".join(out))


# ----------------------------------------------------------------------------
# occupancy grid
# ----------------------------------------------------------------------------
def rasterize(boxes):
    """Return a MAP_H x MAP_W numpy array of pixel values."""
    import numpy as np

    ox, oy = ORIGIN
    # world coordinate of every cell centre
    xs = ox + (np.arange(MAP_W) + 0.5) * RESOLUTION
    ys = oy + (np.arange(MAP_H) + 0.5) * RESOLUTION
    gx, gy = np.meshgrid(xs, ys)

    occupied = np.zeros((MAP_H, MAP_W), dtype=bool)
    for cx, cy, lx, ly, yaw, _ in boxes:
        dx, dy = gx - cx, gy - cy
        c, s = math.cos(-yaw), math.sin(-yaw)
        # into the box's own frame
        u = dx * c - dy * s
        v = dx * s + dy * c
        occupied |= (np.abs(u) <= lx / 2) & (np.abs(v) <= ly / 2)

    for x, y in UNIT_BOXES:
        h = UNIT_BOX_SIZE / 2
        occupied |= ((np.abs(gx - x) <= h) & (np.abs(gy - y) <= h))

    # flood fill the reachable free space from the robot spawn; anything the
    # robot could never see stays unknown, the way map_saver would leave it
    reachable = np.zeros_like(occupied)
    seed_col = int((FLOOD_SEED[0] - ox) / RESOLUTION)
    seed_row = int((FLOOD_SEED[1] - oy) / RESOLUTION)
    if occupied[seed_row, seed_col]:
        raise SystemExit("flood seed %s lands on an obstacle" % (FLOOD_SEED,))

    stack = [(seed_row, seed_col)]
    reachable[seed_row, seed_col] = True
    while stack:
        r, c = stack.pop()
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= nr < MAP_H and 0 <= nc < MAP_W:
                if not occupied[nr, nc] and not reachable[nr, nc]:
                    reachable[nr, nc] = True
                    stack.append((nr, nc))

    img = np.full((MAP_H, MAP_W), PIX_UNKNOWN, dtype=np.uint8)
    img[reachable] = PIX_FREE
    img[occupied] = PIX_OCCUPIED
    # the grid's row 0 is the top of the image but the highest y in the world
    return np.flipud(img)


def write_pgm(path, img):
    with open(path, "wb") as f:
        f.write(b"P5\n")
        f.write(b"# CREATOR: generate_map_extra.py %.3f m/pix\n" % RESOLUTION)
        f.write(b"%d %d\n255\n" % (MAP_W, MAP_H))
        f.write(img.tobytes())


def write_map_yaml(path, pgm_name):
    with open(path, "w") as f:
        f.write("image: %s\n" % pgm_name)
        f.write("resolution: %f\n" % RESOLUTION)
        f.write("origin: [%f, %f, 0.000000]\n" % ORIGIN)
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        f.write("\n")  # the other map yamls end with a blank line


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)                       # .../sim_env
    world_path = os.path.join(pkg, "worlds", "map_extra.world")
    map_dir = os.path.join(pkg, "maps", "map_extra")
    os.makedirs(map_dir, exist_ok=True)

    walls, curve = build_boxes()
    write_world(world_path, walls, curve)
    img = rasterize(walls + curve)
    write_pgm(os.path.join(map_dir, "map_extra.pgm"), img)
    write_map_yaml(os.path.join(map_dir, "map_extra.yaml"), "map_extra.pgm")

    print("world : %s" % world_path)
    print("map   : %s/map_extra.{pgm,yaml}" % map_dir)
    print("curved U: centre %s  radius %.2f  mouth y=%.2f  arc %g deg in %d segments"
          % (ARC_CENTER, ARC_RADIUS, ARM_TOP_Y, ARC_END_DEG - ARC_START_DEG,
             ARC_SEGMENTS))


if __name__ == "__main__":
    main()
