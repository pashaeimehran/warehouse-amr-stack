#!/usr/bin/env python3
"""Generates the warehouse world, its AprilTag models and the landmark map.

One source of truth for the layout: the SDF world, the tag textures and
`config/tag_map.yaml` (the landmark map of the localization stack, v0.4) are
all written from the constants below, so they cannot drift apart. The output
is deterministic; `test/test_world.py` regenerates it and fails if the
committed files differ.

Coordinates: the Gazebo world frame is the map frame (REP 105), with the
origin at the dock. x runs along the aisles, y across them, z up; metres.

Usage:
    python3 generate_world.py [--package-dir DIR]   # default: this package
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

# --- Layout ---------------------------------------------------------------

HALL_X = (-3.0, 22.0)          # inner wall faces
HALL_Y = (-8.0, 8.0)
WALL_HEIGHT = 3.0
WALL_THICKNESS = 0.2

SHELF_ROWS_Y = (4.5, 1.5, -1.5, -4.5)   # row centre lines; aisles at y = 3, 0, -3
SHELF_X = (4.0, 16.0)                     # row extent along x
SHELF_UNIT_LENGTH = 4.0
SHELF_DEPTH = 0.8
SHELF_HEIGHT = 2.0
SHELF_BASE_HEIGHT = 0.45   # solid lower cabinet: the 2D LiDAR (z ~ 0.34 m) sees it
SHELF_BOARDS_Z = (0.45, 1.05, 1.65)
END_PANEL_THICKNESS = 0.02

TAG_FAMILY = "tag36h11"
TAG_PLATE = 0.25            # plate edge incl. one-cell white quiet zone (10 cells)
TAG_SIZE = TAG_PLATE * 8 / 10   # black square edge, the size apriltag_ros needs
TAG_CENTER_Z = 0.60

KLT = (0.40, 0.30, 0.22)    # KLT box (VDA 4280 size class 4322 approx.), m

STATIONS = {
    # name: (x, y, yaw) of the robot's goal pose
    "dock": (0.0, 0.0, 0.0),
    "station_a": (8.0, 0.0, 0.0),
    "station_b": (12.0, -3.0, 0.0),
    "drop_off": (19.0, 4.0, 0.0),
}
DROP_OFF_TABLE = (20.3, 4.0, 1.0, 0.8, 0.75)   # x, y, size x, size y, height

# Walking person (Gazebo actor, mesh from the CI and dev images, see
# docker/fetch_sim_assets.sh). Loops along the outer aisle y = -6, away from the
# dock to Station A corridor; scenario-specific crossings are scripted per scenario.
ACTOR_PATH = ((4.0, -6.0), (14.0, -6.0))
ACTOR_SPEED = 1.0           # m/s
ACTOR_TURN_TIME = 1.0       # s to turn around at each end
ACTOR_Z = 1.0               # pose height of the walk.dae skeleton root


@dataclass(frozen=True)
class Tag:
    tag_id: int
    x: float
    y: float
    yaw: float   # rotation about z of the tag's outward normal (+x of the tag model)


def tags() -> list[Tag]:
    """One tag on each end panel of every shelf row: west ends face -x, east ends +x."""
    out = []
    next_id = 0
    for row_y in SHELF_ROWS_Y:
        out.append(Tag(next_id, SHELF_X[0] - END_PANEL_THICKNESS - 0.003, row_y, math.pi))
        out.append(Tag(next_id + 1, SHELF_X[1] + END_PANEL_THICKNESS + 0.003, row_y, 0.0))
        next_id += 2
    return out


# --- SDF snippets ---------------------------------------------------------

def box_link(name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float,
             rgba: str, collide: bool = True) -> str:
    collision = (f"<collision name='{name}_c'><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}"
                 f"</size></box></geometry></collision>") if collide else ""
    return (
        f"<link name='{name}'><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>{collision}"
        f"<visual name='{name}_v'><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
        f"</geometry><material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>"
        f"</visual></link>"
    )


def walls() -> str:
    (x0, x1), (y0, y1) = HALL_X, HALL_Y
    t, h = WALL_THICKNESS, WALL_HEIGHT
    lx, ly = x1 - x0 + 2 * t, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    grey = "0.75 0.75 0.72 1"
    links = [
        box_link("north", cx, y1 + t / 2, h / 2, lx, t, h, grey),
        box_link("south", cx, y0 - t / 2, h / 2, lx, t, h, grey),
        box_link("west", x0 - t / 2, cy, h / 2, t, ly, h, grey),
        box_link("east", x1 + t / 2, cy, h / 2, t, ly, h, grey),
    ]
    return f"<model name='walls'><static>true</static>{''.join(links)}</model>"


def shelf_row(index: int, row_y: float) -> str:
    links = []
    blue, orange, grey = "0.15 0.3 0.6 1", "0.9 0.5 0.1 1", "0.6 0.6 0.6 1"
    length = SHELF_X[1] - SHELF_X[0]
    cx = (SHELF_X[0] + SHELF_X[1]) / 2
    # Solid lower cabinet and shelf boards along the whole row.
    links.append(box_link("cabinet", cx, row_y, SHELF_BASE_HEIGHT / 2, length, SHELF_DEPTH,
                          SHELF_BASE_HEIGHT, grey))
    for k, z in enumerate(SHELF_BOARDS_Z[1:], start=1):
        links.append(box_link(f"board{k}", cx, row_y, z, length, SHELF_DEPTH, 0.04, orange))
    # Uprights at every unit boundary, end panels at both ends.
    units = int(round(length / SHELF_UNIT_LENGTH))
    for u in range(units + 1):
        x = SHELF_X[0] + u * SHELF_UNIT_LENGTH
        for side in (-1, 1):
            y = row_y + side * (SHELF_DEPTH / 2 - 0.04)
            links.append(box_link(f"upright{u}_{'n' if side > 0 else 's'}", x, y,
                                  SHELF_HEIGHT / 2, 0.08, 0.08, SHELF_HEIGHT, blue))
    for name, x in (("end_w", SHELF_X[0] - END_PANEL_THICKNESS / 2),
                    ("end_e", SHELF_X[1] + END_PANEL_THICKNESS / 2)):
        links.append(box_link(name, x, row_y, SHELF_HEIGHT / 2, END_PANEL_THICKNESS,
                              SHELF_DEPTH, SHELF_HEIGHT, blue))
    return f"<model name='shelf_row_{index}'><static>true</static>{''.join(links)}</model>"


def klt_boxes() -> str:
    """KLT boxes on the boards, on both aisle-facing sides (pickable from v0.7 on)."""
    models = []
    n = 0
    for row_y in SHELF_ROWS_Y:
        for board_z in SHELF_BOARDS_Z:
            for x in (5.0, 8.0, 11.0, 14.0):
                for side in (-1, 1):
                    y = row_y + side * (SHELF_DEPTH / 2 - KLT[1] / 2 - 0.05)
                    z = board_z + 0.02 + KLT[2] / 2
                    models.append(
                        f"<model name='klt_{n:03d}'><static>true</static><pose>{x:.3f} {y:.3f} "
                        f"{z:.3f} 0 0 0</pose>"
                        + box_link("body", 0, 0, 0, KLT[0], KLT[1], KLT[2], "0.2 0.35 0.75 1")
                        + "</model>"
                    )
                    n += 1
    return "".join(models)


def floor_markings() -> str:
    """Visual-only floor patches at the dock and stations (no collision)."""
    colours = {"dock": "0.1 0.7 0.1 1", "station_a": "0.9 0.8 0.1 1",
               "station_b": "0.9 0.8 0.1 1", "drop_off": "0.8 0.1 0.1 1"}
    links = [box_link(name, x, y, 0.001, 0.9, 0.7, 0.002, colours[name], collide=False)
             for name, (x, y, _) in STATIONS.items()]
    return f"<model name='floor_markings'><static>true</static>{''.join(links)}</model>"


def drop_off_table() -> str:
    x, y, sx, sy, h = DROP_OFF_TABLE
    return ("<model name='drop_off_table'><static>true</static>"
            + box_link("top", x, y, h / 2, sx, sy, h, "0.4 0.3 0.2 1") + "</model>")


def actor_waypoints() -> list[tuple[float, float, float, float]]:
    """(time s, x, y, yaw) of the actor's closed walking loop."""
    (x0, y0), (x1, y1) = ACTOR_PATH
    leg = math.hypot(x1 - x0, y1 - y0) / ACTOR_SPEED
    out_yaw = math.atan2(y1 - y0, x1 - x0)
    back_yaw = math.atan2(y0 - y1, x0 - x1)
    t1 = leg
    t2 = t1 + ACTOR_TURN_TIME
    t3 = t2 + leg
    t4 = t3 + ACTOR_TURN_TIME
    return [(0.0, x0, y0, out_yaw), (t1, x1, y1, out_yaw), (t2, x1, y1, back_yaw),
            (t3, x0, y0, back_yaw), (t4, x0, y0, out_yaw)]


def actor() -> str:
    waypoints = "".join(
        f"<waypoint><time>{t:.2f}</time><pose>{x:.3f} {y:.3f} {ACTOR_Z:.3f} 0 0 {yaw:.6f}"
        f"</pose></waypoint>" for t, x, y, yaw in actor_waypoints())
    mesh = "model://actor/meshes/walk.dae"
    return (
        f"<actor name='walking_person'><skin><filename>{mesh}</filename><scale>1.0</scale>"
        f"</skin><animation name='walking'><filename>{mesh}</filename><scale>1.0</scale>"
        f"<interpolate_x>true</interpolate_x></animation><script><loop>true</loop>"
        f"<delay_start>0</delay_start><auto_start>true</auto_start>"
        f"<trajectory id='0' type='walking'>{waypoints}</trajectory></script></actor>"
    )


def tag_includes() -> str:
    return "".join(
        f"<include><name>tag_{t.tag_id}</name><uri>model://{TAG_FAMILY}_{t.tag_id}</uri>"
        f"<pose>{t.x:.3f} {t.y:.3f} {TAG_CENTER_Z:.3f} 0 0 {t.yaw:.6f}</pose></include>"
        for t in tags()
    )


WORLD_HEADER = """<?xml version="1.0"?>
<!-- GENERATED by scripts/generate_world.py; do not edit by hand. -->
<sdf version="1.9">
  <world name="warehouse">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 40</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 40</size></plane></geometry>
          <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def world_sdf(with_actor: bool = True) -> str:
    """The warehouse; with_actor=False gives the static variant used for mapping,
    so a walking person cannot leave traces in the saved map."""
    body = [walls()] + [shelf_row(i, y) for i, y in enumerate(SHELF_ROWS_Y)]
    body += [klt_boxes(), floor_markings(), drop_off_table(), tag_includes()]
    if with_actor:
        body.append(actor())
    return WORLD_HEADER + "\n".join(f"    {b}" for b in body) + "\n  </world>\n</sdf>\n"


# --- Tag models -----------------------------------------------------------

def tag_texture(tag_id: int):
    """10 x 10 cell tag36h11 image (8 x 8 marker plus white quiet zone), 40 px per cell."""
    import cv2  # imported lazily: only the generator needs OpenCV
    import numpy as np

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, tag_id, 8)
    cells = np.full((10, 10), 255, dtype=np.uint8)
    cells[1:9, 1:9] = marker
    return cv2.resize(cells, (400, 400), interpolation=cv2.INTER_NEAREST)


def tag_model_sdf(tag_id: int) -> str:
    name = f"{TAG_FAMILY}_{tag_id}"
    return f"""<?xml version="1.0"?>
<!-- GENERATED by scripts/generate_world.py -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <!-- Thin plate; the textured face is the +x face (the tag's outward normal). -->
      <visual name="tag">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>0.004 {TAG_PLATE} {TAG_PLATE}</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <albedo_map>materials/textures/{name}.png</albedo_map>
            <metalness>0.0</metalness>
            <roughness>1.0</roughness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def tag_model_config(tag_id: int) -> str:
    name = f"{TAG_FAMILY}_{tag_id}"
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>AprilTag {TAG_FAMILY} id {tag_id}, black square {TAG_SIZE:.3f} m (generated).</description>
</model>
"""


def tag_map_yaml() -> str:
    lines = [
        "# GENERATED by scripts/generate_world.py: AprilTag landmark map (map frame).",
        "# Pose of each tag centre; yaw is the direction of the tag's outward normal.",
        f"family: {TAG_FAMILY}",
        f"size: {TAG_SIZE:.3f}  # m, black square edge",
        "tags:",
    ]
    for t in tags():
        lines.append(f"  - {{id: {t.tag_id}, x: {t.x:.3f}, y: {t.y:.3f}, z: {TAG_CENTER_Z:.3f}, "
                     f"yaw: {t.yaw:.6f}}}")
    return "\n".join(lines) + "\n"


def stations_yaml() -> str:
    lines = ["# GENERATED by scripts/generate_world.py: named goal poses (map frame).",
             "stations:"]
    for name, (x, y, yaw) in STATIONS.items():
        lines.append(f"  {name}: {{x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.6f}}}")
    return "\n".join(lines) + "\n"


def generate(package_dir: Path) -> list[Path]:
    """Writes every generated file under package_dir; returns their paths."""
    import cv2

    written = []

    def write_text(path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        written.append(path)

    write_text(package_dir / "worlds" / "warehouse.sdf", world_sdf())
    write_text(package_dir / "worlds" / "warehouse_static.sdf", world_sdf(with_actor=False))
    write_text(package_dir / "config" / "tag_map.yaml", tag_map_yaml())
    write_text(package_dir / "config" / "stations.yaml", stations_yaml())
    for t in tags():
        model_dir = package_dir / "models" / f"{TAG_FAMILY}_{t.tag_id}"
        write_text(model_dir / "model.sdf", tag_model_sdf(t.tag_id))
        write_text(model_dir / "model.config", tag_model_config(t.tag_id))
        png = model_dir / "materials" / "textures" / f"{TAG_FAMILY}_{t.tag_id}.png"
        png.parent.mkdir(parents=True, exist_ok=True)
        # PNG compression level fixed so the bytes are reproducible.
        cv2.imwrite(str(png), tag_texture(t.tag_id), [cv2.IMWRITE_PNG_COMPRESSION, 9])
        written.append(png)
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    for path in generate(args.package_dir):
        print(path.relative_to(args.package_dir))


if __name__ == "__main__":
    main()
