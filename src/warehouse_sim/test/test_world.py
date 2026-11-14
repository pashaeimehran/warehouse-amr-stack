"""Checks of the generated warehouse: reproducibility, decodable tags, and a
layout the robot can actually drive (clear goals, wide enough aisles)."""

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import pytest
import yaml
from apriltag import apriltag

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generate_world",
                                              PACKAGE / "scripts" / "generate_world.py")
gen = importlib.util.module_from_spec(spec)
sys.modules["generate_world"] = gen   # dataclasses look the module up there
spec.loader.exec_module(gen)

ROBOT_RADIUS = 0.47   # circumscribed radius of the base footprint incl. wheels, m


def test_committed_files_match_generator(tmp_path):
    for path in gen.generate(tmp_path):
        committed = PACKAGE / path.relative_to(tmp_path)
        assert committed.exists(), f"{committed} missing: run scripts/generate_world.py"
        assert committed.read_bytes() == path.read_bytes(), f"{committed} is stale"


def test_world_is_valid_xml_with_required_systems():
    root = ET.parse(PACKAGE / "worlds" / "warehouse.sdf").getroot()
    plugins = {p.get("name") for p in root.iter("plugin")}
    for system in ("Physics", "Sensors", "Imu", "Contact", "SceneBroadcaster", "UserCommands"):
        assert f"gz::sim::systems::{system}" in plugins


@pytest.mark.parametrize("tag", gen.tags(), ids=lambda t: f"id{t.tag_id}")
def test_tag_texture_decodes_to_its_id(tag):
    png = (PACKAGE / "models" / f"{gen.TAG_FAMILY}_{tag.tag_id}" / "materials" / "textures"
           / f"{gen.TAG_FAMILY}_{tag.tag_id}.png")
    image = cv2.imread(str(png), cv2.IMREAD_GRAYSCALE)
    # Pad with white like the plate's surroundings, then detect.
    image = cv2.copyMakeBorder(image, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    detections = apriltag("tag36h11").detect(image)
    assert [d["id"] for d in detections] == [tag.tag_id]


def test_tag_map_matches_world():
    tag_map = yaml.safe_load((PACKAGE / "config" / "tag_map.yaml").read_text())
    assert tag_map["family"] == gen.TAG_FAMILY
    assert tag_map["size"] == pytest.approx(0.2)
    assert [t["id"] for t in tag_map["tags"]] == list(range(8))
    ids = [t.tag_id for t in gen.tags()]
    assert len(ids) == len(set(ids))


def distance_to_shelves(x, y):
    """Distance from (x, y) to the nearest shelf row rectangle (0 inside)."""
    best = math.inf
    half = gen.SHELF_DEPTH / 2
    for row_y in gen.SHELF_ROWS_Y:
        dx = max(gen.SHELF_X[0] - gen.END_PANEL_THICKNESS - x, 0.0,
                 x - gen.SHELF_X[1] - gen.END_PANEL_THICKNESS)
        dy = max(row_y - half - y, 0.0, y - row_y - half)
        best = min(best, math.hypot(dx, dy))
    return best


@pytest.mark.parametrize("name", list(gen.STATIONS))
def test_station_goals_leave_the_robot_clear(name):
    x, y, _ = gen.STATIONS[name]
    assert distance_to_shelves(x, y) > ROBOT_RADIUS + 0.2
    (x0, x1), (y0, y1) = gen.HALL_X, gen.HALL_Y
    assert x0 + ROBOT_RADIUS < x < x1 - ROBOT_RADIUS
    assert y0 + ROBOT_RADIUS < y < y1 - ROBOT_RADIUS
    tx, ty, sx, sy, _ = gen.DROP_OFF_TABLE
    assert max(abs(x - tx) - sx / 2, abs(y - ty) - sy / 2) > ROBOT_RADIUS


def test_aisles_fit_the_robot_with_margin():
    rows = sorted(gen.SHELF_ROWS_Y)
    widths = [(b - a) - gen.SHELF_DEPTH for a, b in zip(rows, rows[1:])]
    assert min(widths) >= 2 * ROBOT_RADIUS + 1.0   # 2.2 m aisles for a 0.94 m robot


def test_actor_loop_is_closed_and_clear():
    waypoints = gen.actor_waypoints()
    assert waypoints[0][1:3] == waypoints[-1][1:3]          # closed loop
    assert [w[0] for w in waypoints] == sorted(w[0] for w in waypoints)
    for _, x, y, _ in waypoints:
        assert distance_to_shelves(x, y) > 0.5
        # Never in the dock to Station A corridor (aisle y = 0, robot there in tests).
        assert abs(y) > 1.5
    root = ET.parse(PACKAGE / "worlds" / "warehouse.sdf").getroot()
    assert root.find(".//actor[@name='walking_person']/skin/filename").text == \
        "model://actor/meshes/walk.dae"
