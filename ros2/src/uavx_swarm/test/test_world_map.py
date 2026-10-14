"""Unit tests for the 3-D map: planning, radio shadowing, world generation (no simulator)."""
import math
import os
import random

import pytest

from uavx_swarm.radio_model import RadioParams, predict_pdr
from uavx_swarm.world_map import WorldMap

HERE = os.path.dirname(os.path.abspath(__file__))
MAP = os.path.join(HERE, "..", "..", "uavx_gazebo", "maps", "quake_town.yaml")
WORLD = os.path.join(HERE, "..", "..", "uavx_gazebo", "worlds", "uavx_disaster.sdf")


@pytest.fixture(scope="module")
def town():
    return WorldMap.load(MAP)


def _path_clear(m, start, path, alt, clearance):
    pts = [start] + path
    for a, b in zip(pts, pts[1:]):
        n = max(2, int(math.dist(a, b)))
        for i in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n
            for o in m.obstacles:
                if o.height + clearance >= alt and o.horizontal_gap(x, y) < clearance - 1.0:
                    return False
    return True


def test_map_loads(town):
    assert town.name == "quake_town"
    assert len(town.obstacles) >= 10
    assert {o.id for o in town.obstacles} >= {"B2", "B5", "T1", "R1"}


def test_contains_and_clearance(town):
    b5 = next(o for o in town.obstacles if o.id == "B5")
    assert b5.contains(b5.cx, b5.cy, 10.0)
    assert not b5.contains(b5.cx, b5.cy, b5.height + 1.0)
    assert town.clearance(b5.cx, b5.cy, b5.height + 5.0) == pytest.approx(5.0, abs=0.01)


@pytest.mark.parametrize("alt", [25.0, 45.0, 65.0])
def test_planned_paths_avoid_obstacles(town, alt):
    rng = random.Random(3)
    for _ in range(25):
        start = town.random_free_point(rng, (-20, -100, 240, 100), margin=12.0)
        goal = town.random_free_point(rng, (-20, -100, 240, 100), margin=12.0)
        path = town.plan(start, goal, alt, clearance=10.0)
        assert path, "planner returned nothing"
        assert math.dist(path[-1], goal) < 6.0
        assert _path_clear(town, tuple(start), path, alt, 10.0)


def test_low_level_detours_high_level_flies_straight(town):
    # across the town centre: blocked at 25 m (B2/B3/B4), clear above the 46 m tower block
    start, goal = (60.0, -5.0), (230.0, 20.0)
    low = town.plan(start, goal, 25.0, 10.0)
    high = town.plan(start, goal, 70.0, 10.0)
    assert len(high) == 1
    assert len(low) >= 2


def test_buildings_shadow_the_radio(town):
    p = RadioParams()
    b2 = next(o for o in town.obstacles if o.id == "B2")
    behind_low = (b2.cx + 20.0, b2.cy, 15.0)
    in_front = (b2.cx - 20.0, b2.cy, 15.0)
    assert town.los_loss_db(in_front, behind_low) >= b2.attenuation_db
    assert predict_pdr(p, in_front, behind_low, world_map=town) < 0.05
    assert predict_pdr(p, in_front, behind_low) > 0.9          # same geometry, no map
    over = (b2.cx + 20.0, b2.cy, b2.height + 20.0)
    assert town.los_loss_db((b2.cx - 20.0, b2.cy, b2.height + 20.0), over) == 0.0


def test_world_generation(town, tmp_path):
    out = town.write_world(WORLD, str(tmp_path / "world.sdf"))
    text = open(out, encoding="utf-8").read()
    assert text.count("<model name=\"map_") == len(town.obstacles)
    assert "name=\"rubble\"" not in text
    assert text.strip().endswith("</sdf>")
