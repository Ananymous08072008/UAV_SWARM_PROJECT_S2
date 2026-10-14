"""3-D map of the disaster site - one description used by every part of the stack.

A map file (uavx_gazebo/maps/<name>.yaml) lists 3-D obstacles: standing and collapsed
buildings, towers, landslide ridges. The same objects are

  * written into the Gazebo world (sdf_models)            -> physics, camera, rangefinder
  * avoided by each UAV's companion (plan)                 -> A* on a 4 m grid at the UAV's
                                                             flight level, obstacles inflated
                                                             by the clearance, then shortcut
  * attenuating radio links that pass through them (los_loss_db) -> radio_net and the GCS
  * checked by the referee (contains / distance)           -> obstacle collisions, clearance

Frame: world ENU (origin = GCS), heights above ground. Tilted (collapsed) slabs are
planned and attenuated as their upright bounding box of the given height - conservative.
"""
from __future__ import annotations

import heapq
import math
import os
from dataclasses import dataclass, field

import yaml

GEOFENCE = (-200.0, 1100.0, -550.0, 550.0)


@dataclass
class Obstacle:
    id: str
    shape: str                      # "box" | "cylinder"
    cx: float
    cy: float
    height: float
    length: float = 0.0             # box: along its yaw axis
    width: float = 0.0
    radius: float = 0.0             # cylinder
    yaw: float = 0.0                # rad
    roll: float = 0.0               # visual tilt of collapsed slabs (rad)
    pitch: float = 0.0
    kind: str = "building"
    attenuation_db: float = 20.0
    color: tuple = (0.55, 0.53, 0.5)
    bound_r: float = field(init=False)

    def __post_init__(self):
        self.bound_r = self.radius if self.shape == "cylinder" else 0.5 * math.hypot(self.length, self.width)

    def _local(self, x: float, y: float):
        dx, dy = x - self.cx, y - self.cy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return dx * c + dy * s, -dx * s + dy * c

    def horizontal_gap(self, x: float, y: float) -> float:
        """Horizontal distance from (x, y) to the footprint (0 inside)."""
        if self.shape == "cylinder":
            return max(0.0, math.hypot(x - self.cx, y - self.cy) - self.radius)
        u, v = self._local(x, y)
        du = max(0.0, abs(u) - self.length / 2)
        dv = max(0.0, abs(v) - self.width / 2)
        return math.hypot(du, dv)

    def contains(self, x: float, y: float, z: float, margin: float = 0.0) -> bool:
        if z > self.height + margin or z < -margin:
            return False
        return self.horizontal_gap(x, y) <= margin

    def distance(self, x: float, y: float, z: float) -> float:
        return math.hypot(self.horizontal_gap(x, y), max(0.0, z - self.height))

    def near_segment(self, a, b, margin: float) -> bool:
        """Cheap reject: can the 2-D segment a-b come within bound_r + margin of the centre?"""
        ax, ay, bx, by = a[0], a[1], b[0], b[1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((self.cx - ax) * dx + (self.cy - ay) * dy) / L2))
        return math.hypot(ax + t * dx - self.cx, ay + t * dy - self.cy) <= self.bound_r + margin

    def sdf(self) -> str:
        r, g, b = self.color
        mat = f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"
        if self.shape == "cylinder":
            geo = f"<cylinder><radius>{self.radius}</radius><length>{self.height}</length></cylinder>"
        else:
            geo = f"<box><size>{self.length} {self.width} {self.height}</size></box>"
        pose = f"{self.cx} {self.cy} {self.height / 2:.3f} {self.roll:.4f} {self.pitch:.4f} {self.yaw:.4f}"
        return (f'    <model name="map_{self.id}"><static>true</static><pose>{pose}</pose>\n'
                f'      <link name="link"><collision name="c"><geometry>{geo}</geometry></collision>\n'
                f'      <visual name="v"><geometry>{geo}</geometry>{mat}</visual></link></model>\n')


class WorldMap:
    def __init__(self, data: dict, source: str = ""):
        self.name = data.get("name", os.path.splitext(os.path.basename(source))[0])
        self.description = data.get("description", "")
        self.source = source
        self.cell = float(data.get("planner_cell_m", 4.0))
        self.obstacles: list[Obstacle] = []
        for i, o in enumerate(data.get("obstacles", [])):
            shape = o.get("shape", "box")
            size = o.get("size", [10, 10, 10])
            self.obstacles.append(Obstacle(
                id=str(o.get("id", f"O{i + 1}")), shape=shape, cx=float(o["center"][0]), cy=float(o["center"][1]),
                height=float(o.get("height", size[2] if shape == "box" else 10.0)),
                length=float(size[0]) if shape == "box" else 0.0, width=float(size[1]) if shape == "box" else 0.0,
                radius=float(o.get("radius", 0.0)), yaw=math.radians(float(o.get("yaw_deg", 0.0))),
                roll=math.radians(float(o.get("roll_deg", 0.0))), pitch=math.radians(float(o.get("pitch_deg", 0.0))),
                kind=o.get("kind", "building"), attenuation_db=float(o.get("attenuation_db", 20.0)),
                color=tuple(o.get("color", (0.55, 0.53, 0.5)))))

    # ------------------------------------------------------------------ loading
    @classmethod
    def load(cls, name_or_path: str) -> "WorldMap":
        path = name_or_path
        if not os.path.isfile(path):
            from ament_index_python.packages import get_package_share_directory
            name = name_or_path if name_or_path.endswith(".yaml") else f"{name_or_path}.yaml"
            path = os.path.join(get_package_share_directory("uavx_gazebo"), "maps", name)
        with open(path, encoding="utf-8") as fh:
            return cls(yaml.safe_load(fh) or {}, path)

    @staticmethod
    def for_scenario(scenario: dict):
        return WorldMap.load(scenario["map"]) if scenario.get("map") else None

    # ------------------------------------------------------------------ queries
    def blocked(self, x: float, y: float, z: float, margin: float = 0.0, obstacles=None) -> bool:
        return any(o.contains(x, y, z, margin) for o in (obstacles or self.obstacles))

    def clearance(self, x: float, y: float, z: float) -> float:
        return min((o.distance(x, y, z) for o in self.obstacles), default=math.inf)

    def los_loss_db(self, a, b, step: float = 2.0) -> float:
        """Extra path loss (dB) of the 3-D radio path a-b: attenuation of every obstacle it crosses."""
        total = 0.0
        d = math.dist(a, b)
        n = max(2, int(d / step))
        for o in self.obstacles:
            if not o.near_segment(a, b, 0.0) or (a[2] > o.height and b[2] > o.height):
                continue
            for i in range(1, n):
                t = i / n
                if o.contains(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])):
                    total += o.attenuation_db
                    break
        return total

    def _relevant(self, alt: float, clearance: float):
        return [o for o in self.obstacles if o.height + clearance >= alt]

    def segment_clear(self, a, b, alt: float, clearance: float, obstacles=None) -> bool:
        obs = [o for o in (obstacles if obstacles is not None else self._relevant(alt, clearance))
               if o.near_segment(a, b, clearance)]
        if not obs:
            return True
        d = math.dist(a, b)
        n = max(2, int(d / 1.0))
        for i in range(n + 1):
            t = i / n
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            if any(o.horizontal_gap(x, y) <= clearance for o in obs):
                return False
        return True

    def plan(self, start, goal, alt: float, clearance: float = 10.0) -> list[tuple[float, float]]:
        """Obstacle-free waypoints from start to goal at height alt (start excluded).

        The last waypoint is the goal, or the nearest free point if the goal itself is too
        close to an obstacle. Falls back to the straight line if no path exists.
        """
        start, goal = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
        obs = self._relevant(alt, clearance)
        if not obs or self.segment_clear(start, goal, alt, clearance, obs):
            return [goal]
        c = self.cell
        x0 = max(GEOFENCE[0], min(start[0], goal[0]) - 120.0)
        x1 = min(GEOFENCE[1], max(start[0], goal[0]) + 120.0)
        y0 = max(GEOFENCE[2], min(start[1], goal[1]) - 120.0)
        y1 = min(GEOFENCE[3], max(start[1], goal[1]) + 120.0)
        nx, ny = int((x1 - x0) / c) + 1, int((y1 - y0) / c) + 1

        def centre(i, j):
            return (x0 + i * c, y0 + j * c)

        def cell_of(p):
            return (min(nx - 1, max(0, int(round((p[0] - x0) / c)))), min(ny - 1, max(0, int(round((p[1] - y0) / c)))))

        free_cache: dict = {}

        def free(i, j):
            k = (i, j)
            if k not in free_cache:
                x, y = centre(i, j)
                free_cache[k] = all(o.horizontal_gap(x, y) > clearance for o in obs)
            return free_cache[k]

        s, g = cell_of(start), cell_of(goal)
        if not free(*g):          # goal too close to an obstacle: nearest free cell
            best, best_d = None, math.inf
            for r in range(1, 12):
                for i in range(g[0] - r, g[0] + r + 1):
                    for j in range(g[1] - r, g[1] + r + 1):
                        if 0 <= i < nx and 0 <= j < ny and free(i, j):
                            d = math.dist(centre(i, j), goal)
                            if d < best_d:
                                best, best_d = (i, j), d
                if best:
                    break
            if best is None:
                return [goal]
            g = best
            goal = centre(*g)
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        openq = [(0.0, s)]
        g_cost = {s: 0.0}
        parent = {}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                break
            for di, dj, w in moves:
                nb = (cur[0] + di, cur[1] + dj)
                if not (0 <= nb[0] < nx and 0 <= nb[1] < ny) or not (free(*nb) or nb == s):
                    continue
                cost = g_cost[cur] + w * c
                if cost < g_cost.get(nb, math.inf):
                    g_cost[nb] = cost
                    parent[nb] = cur
                    heapq.heappush(openq, (cost + math.dist(centre(*nb), centre(*g)), nb))
        if g not in parent and g != s:
            return [goal]
        cells = [g]
        while cells[-1] != s:
            cells.append(parent[cells[-1]])
        pts = [start] + [centre(*k) for k in reversed(cells[:-1])]
        pts[-1] = goal
        # line-of-sight shortcutting
        out, i = [], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.segment_clear(pts[i], pts[j], alt, clearance, obs):
                j -= 1
            out.append(pts[j])
            i = j
        if not out:
            out = [goal]
        return [(round(x, 2), round(y, 2)) for x, y in out]

    def random_free_point(self, rng, region, margin: float = 20.0, avoid=(), avoid_r: float = 30.0, tries: int = 400):
        x0, y0, x1, y1 = region
        for _ in range(tries):
            p = (rng.uniform(x0, x1), rng.uniform(y0, y1))
            if all(o.horizontal_gap(*p) > margin for o in self.obstacles) and \
                    all(math.dist(p, q[:2]) > avoid_r for q in avoid):
                return [round(p[0], 1), round(p[1], 1)]
        return None

    # ------------------------------------------------------------------ Gazebo
    def sdf_models(self) -> str:
        return "".join(o.sdf() for o in self.obstacles)

    def write_world(self, base_world_path: str, out_path: str) -> str:
        """Base world (ground, GCS, pads, plugins) + this map's obstacles -> out_path."""
        import re
        with open(base_world_path, encoding="utf-8") as fh:
            text = fh.read()
        text = re.sub(r'<model name="rubble">.*?</model>\s*', "", text, flags=re.S)
        text = text.replace("</world>", f"    <!-- 3-D map: {self.name} ({len(self.obstacles)} obstacles) -->\n"
                            + self.sdf_models() + "  </world>")
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(text)
        return out_path
