"""What the gripper holds, as rammp_feeding's arm checks it (arm.Held), and the planner's world for drinking.

  Gripper  nothing held: its tip the tool frame (between the fingers); its planned paths keep the fingers
           min_tcp_above_table_m above the table; any tilt
  Mug      the cup, held as picked up: its tip drinking.mug.tip_in_tool_m (what goes to the mouth); its axis kept within
           max_tilt_deg of vertical; its shape spheres along its axis (where the handle puts it in the gripper), kept out
           of the table and the user on every planned path
  world    RAMMP-CuRobo's world (arm.base_frame): the pedestal, the table (once the cup's handle has placed it: its top at
           the cup's bottom), raised for the planner by what hangs below the tool (the planner knows the arm, not what
           it holds), the cup where it stands when it is not held, and the user round their measured mouth
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from rammp_drinking.config import D, cfg
from rammp_drinking.handle import Handle, handle_in_tool, nominal
from rammp_feeding.arm import Held
from rammp_feeding.geometry import rpy_matrix


def cup_table() -> tuple[float | None, dict]:
    """The cup's table: (its top z or None, its extent {x: [near, far], y: [right, left]}) -- drinking.table where set
    (the cup on its own table), else the site's one table (workspace, shared with rammp_feeding)."""
    T, W = D().table, cfg.workspace
    th = T.get("height_m", None)
    return (W.get("table_height_m", None) if th is None else th), (T.get("extent_m", None) or W.table_extent_m)


def table_z(h: Handle) -> float:
    """The cup's table's top (cup_table()), else the bottom of the cup standing on it, from its handle."""
    th = cup_table()[0]
    return float(h.p[2] - float(D().mug.handle_centre_above_bottom_m) if th is None else th)


def cup_axis(h: Handle) -> tuple[np.ndarray, np.ndarray]:
    """(the cup's bottom centre, its up axis) in the handle's frame's parent, from its handle."""
    M = D().mug
    v = M.handle_to_axis_m                                      # behind its face (a number), or [x, y, z] in its frame
    a = h.p + h.R @ np.asarray(v, float) if isinstance(v, (list, tuple)) else h.p - h.R[:, 2] * float(v)
    up = h.R[:, 0]
    return a - up * float(M.handle_centre_above_bottom_m), up


def _boxes(obstacles: list[dict]):
    return (np.array([o["position"] for o in obstacles]).reshape(-1, 3),
            np.array([rpy_matrix(*np.radians(o.get("rpy_deg", [0.0, 0.0, 0.0]))) for o in obstacles]).reshape(-1, 3, 3),
            np.array([np.asarray(o["dims"]) / 2 for o in obstacles]).reshape(-1, 3), [o["name"] for o in obstacles])


def world(table: float | None, cup: Handle | None, raise_m: float) -> dict:
    """The planner's world: the table's top at `table` (None: not yet seen), raised by raise_m; the cup standing at its
    handle `cup` (None: held, or not seen); the user once their mouth is measured."""
    from rammp_feeding.scene import user_mouth, user_obstacles
    obs = [dict(name="pedestal", position=[0.0, 0.0, -0.05], dims=[0.14, 0.14, 0.04])]   # (the planner's own: unpadded)
    if table is not None:
        ext = cup_table()[1]
        tx, ty = [float(v) for v in ext.x], [float(v) for v in ext.y]
        top = table + raise_m
        obs.append(dict(name="table", position=[sum(tx) / 2, sum(ty) / 2, top - 0.025],
                        dims=[tx[1] - tx[0], ty[1] - ty[0], 0.05]))
    if cup is not None:
        M = D().mug
        b, up = cup_axis(cup)
        r, h = float(M.radius_m), float(M.height_m)
        obs.append(dict(name="cup", position=[float(v) for v in b + up * h / 2], dims=[2 * r, 2 * r, h]))
    m = user_mouth()
    if m is not None:
        obs += user_obstacles(m)
    for o in obs:
        o["position"], o["dims"] = [float(v) for v in o["position"]], [float(v) for v in o["dims"]]
        o["rpy_deg"] = [float(v) for v in o.get("rpy_deg", [0.0, 0.0, 0.0])]
    return dict(base_frame=str(cfg.arm.base_frame), obstacles=obs, objects=[], targets=[])


def push(robot, w: dict) -> None:
    """Our world into the shared planner: a YAML file both containers see, then set_world (the planner's own back when
    the command ends: rammp_feeding.scene.restore_world)."""
    p = Path(str(cfg.planner.world_file)).expanduser()
    text = yaml.safe_dump(w)
    if not p.exists() or p.read_text() != text:                # (written only when it changes)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
    robot.set_world(str(cfg.planner.world_file_in_planner or p))
    robot.world_pushed = True


class Gripper(Held):
    """The gripper with nothing in it."""
    name = "the gripper"
    tip_in_tool = np.zeros(3)
    max_tilt_deg = 180.0

    def __init__(self, table: float | None = None, cup: Handle | None = None, camera: bool = True):
        self.table = cup_table()[0] if table is None else table     # the table's top (None: not known yet) ...
        self.cup = cup                                          # ... and the cup standing there (None: unseen)
        self.camera = camera                                    # (False: an arm built without the wrist camera's model)

    def floor(self, x: float, y: float) -> float:
        ext = cup_table()[1]
        if self.table is not None and ext.x[0] <= x <= ext.x[1] and ext.y[0] <= y <= ext.y[1]:
            return self.table + float(D().min_tcp_above_table_m)
        return -math.inf

    def push_world(self, robot) -> None:
        push(robot, world(self.table, self.cup, float(D().min_tcp_above_table_m)))


class Mug(Held):
    """The cup in the gripper, picked up by its handle h (None: as the grasp holds an upright cup)."""
    name = "the drink"

    def __init__(self, h: Handle | None, table: float | None):
        M = D().mug
        self.table = table
        self.tip_in_tool = np.asarray(M.tip_in_tool_m, float)
        X = handle_in_tool(nominal() if h is None else h)        # tool <- handle at the grasp
        b, up = cup_axis(Handle(X[:3, 3], X[:3, :3]))            # the cup's bottom and axis, tool frame
        self.tilt_axis_in_tool, self.tilt_ref = up, np.array([0.0, 0.0, 1.0])
        self.max_tilt_deg = float(M.max_tilt_deg)
        r, ht, pad = float(M.radius_m), float(M.height_m), float(M.padding_m)
        n = max(2, int(math.ceil(ht / max(r, 1e-3))))
        self._spheres = [[*(b + up * (r + k * max(ht - 2 * r, 0.0) / (n - 1))), r + pad] for k in range(n)]

    def spheres(self, arm, carrying: bool) -> list[list[float]]:
        return [[float(v) for v in s] for s in self._spheres]

    def boxes(self):
        from rammp_feeding.scene import user_mouth, user_obstacles
        obs = []
        if self.table is not None:
            ext = cup_table()[1]
            tx, ty = [float(v) for v in ext.x], [float(v) for v in ext.y]
            obs.append(dict(name="table", position=[sum(tx) / 2, sum(ty) / 2, self.table - 0.025],
                            dims=[tx[1] - tx[0], ty[1] - ty[0], 0.05]))
        m = user_mouth()
        if m is not None:
            obs += user_obstacles(m)
        return _boxes(obs)

    def push_world(self, robot) -> None:
        push(robot, world(self.table, None, self.hang_m()))

    def hang_m(self) -> float:
        """How far the cup (padded) hangs below the tool frame's origin, upright: the planner's table rises by this."""
        up = self.tilt_axis_in_tool
        return float(max(0.0, -min(np.asarray(s[:3]) @ up - s[3] for s in self._spheres)))
