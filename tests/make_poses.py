"""The test site's arm poses (tests/poses/*.json), solved on the fake robot's model in one arm shape (the first of
drinking.grasp.ik_seeds_deg: reaching forward), so that the fake's planner -- a straight joint line, the short way -- stays
inside the joint limits between them: home parked over the table's near edge, the gripper down; drink_gaze the wrist
camera on the cup's handle (CupScene) from 0.2 m in front and 0.25 m above it; drink_hold and drink_deliver the cup
upright (tool y up), its tip toward the user's mouth, drink_deliver with the tip 0.25 m from it. Every line the drink
takes between them is checked against the limits. Run once in the image: python3 tests/make_poses.py"""

import itertools
import json

import numpy as np

import fixture  # noqa: F401  (the test site)
from cup_robot import CupRobot, CupScene
from rammp_drinking.config import cfg
from rammp_drinking.handle import Handle, grasp_poses
from rammp_feeding.arm import Arm
from rammp_feeding.geometry import fold

sc = CupScene()
FAMILY = [float(v) for v in cfg.drinking.grasp.ik_seeds_deg[0]]
robot = CupRobot(sc, FAMILY)
arm = Arm(robot, kin=robot.kin, measured=False)
(fixture.ROOT / "tests" / "poses").mkdir(exist_ok=True)


def frame(z, up=(0.0, 0.0, 1.0)):
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.cross(np.asarray(up, float), z)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1)


def level(d):
    """Tool y up, tool z along the horizontal direction d."""
    d = np.array([d[0], d[1], 0.0]) / np.hypot(d[0], d[1])
    return np.stack([np.cross([0.0, 0.0, 1.0], d), [0.0, 0.0, 1.0], d], 1)


def line_ok(a, b) -> bool:
    """The fake planner's path a -> b (the short way per joint) inside the model's limits."""
    a = np.asarray(a, float)
    d = fold(np.asarray(b, float) - a)
    return all(arm.margin(a + d * s) >= 0 for s in np.linspace(0, 1, 41))


def solve(T, R, seeds):
    for s in seeds:
        q = arm.ik(np.asarray(T, float), R, s)
        if q is not None:
            Tq, Rq = arm.tool(q)
            if np.linalg.norm(Tq - T) < 1e-3 and np.linalg.norm(Rq - R) < 1e-2:
                return q
    return None


def first(name, candidates, prev):
    """The first candidate (T, R) solved with every joint min_limit_margin_deg from its limit (the planner's goal
    margin) whose line from prev stays inside the limits, written as <name>.json."""
    m = float(cfg.drinking.grasp.min_limit_margin_deg)
    for T, R in candidates:
        q = solve(T, R, ([prev] if prev is not None else []) + [FAMILY])
        if q is not None and arm.margin(q) >= m and (prev is None or line_ok(prev, q)):
            (fixture.ROOT / "tests" / "poses" / (name + ".json")).write_text(json.dumps(dict(
                label=name + " (tests/make_poses.py)", joints_deg=[round(float(v), 3) for v in q]), indent=1))
            print(name, np.round(T, 3).tolist(), np.round(q, 1).tolist())
            return q
    raise SystemExit("no pose for " + name)


def grid(c, s):
    return [np.asarray(c, float) + np.array(o) for o in itertools.product((0.0, -s, s), repeat=3)]


hp = np.asarray(sc.handle_p, float)
mouth = np.asarray(sc.mouth, float)
q_home = first("home", [(T, frame([0, 0, -1], up=(1, 0, 0))) for T in grid([0.35, -0.30, sc.table_z + 0.30], 0.05)], None)
q_gaze = first("drink_gaze", [(T, frame(hp - T)) for T in grid(hp + [-0.2, 0.0, 0.25], 0.03)], q_home)
P = grasp_poses(Handle(hp, np.asarray(sc.handle_R, float)))
above = (P["pre"][0] + np.array([0.0, 0.0, float(cfg.drinking.grasp.rise_m)]), P["pre"][1])
q_pre = max((q for q in (arm.ik(*above, s) for s in [q_gaze, *cfg.drinking.grasp.ik_seeds_deg]) if q is not None),
            key=arm.margin)
assert arm.margin(q_pre) >= float(cfg.drinking.grasp.min_limit_margin_deg), "pre: %.1f deg" % arm.margin(q_pre)
assert line_ok(q_gaze, q_pre), "gaze -> pre"
q_lift = arm.ik(*P["lifted"], q_pre)
holds = [(T, level(d(T))) for c in ([0.40, 0.05, sc.table_z + 0.35], [0.50, 0.0, sc.table_z + 0.30], [0.30, 0.20, sc.table_z + 0.40])
         for T in grid(c, 0.05) for d in (lambda T: mouth - T, lambda T: np.array([1.0, 0.0, 0.0]),
                                           lambda T: np.array([0.0, 1.0, 0.0]))]
q_hold = first("drink_hold", holds, q_lift)
dels = []
for dist, dz in itertools.product((0.25, 0.30, 0.35), (0.0, -0.05, 0.05)):
    for lean in (0.0, 0.3, -0.3):                    # the approach turned this much (rad) about the vertical
        T0 = arm.tool(q_hold)[0]
        R = level(mouth - T0)
        c, s_ = np.cos(lean), np.sin(lean)
        R = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]) @ R
        tip = mouth - dist * R[:, 2] + np.array([0.0, 0.0, dz])
        dels.append((tip - R @ np.asarray(fixture.MUG["tip_in_tool_m"], float), R))
q_del = first("drink_deliver", dels, q_hold)
assert line_ok(q_lift, q_hold) and line_ok(q_pre, q_home), "the put-down's lines"
print("every line inside the limits")
