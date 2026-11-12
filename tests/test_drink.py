"""The whole drink against the fake robot (tests/cup_robot.py): pick the cup up, one sip, put it down -- the module's
own code from the handle's frame to the arm's joints, without ROS. Exit 0 and "THE DRINK GOES THROUGH" when every
check holds."""

from __future__ import annotations

import sys

import fixture  # noqa: F401  (the test site: before rammp_drinking is imported)
import numpy as np  # noqa: E402

from rammp_drinking import actions, config  # noqa: E402
from rammp_drinking.config import cfg, captured  # noqa: E402
from rammp_drinking.handle import Handle, grasp_poses  # noqa: E402
from rammp_drinking.mug import Gripper  # noqa: E402
from cup_robot import CupRobot, CupScene  # noqa: E402
from fake_robot import FakeFace  # noqa: E402
from rammp_feeding.arm import Arm  # noqa: E402
from rammp_feeding.geometry import max_step  # noqa: E402


def passed(robot, T) -> float:
    return float(min(np.linalg.norm(np.asarray(p) - T) for p in robot.tool_path))


def at(arm, name) -> bool:
    """The tool at the pose captured as `name` (a level plan reaches its pose, maybe in other joints)."""
    (T, R), (Tc, Rc) = arm.tool(), arm.tool(captured(name))
    return float(np.linalg.norm(T - Tc)) < 0.003 and float(np.abs(R - Rc).max()) < 0.02


def main() -> int:
    sc = CupScene()
    robot = CupRobot(sc, captured("home"))
    arm = Arm(robot, kin=robot.kin, held=Gripper())
    grips = []
    truth = Handle(np.asarray(sc.handle_p, float), np.asarray(sc.handle_R, float))
    P = grasp_poses(truth)

    out = actions.pick_up(robot, arm, grips.append)
    s = config.session()
    h = Handle.from_json(s["handle"])
    assert grips == [1.0, 0.0], grips                      # closed into the handle, opened to hold it
    assert s["holding"] and len(s["lifted_joints"]) == 7
    assert np.linalg.norm(h.p - truth.p) < 0.004, "the handle %.1f mm off" % (1000 * np.linalg.norm(h.p - truth.p))
    assert h.R[:, 0] @ truth.R[:, 0] > np.cos(np.radians(3)) and h.R[:, 2] @ truth.R[:, 2] > np.cos(np.radians(3))
    for k in ("pre", "inside_bottom", "inside_top", "lifted"):   # (pre: reached straight down from above it)
        assert passed(robot, P[k][0]) < 0.006, "%s: %.1f mm" % (k, 1000 * passed(robot, P[k][0]))
    assert at(arm, "drink_hold")
    print("  picked up: the handle %.1f mm off, the cup at %s" % (1000 * np.linalg.norm(h.p - truth.p), out["cup_xyz_m"]))

    rc = actions.give_sip(robot, arm, FakeFace(robot))
    stop = float(cfg.delivery.stop_m)
    assert rc == 0, rc
    assert robot.mouth_dist and abs(min(robot.mouth_dist) - stop) < 0.01, (min(robot.mouth_dist), stop)
    assert at(arm, "drink_hold")
    print("  the sip: the cup's tip %.0f mm from the mouth for %.1f s" % (1000 * min(robot.mouth_dist),
                                                                         float(cfg.drinking.sip_s)))

    n = len(robot.tool_path)
    actions.put_down(robot, arm, grips.append)
    assert grips == [1.0, 0.0, 1.0], grips                 # closed: it let go
    assert config.session() == {}
    tail = robot.tool_path[n:]
    assert min(np.linalg.norm(np.asarray(p) - P["inside_top"][0]) for p in tail) < 0.006, "not set down where it stood"
    assert max_step(arm.q(), captured("home")) < 1.0
    print("THE DRINK GOES THROUGH (%d moves; picked up, a sip %.0f mm short of the mouth, set down where it stood, home)"
          % (len(robot.moves), 1000 * stop))
    return 0


if __name__ == "__main__":
    sys.exit(main())
