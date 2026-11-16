"""rammp_feeding's fake robot (tests/fake_robot.py: a kinematic arm that plays its trajectories, a straight-line planner,
the wrist camera rendered from where the arm holds it) with a cup on the table: its handle a flat plate the depth reads
and SAM3's mask covers, where CupScene puts it. It records every tool position it passes and, at each wait's check, how
far the cup's tip is from the user's mouth."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rammp_drinking.config import cfg  # noqa: I001  (first: rammp_feeding loads with drinking's defaults)
from fake_robot import TILT, FakeRobot, Scene
from rammp_feeding.ros import Detection

TABLE_Z = 0.095 if TILT else 0.15         # (TILT: rammp_feeding's tilt30 test site; level: the cup far enough out and up)


@dataclass
class CupScene(Scene):
    table_z: float = TABLE_Z
    mouth: tuple = (-0.27, -0.20, 0.60) if TILT else (-0.277, 0.306, TABLE_Z + 0.40)
    handle_p: tuple = (0.60, -0.15, TABLE_Z + 0.05) if TILT else (0.80, -0.05, TABLE_Z + 0.05)   # its centre (fixture:
                                                                    # 0.05 above the cup's bottom)
    handle_R: tuple = (((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)) if TILT else   # columns x up, y, z out of its
                       ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)))             # face: toward the arm (level);
                                                                    # tilt30: toward +y, the side the leaning arm reaches
    handle_size: tuple = (0.06, 0.03)                               # along its x and y


class CupRobot(FakeRobot):
    def __init__(self, sc: CupScene, q0):
        super().__init__(sc, q0)
        self.tool_path: list[np.ndarray] = []
        self.mouth_dist: list[float] = []

    def joint_sample(self, wait_s: float = 2.0):
        s = super().joint_sample(wait_s)
        self.tool_path.append(self.kin.fk(self.joints())[0])
        return s

    def plan_pose(self, start_deg, T, R, timeout_s, hold=1):
        """The planner's HOLD_LEVEL, faked: a straight tool line, its orientation turned evenly (level stays level)."""
        from rammp_feeding.geometry import rodrigues, rot_log
        T0, R0 = self.kin.fk(list(start_deg))
        w, q, out = rot_log(np.asarray(R) @ R0.T), list(start_deg), [list(start_deg)]
        for k in range(1, 61):
            q = self.kin.ik(T0 + (np.asarray(T) - T0) * k / 60, rodrigues(w * k / 60) @ R0, q)
            assert q is not None, "no level path"
            out.append(list(q))
        return np.degrees(np.unwrap(np.radians(np.array(out, float)), axis=0)), np.linspace(0, 3, 61)

    def check(self) -> None:
        T, R = self.kin.fk(self.joints())
        tip = T + R @ np.asarray(cfg.drinking.mug.tip_in_tool_m, float)
        self.mouth_dist.append(float(np.linalg.norm(tip - np.asarray(self.sc.mouth, float))))

    def _plate(self):
        """(the pixels whose ray meets the handle's plate, its depth there along each ray)."""
        o, d = self._rays()
        p0, R = np.asarray(self.sc.handle_p, float), np.asarray(self.sc.handle_R, float)
        n = R[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((p0 - o) @ n) / (d @ n)
        p = o + t[..., None] * d - p0
        a, b = p @ R[:, 0], p @ R[:, 1]
        w, h = self.sc.handle_size
        return (np.abs(a) <= w / 2) & (np.abs(b) <= h / 2) & (t > 0), t

    def render_depth(self) -> np.ndarray:
        z = super().render_depth().astype(float)
        hit, t = self._plate()
        return np.where(hit, 1000 * t, z).astype(np.uint16)

    def detect(self, role, prompts, threshold):
        if not any(p in cfg.drinking.handle.prompts for p in prompts):
            return super().detect(role, prompts, threshold)
        hit, _ = self._plate()
        if not hit.any():
            return []
        v, u = np.nonzero(hit)
        return [Detection(str(cfg.drinking.handle.prompts[0]), 0.85, [float(u.min()), float(v.min()), float(u.max()),
                                                                      float(v.max())], hit)]
