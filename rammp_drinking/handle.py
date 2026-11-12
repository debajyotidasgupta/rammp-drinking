"""The cup's handle, from the wrist camera at drink_gaze, and the grasp's tool poses from it (the old system's grasp).

find       one colour frame and a depth median from frames after the arm stopped; SAM3 (the platform's detector) on
           drinking.handle.prompts; the best-scored mask with enough depth under it; its points in the base frame, those
           within inlier_m of their median kept; a plane through them (its normal toward the camera: the frame's z); x
           the base frame's up laid into that plane (the cup stands on the table: the handle runs up it); y = z x x;
           the origin the centre of the points' smallest enclosing rectangle in the plane
grasp      a tool pose per drinking.grasp offset, in the handle's frame turned by grasp.rpy_deg, then levelled: roll
           level_roll_deg, pitch 0, and the yaw that gives (the old system's poses, exactly)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from rammp_drinking.config import D
from rammp_feeding import record

log = logging.getLogger("rammp")


@dataclass
class Handle:
    p: np.ndarray                   # its centre, base frame
    R: np.ndarray                   # base <- handle: x up the handle, y right to left, z out of its face to the camera
    points: int = 0
    score: float = 0.0
    size_m: list = field(default_factory=list)    # its rectangle in the plane: along x, along y

    def matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.R, self.p
        return T

    def to_json(self) -> dict:
        return dict(p=[float(v) for v in self.p], R=np.asarray(self.R, float).round(6).tolist(), points=int(self.points),
                    score=round(float(self.score), 3), size_m=[round(float(v), 4) for v in self.size_m])

    @classmethod
    def from_json(cls, d: dict) -> "Handle":
        return cls(np.array(d["p"], float), np.array(d["R"], float), d.get("points", 0), d.get("score", 0.0),
                   d.get("size_m", []))


def frame_from_points(pts: np.ndarray, camera_at: np.ndarray) -> tuple[np.ndarray, np.ndarray, list[float]]:
    """(centre, R, rectangle size) of a flat handle from its base-frame points, the camera at camera_at."""
    import cv2
    c = pts.mean(0)
    n = np.linalg.svd(pts - c, full_matrices=False)[2][2]
    if n @ (camera_at - c) < 0:                          # its face toward the camera
        n = -n
    up = np.array([0.0, 0.0, 1.0])
    x = up - (up @ n) * n
    if np.linalg.norm(x) < 1e-3:
        raise ValueError("the handle's face is level -- not a cup standing on the table")
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    uv = np.stack([(pts - c) @ x, (pts - c) @ y], 1).astype(np.float32)
    (cu, cv), (w, h), ang = cv2.minAreaRect(uv)
    a = np.radians(ang)                                  # the rectangle's sides, in the plane: its size along x and y
    e1, e2 = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    sx = abs(e1[0]) * w + abs(e2[0]) * h
    sy = abs(e1[1]) * w + abs(e2[1]) * h
    return c + cu * x + cv * y, np.stack([x, y, n], 1), [float(sx), float(sy)]


def find(robot, arm, out: Path | None = None) -> Handle:
    """The handle as the wrist camera sees it from here (the arm still)."""
    from rammp_feeding import scene
    from rammp_feeding.perception import _in, world_points
    H = D().handle
    q = arm.q()
    cam = robot.cameras["camera"]
    since = time.time()
    _, rgb = cam.frame(after=since)
    Dm = cam.depth_median(int(H.depth_frames), since, wait_s=float(H.depth_wait_s))
    record.save_image(out, "colour.jpg", rgb, quality=95)
    record.save_npy(out, "depth.npy", Dm)
    dets = robot.detect("camera", [str(p) for p in H.prompts], float(H.score_threshold))
    W, uv, _ = world_points(arm.cam, arm, Dm, q)
    o, _ = scene.camera_pose(arm, q)
    best, why = None, "SAM3 found no handle (%s)" % ", ".join(H.prompts)
    for d in sorted(dets, key=lambda d: -d.score):
        pts = W[_in(d.mask, uv)]
        if len(pts):
            pts = pts[np.linalg.norm(pts - np.median(pts, 0), axis=1) < float(H.inlier_m)]
        if len(pts) < int(H.min_points):
            why = "the handle's mask has %d depth points (at least %d)" % (len(pts), int(H.min_points))
            continue
        p, R, size = frame_from_points(pts, o)
        best = Handle(p, R, len(pts), float(d.score), size)
        break
    record.save_json(out, "handle.json", dict(handle=None if best is None else best.to_json(), why=None if best else why,
                                              detections=[dict(label=d.label, score=d.score, box=d.box) for d in dets]))
    if best is None:
        raise RuntimeError(why)
    log.info("  the handle at (%.3f %.3f %.3f), its face %.0f deg off level toward the camera; %d points, SAM3 %.2f",
             *best.p, np.degrees(np.arcsin(abs(best.R[2, 2]))), best.points, best.score)
    return best


def _level(R: np.ndarray) -> np.ndarray:
    """R with its roll and pitch replaced (xyz Euler, the old system's): roll level_roll_deg, pitch 0, its yaw kept."""
    from scipy.spatial.transform import Rotation
    yaw = Rotation.from_matrix(R).as_euler("xyz")[2]
    return Rotation.from_euler("xyz", [np.radians(float(D().grasp.level_roll_deg)), 0.0, yaw]).as_matrix()


def grasp_pose(h: Handle, offset_m) -> tuple[np.ndarray, np.ndarray]:
    """A tool pose (position, R) in the base frame: offset_m in the handle's frame, its orientation levelled."""
    from scipy.spatial.transform import Rotation
    G = D().grasp
    R0 = h.R @ Rotation.from_euler("xyz", np.radians(np.asarray(G.rpy_deg, float))).as_matrix()
    return h.p + h.R @ np.asarray(offset_m, float), _level(R0)


def grasp_poses(h: Handle) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    G = D().grasp
    return {k: grasp_pose(h, G[k + "_m"]) for k in ("pre", "inside_bottom", "inside_top", "lifted")}


def nominal() -> Handle:
    """A handle standing upright at the origin, its face toward the arm's base (-x): where the cup sits in the gripper
    when no handle was seen (its place relative to the tool does not depend on where the cup stood)."""
    return Handle(np.zeros(3), np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]).T)


def handle_in_tool(h: Handle) -> np.ndarray:
    """4 x 4 tool <- handle at the grasp (inside_top): where the cup sits in the gripper."""
    T, R = grasp_pose(h, D().grasp.inside_top_m)
    G = np.eye(4)
    G[:3, :3], G[:3, 3] = R, T
    return np.linalg.inv(G) @ h.matrix()

