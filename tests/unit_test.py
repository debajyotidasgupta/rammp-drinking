"""Unit tests: the grasp's poses against the old system's computation, the handle's frame from its points, the cup as
the arm's checks take it, the planner's world, the session, and the engine's availability."""

from __future__ import annotations

import math
import unittest

import fixture  # noqa: F401  (the test site: before rammp_drinking is imported)
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from rammp_drinking import config  # noqa: E402
from rammp_drinking.config import cfg  # noqa: E402
from rammp_drinking.handle import Handle, frame_from_points, grasp_poses, handle_in_tool, nominal  # noqa: E402
from rammp_drinking.mug import Gripper, Mug, cup_axis, cup_table, table_z, world  # noqa: E402


def old_pose(handle_pos, handle_quat, offset_rpy, offset_xyz):
    """The old system's get_aruco_relative_pose(transform, "drink") (rammp/interfaces/perception_interface.py)."""
    A = np.eye(4)
    A[:3, :3], A[:3, 3] = Rotation.from_quat(handle_quat).as_matrix(), handle_pos
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rotation.from_euler("xyz", offset_rpy).as_matrix(), offset_xyz
    G = A @ T
    _, _, yaw = Rotation.from_matrix(G[:3, :3]).as_euler("xyz")
    return G[:3, 3], Rotation.from_euler("xyz", [np.pi / 2, 0, yaw]).as_matrix()


class Grasp(unittest.TestCase):
    def test_the_old_systems_poses(self):
        rng = np.random.default_rng(0)
        for _ in range(20):
            R = Rotation.from_euler("xyz", [rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(-3, 3)]) * \
                Rotation.from_matrix(nominal().R)
            h = Handle(rng.uniform(-0.5, 0.5, 3), R.as_matrix())
            P = grasp_poses(h)
            for k, off in (("pre", [0.02, 0.0, 0.15]), ("inside_bottom", [0.02, 0.0, 0.017]),
                           ("inside_top", [0.05, 0.0, 0.017]), ("lifted", [0.233, 0.0, 0.017])):
                p, Ro = old_pose(h.p, R.as_quat(), [np.pi, 0, np.pi / 2], off)
                self.assertLess(np.linalg.norm(P[k][0] - p), 1e-9, k)
                self.assertLess(np.abs(P[k][1] - Ro).max(), 1e-9, k)

    def test_an_upright_handle_is_entered_level(self):
        """Its face toward the arm: the tool's z into the handle, its y up (the cup upright when held)."""
        P = grasp_poses(nominal())
        _, R = P["inside_top"]
        self.assertLess(np.linalg.norm(R[:, 1] - [0, 0, 1]), 1e-9)
        self.assertLess(np.linalg.norm(R[:, 2] - [1, 0, 0]), 1e-9)
        self.assertLess(np.linalg.norm(P["pre"][0] - P["inside_bottom"][0] - [-0.133, 0, 0]), 1e-9)


class HandleFrame(unittest.TestCase):
    def test_a_plate_seen_from_the_arm(self):
        rng = np.random.default_rng(1)
        c, x, y = np.array([0.45, -0.1, 0.06]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0])
        a, b = rng.uniform(-0.03, 0.03, 800), rng.uniform(-0.015, 0.015, 800)
        pts = c + a[:, None] * x + b[:, None] * y + rng.normal(0, 0.0005, (800, 3))
        p, R, size = frame_from_points(pts, np.array([0.2, -0.1, 0.2]))
        self.assertLess(np.linalg.norm(p - c), 0.003)
        self.assertGreater(R[:, 0] @ x, math.cos(math.radians(2)))      # x up the handle
        self.assertGreater(R[:, 2] @ [-1, 0, 0], math.cos(math.radians(2)))   # z toward the camera
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=6)
        self.assertAlmostEqual(size[0], 0.06, delta=0.004)
        self.assertAlmostEqual(size[1], 0.03, delta=0.004)

    def test_a_level_face_is_refused(self):
        pts = np.random.default_rng(2).uniform(-0.02, 0.02, (300, 3)) * [1, 1, 0]
        with self.assertRaises(ValueError):
            frame_from_points(pts, np.array([0.0, 0.0, 0.5]))



class Opening(unittest.TestCase):
    def test_the_fingers_aim_at_the_opening_not_the_outline(self):
        """A C-shaped handle against the cup's wall: the aim is the opening's centre, on the handle's plane."""
        import types
        from rammp_drinking import handle as H
        from rammp_feeding import scene
        m = np.zeros((480, 640), bool)
        m[100:120, 200:400] = m[280:300, 200:400] = m[100:300, 380:400] = True    # top and bottom bars, the outer bar
        colour = types.SimpleNamespace(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        arm = types.SimpleNamespace(cam=types.SimpleNamespace(colour=colour))
        pose, scene.camera_pose = scene.camera_pose, lambda a, q: (np.zeros(3), np.eye(3))
        try:
            p, size = H.opening(m, arm, None, np.array([0.0, 0.0, 1.0]), np.eye(3))   # its plane 1 m ahead
        finally:
            scene.camera_pose = pose
        self.assertTrue(np.allclose(p, [(289.5 - 320) / 600, (199.5 - 240) / 600, 1.0], atol=1e-3))
        self.assertAlmostEqual(size[0], 180 / 600, delta=0.01)
        self.assertAlmostEqual(size[1], 160 / 600, delta=0.01)
        self.assertIsNone(H.opening(np.zeros((480, 640), bool), arm, None, np.zeros(3), np.eye(3)))


class Table(unittest.TestCase):
    def test_the_site_table_unless_the_cup_has_its_own(self):
        th, ext = cup_table()
        self.assertEqual(th, cfg.workspace.get("table_height_m", None))
        self.assertEqual(list(ext.x), list(cfg.workspace.table_extent_m.x))
        self.assertEqual(Gripper().table, th)

class Cup(unittest.TestCase):
    def test_upright_in_the_gripper(self):
        m = Mug(None, table=0.0)
        self.assertLess(np.linalg.norm(m.tilt_axis_in_tool - [0, 1, 0]), 1e-9)        # the cup's up: the tool's y
        self.assertEqual(list(m.tip_in_tool), [0.07, 0.10, 0.18])
        sp = np.array(m.spheres(None, True))
        self.assertTrue(np.allclose(sp[:, 3], 0.05))                                  # radius + padding
        axis_pts = sp[:, :3] - sp[0, :3]
        self.assertLess(np.abs(np.cross(axis_pts[1:], [0, 1, 0])).max(), 1e-9)        # along the cup's axis
        X = handle_in_tool(nominal())
        b, _ = cup_axis(Handle(X[:3, 3], X[:3, :3]))
        self.assertAlmostEqual(sp[0, 1] - b[1], 0.04, places=9)                       # the first a radius up its axis
        self.assertGreater(m.hang_m(), 0.0)

    def test_the_table_under_a_standing_cup(self):
        h = Handle(np.array([0.4, 0.0, 0.06]), nominal().R)
        self.assertAlmostEqual(table_z(h), 0.01)
        b, up = cup_axis(h)
        self.assertLess(np.linalg.norm(b - [0.46, 0.0, 0.01]), 1e-9)                  # behind its face, at the bottom

    def test_world(self):
        from rammp_feeding.scene import measured_mouth
        h = Handle(np.array([0.4, 0.0, 0.06]), nominal().R)
        w = world(0.01, h, 0.03)
        names = [o["name"] for o in w["obstacles"]]
        self.assertEqual(names, ["pedestal", "table", "cup"])
        tab = w["obstacles"][1]
        self.assertAlmostEqual(tab["position"][2] + tab["dims"][2] / 2, 0.04)          # its top raised by 0.03
        measured_mouth([-0.3, 0.3, 0.45])
        try:
            self.assertEqual([o["name"] for o in world(None, None, 0.0)["obstacles"]], ["pedestal", "user_head",
                                                                                         "user_torso"])
            self.assertIn("user_head", Mug(h, 0.01).boxes()[3])
        finally:
            measured_mouth(None)
        self.assertEqual(Gripper(0.01, h).floor(0.4, 0.0), 0.01 + float(cfg.drinking.min_tcp_above_table_m))
        self.assertEqual(Gripper(None).floor(0.4, 0.0), -math.inf)


class Session(unittest.TestCase):
    def test_saved_and_cleared(self):
        config.save_session(dict(holding=True))
        self.assertTrue(config.session()["holding"])
        config.save_session({})
        self.assertEqual(config.session(), {})

    def test_measured(self):
        self.assertEqual(config.unmeasured(), [])
        for n in cfg.drinking.poses:
            self.assertTrue(config.captured(n))


if __name__ == "__main__":
    unittest.main()
