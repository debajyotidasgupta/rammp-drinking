"""The drinking engine end to end on the real arm stack (tests/test_e2e.sh): the node as sheppy runs it, standalone, driven
through its own /drinking/execute_action as the orchestrator would, with the real 2.0 driver (each goal accepted, then
cancelled: rammp_feeding's tests/sim_arm.py), its description node and the 2.0.1 planner (its HOLD_LEVEL for every path
that carries the cup), and the gripper through the engine library to the driver. The cameras, SAM3, the cup and the face
are tests/cup_robot.py's.

  pick_up_drink               the handle found, the cup picked up and held upright at drink_hold
  give_sip                    one sip: the cup's tip delivery.stop_m short of the mouth, held, back
  give_sip, cancelled at the mouth   ends as cancelled within a few seconds
  put_down_drink              recovers (back from the user), the cup set back where it stood, home

Each action: its result, /drinking/status, the planner's own world back (/reset) as it ends, the token on its goals,
and none of its subscriptions left once it has ended."""

from __future__ import annotations

import json
import os
import sys
import threading
import time

import fixture  # (the test site: before rammp_drinking is imported)

os.environ["RAMMP_SET"] += ",planner.world_file=/share/drink_world.yaml,planner.world_file_in_planner=/feeding/drink_world.yaml"

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402


def main() -> int:
    rclpy.init(args=["--ros-args", "-p", "use_orchestrator:=false", "-p", "state_dir:=%s" % fixture.STATE,
                     "-p", "runs_dir:=%s" % (fixture.WORK / "runs"), "-p", "record:=true"])
    from rammp_drinking import config
    from rammp_drinking.config import captured
    from rammp_drinking.engine import Drinking
    from rammp_feeding.ros import Robot
    from rammp_orchestrator_interfaces.action import ExecuteAction
    from rammp_orchestrator_interfaces.msg import Result
    from std_msgs.msg import Empty, String
    from cup_robot import CupRobot, CupScene
    from fake_robot import FakeFace
    from sim_arm import SimArmRobot

    class SimCup(SimArmRobot, CupRobot):
        """The cup's scene (cameras, SAM3) on the real driver and planner."""

    SC = CupScene()

    class TestDrinking(Drinking):
        sim = None

        def _robot(self, check):
            ros = Robot(node=self, group=self._group, roles=(), check=check, static_tf=lambda m: None,
                        reset=self._reset, token=self._token)
            if TestDrinking.sim is None:                # one fake arm across actions: it stays where it stopped
                TestDrinking.sim = SimCup(SC, ros, captured("home"))
                self._face = FakeFace(TestDrinking.sim)
            TestDrinking.sim.ros = ros
            return TestDrinking.sim

        def _grip(self, position, speed=None, force=None):
            """The driver's gripper, and where the handle stops the fingers (the driver's --sim has no handle)."""
            super()._grip(position, speed, force)
            return 0.5 if position == 0.0 else 1.0

    engine = TestDrinking()
    client = rclpy.create_node("e2e_orchestrator")
    status, resets, grips = [], [], []
    client.create_subscription(String, "/drinking/status", lambda m: status.append(json.loads(m.data)), 10)
    client.create_subscription(Empty, "/reset", lambda m: resets.append(time.time()), 10)
    from rammp_arm_interfaces.msg import GripperSetpoint
    from rclpy.qos import qos_profile_sensor_data
    client.create_subscription(GripperSetpoint, "/setpoint/gripper", lambda m: grips.append(round(m.position, 2)),
                               qos_profile_sensor_data)
    ex = MultiThreadedExecutor()
    ex.add_node(engine)
    ex.add_node(client)
    threading.Thread(target=ex.spin, daemon=True).start()
    act = ActionClient(client, ExecuteAction, "/drinking/execute_action")
    from rammp_curobo_interfaces.action import PlanToPose
    from rammp_curobo_interfaces.srv import SetWorld
    plan, world = ActionClient(client, PlanToPose, "/rammp_curobo/plan_to_pose"), client.create_client(SetWorld,
                                                                                                       "/rammp_curobo/set_world")
    t_end = time.time() + 600                           # the planner compiles its kernels on a first run
    while not (act.wait_for_server(timeout_sec=5.0) and plan.wait_for_server(timeout_sec=5.0)
               and world.wait_for_service(timeout_sec=5.0)):
        assert time.time() < t_end, "the engine or the planner did not come up"
    time.sleep(1.0)
    idle_subs = client.count_subscribers("/joint_states")

    def run(action_id, cancel_when=None):
        while engine._current is not None:
            time.sleep(0.1)
        n_reset = len(resets)
        sent = act.send_goal_async(ExecuteAction.Goal(engine_id="drinking", action_id=action_id, params_json="{}"))
        while not sent.done():
            time.sleep(0.05)
        h = sent.result()
        assert h.accepted, "%s rejected" % action_id
        t0, res, t_cancel = time.time(), h.get_result_async(), None
        if cancel_when is not None:
            while not cancel_when(t0):
                assert not res.done(), "%s ended before its cancel" % action_id
                time.sleep(0.05)
            t_cancel = time.time()
            h.cancel_goal_async()
        while not res.done():
            time.sleep(0.05)
            assert time.time() - t0 < 120, "%s ran past the orchestrator's 120 s" % action_id
        t_done = time.time()
        r = res.result().result.result
        time.sleep(0.5)
        assert client.count_subscribers("/joint_states") == idle_subs, "%s left a subscription behind" % action_id
        print("  %-15s status %d %-16s %s in %.1f s%s" % (action_id, r.status, r.error_code, r.payload_json,
              t_done - t0, "" if t_cancel is None else ", %.1f s after the cancel" % (t_done - t_cancel)), flush=True)
        return r, len(resets) - n_reset, None if t_cancel is None else t_done - t_cancel

    from rammp_common_interfaces.srv import AcquireControl
    token = list(range(1, 17))
    acq = client.create_client(AcquireControl, "/drinking/acquire_control")
    assert acq.wait_for_service(timeout_sec=10.0), "no ~/acquire_control"
    f = acq.call_async(AcquireControl.Request(owner_id="drinking", token=token))
    while not f.done():
        time.sleep(0.05)
    S = Result

    r, n_reset, _ = run("pick_up_drink")
    assert r.status == S.STATUS_SUCCESS, r
    assert json.loads(r.payload_json)["holding"] and config.session()["holding"]
    assert engine.sim.ros.token == token, "the action's arm goals do not carry the orchestrator's token"
    assert n_reset == 1
    assert grips and grips[0] == 1.0 and grips[-1] == 0.0, grips[:3] + grips[-3:]   # closed in, opened to hold
    sim = engine.sim
    print("    (the cup picked up, the gripper %s; %d trajectories to the driver)" % (sorted(set(grips)), sim.accepted))

    r, n_reset, _ = run("give_sip")
    assert r.status == S.STATUS_SUCCESS and json.loads(r.payload_json) == {"sip_given": True}, r
    assert n_reset == 1

    from rammp_drinking.config import cfg
    sip_at = dict(t=None)
    real_sip = float(cfg.drinking.sip_s)
    cfg["drinking"]["sip_s"] = 20.0                     # (long enough to cancel in the middle of it)

    def at_mouth(t0):
        T, R = sim.kin.fk(sim.joints())
        tip = T + R @ np.asarray(cfg.drinking.mug.tip_in_tool_m, float)
        near = float(np.linalg.norm(tip - np.asarray(SC.mouth, float))) < float(cfg.delivery.stop_m) + 0.01
        if near and sip_at["t"] is None:
            sip_at["t"] = time.time()
        return sip_at["t"] is not None and time.time() > sip_at["t"] + 1.0
    r, _, after_cancel = run("give_sip", cancel_when=at_mouth)
    cfg["drinking"]["sip_s"] = real_sip
    assert r.status == S.STATUS_CANCELLED, r
    assert after_cancel < 5.0, "the cancel took %.1f s" % after_cancel
    assert config.session()["holding"], "a cancelled sip forgot the cup"

    r, n_reset, _ = run("put_down_drink")
    assert r.status == S.STATUS_SUCCESS and json.loads(r.payload_json) == {"put_down": True}, r
    assert config.session() == {} and grips[-1] == 1.0
    (T, R), (Th, Rh) = sim.kin.fk(sim.joints()), sim.kin.fk(captured("home"))
    assert np.linalg.norm(T - Th) < 0.01, "not home"
    assert {s["state"] for s in status} >= {"idle", "executing"} and status[-1]["state"] == "idle", status[-3:]
    assert sim.accepted == len(sim.moves), (sim.accepted, len(sim.moves))
    print("E2E DRINKING ENGINE THROUGH THE REAL ARM STACK (%d moves, each goal accepted by the driver; level plans "
          "by the planner; %d gripper setpoints; every action's subscriptions gone as it ended)"
          % (len(sim.moves), len(grips)), flush=True)
    ex.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
