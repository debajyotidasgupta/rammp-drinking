"""Drinking's actions, on rammp_feeding's guarded arm (rammp_feeding.arm.Arm, holding a Gripper or a Mug) and its
delivery. `grip(position)` commands the gripper (0 open .. 1 closed; the engine: capability_engine's self.arm.gripper);
`step(name, fraction, message)` reports each stage.

  pick_up   the fingers closed (they go into the handle); a planned path to drink_gaze; the handle found (handle.find):
            the table's top and the grasp's poses from it; a planned path to grasp.rise_m above in front of the handle
            (the cup in the planner's world), straight down in front of it; straight into the handle, up against its
            top, the fingers opened (the hold, as the old
            system's); straight up with the cup; a planned path to drink_hold, carrying it upright (every path that
            carries the cup is the planner's HOLD_LEVEL: its tilt held all the way). The session (drink_state.json)
            keeps the handle and the joints where the cup was lifted, for put_down
  give_sip  a planned path to drink_deliver carrying the cup; rammp_feeding's presentation (a face, an open mouth, the
            mouth measured once as it opens, one straight move to delivery.stop_m short of it); the cup held there
            drinking.sip_s; back (straight to drink_deliver, a planned path to drink_hold). 7: no open mouth (back)
  put_down  a planned path to where the cup was lifted; straight down to where it was picked up; the fingers closed (it
            lets go); straight down out of the handle, back out in front of it and up grasp.rise_m; a planned path home
  recover   after an action stopped mid-way: at the user, straight back to drink_deliver; the fingers in the handle with
            the cup still standing, straight back out in front of it
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Callable

import numpy as np

from rammp_drinking.config import D, captured, cfg, save_session, session
from rammp_drinking.handle import Handle, find, grasp_poses
from rammp_drinking.mug import Gripper, Mug, table_z
from rammp_feeding import record
from rammp_feeding.arm import Arm, MoveError
from rammp_feeding.geometry import axis, max_step

log = logging.getLogger("rammp")
Step = Callable[[str, float, str], None]


def _no_step(name: str, fraction: float, message: str) -> None:
    log.info("  [%s] %s", name, message)


def _held(s: dict) -> tuple[Handle | None, float | None]:
    return (Handle.from_json(s["handle"]) if s.get("handle") else None), s.get("table_z")


def fingers(a) -> float | None:
    """Where the fingers stopped (0 open .. 1 closed), grasp.settle_s after a command, from the arm node's
    /gripper_state through the engine library's ArmInterface a; None when it serves none."""
    time.sleep(float(D().grasp.settle_s))
    try:
        s = a.gripper_state()
        return float(s.position) if s is not None and s.present else None
    except Exception:  # noqa: BLE001
        return None


def pick_up(robot, arm: Arm, grip: Callable[[float], None], step: Step = _no_step, out: Path | None = None) -> dict:
    G = D().grasp
    if session().get("holding"):
        raise MoveError("the gripper already holds the cup -- put it down first")
    arm.hold(Gripper())
    step("gaze", 0.05, "Looking for the cup")
    grip(float(G.gripper_closed))
    arm.travel(captured("drink_gaze"), "to drink_gaze", pose=True)   # (its pose: the planner picks the arm's shape)
    to_gaze = arm.last_run
    step("find", 0.2, "Finding the cup's handle")
    h = find(robot, arm, out)
    tz, P = table_z(h), grasp_poses(h)
    arm.hold(Gripper(tz, h))
    step("grasp", 0.35, "Picking up the cup")
    above = (P["pre"][0] + np.array([0.0, 0.0, float(G.rise_m)]), P["pre"][1])
    sols = [q for q in (arm.ik(*above, s) for s in [arm.q(), *G.ik_seeds_deg]) if q is not None]
    roomy = [q for q in sols if arm.margin(q) >= float(G.min_limit_margin_deg)]   # (the planner refuses a goal near
    q_pre = (min(roomy, key=lambda q: max_step(arm.q(), q)) if roomy               # a joint's limit); the nearest arm
             else max(sols, key=arm.margin, default=None))                         # shape: no swing over the top
    if q_pre is None or arm.margin(q_pre) < float(G.min_limit_margin_deg):
        raise MoveError("in front of the handle is out of the arm's reach%s -- the cup further in front of the arm"
                        % ("" if q_pre is None else " (a joint %.1f deg from its limit, at least %.0f)"
                           % (arm.margin(q_pre), float(G.min_limit_margin_deg))))
    arm.travel(q_pre, "above in front of the handle")
    to_pre = arm.last_run
    arm.straight(*P["pre"], float(G.lift_mps), "down in front of the handle")
    arm.straight(*P["inside_bottom"], float(G.approach_mps), "into the handle", torque_stop=False)
    arm.straight(*P["inside_top"], float(G.approach_mps), "up against the handle", torque_stop=False)
    at = grip(float(G.gripper_open), float(G.open_speed), float(G.open_force))   # open inside the handle: the hold
    if at is None:
        log.info("    the arm node serves no gripper position: the hold is not checked")
    elif at < float(G.gripper_open) + float(G.lock_margin):
        grip(float(G.gripper_closed))                    # nothing stopped them: not in the handle -- back out, empty
        arm.straight(*P["inside_bottom"], float(G.approach_mps), "down in the handle", torque_stop=False)
        arm.straight(*P["pre"], float(G.approach_mps), "back out of the handle", torque_stop=False)
        raise MoveError("missed the handle: the fingers opened fully (%.2f) -- closed again and backed out" % at)
    else:
        log.info("    the fingers stopped at %.2f (open %.2f): the handle between them", at, float(G.gripper_open))
    s = dict(holding=True, handle=h.to_json(), table_z=tz, t=time.time(), to_gaze=to_gaze, to_pre=to_pre)
    save_session(s)                                      # (held from here: a later stop puts it down from here on)
    arm.hold(Mug(h, tz))
    arm.straight(*P["lifted"], float(G.lift_mps), "lift the cup", torque_stop=False)
    s["lifted_joints"] = [float(v) for v in arm.q()]
    save_session(s)
    step("hold", 0.8, "Holding the cup")
    arm.travel(captured("drink_hold"), "to drink_hold", carrying=True, level=True, short_way=True)
    s["to_hold"] = arm.last_run
    save_session(s)
    record.event("picked_up", handle=h.to_json(), table_z=tz)
    return dict(holding=True, cup_xyz_m=[round(float(v), 3) for v in h.p])


def back(robot, arm: Arm, way=None) -> None:
    """From in front of drink_deliver (at the user) straight back to it, then to drink_hold: back along the way it came
    (way: the swing's timed path), else a planned path."""
    T0, _ = arm.tool()
    Te, Re = arm.tool(captured("drink_deliver"))
    if float((T0 - Te) @ axis(Re)) > float(cfg.motion.arrived_m):
        arm.straight(Te, Re, float(cfg.delivery.retract.back_off_mps), "back to drink_deliver", torque_stop=False)
    if not arm.retrace(way, "back to drink_hold"):
        arm.travel(captured("drink_hold"), "to drink_hold", carrying=True, level=True, short_way=True)


def give_sip(robot, arm: Arm, face, step: Step = _no_step, out: Path | None = None, wait_open: bool = True,
             open_timeout_s: float | None = None) -> int:
    """0: the sip given; 7: the user did not open their mouth (the cup taken back)."""
    from rammp_feeding.deliver import present
    s = session()
    if not s.get("holding"):
        raise MoveError("no cup in the gripper -- pick it up first")
    arm.hold(Mug(*_held(s)))
    step("swing", 0.1, "Bringing the drink over")
    arm.travel(captured("drink_deliver"), "to drink_deliver", carrying=True, level=True, short_way=True)
    way = arm.last_run
    step("present", 0.3, "Waiting for an open mouth")
    try:
        rc = present(robot, arm, face, out, wait_open, open_timeout_s)
    except MoveError:
        step("back", 0.85, "Taking the drink back")
        back(robot, arm, way)
        raise
    if rc == 7:
        log.info("    no open mouth -- not approaching; taking the drink back")
        step("back", 0.85, "Taking the drink back")
        back(robot, arm, way)
        return 7
    step("sip", 0.6, "Holding the drink for a sip")
    t_end = time.time() + float(D().sip_s)
    while time.time() < t_end:                           # (a cancel ends it: the check every wait polls)
        robot.check()
        time.sleep(0.05)
    step("back", 0.85, "Taking the drink back")
    back(robot, arm, way)
    record.event("sip", seconds=float(D().sip_s))
    return 0


def put_down(robot, arm: Arm, grip: Callable[[float], None], step: Step = _no_step) -> dict:
    G = D().grasp
    s = session()
    if not s.get("holding"):
        raise MoveError("no cup in the gripper")
    h, tz = _held(s)
    if h is None:
        raise MoveError("where the cup was picked up is not known -- put it down by hand (rammp-drink gripper close)")
    P = grasp_poses(h)
    arm.hold(Mug(h, tz))
    step("place", 0.1, "Taking the cup back to where it stood")
    q_lift = s.get("lifted_joints") or arm.ik(*P["lifted"], arm.q())
    if q_lift is None:
        raise MoveError("over where the cup stood is out of the arm's reach")
    if not arm.retrace(s.get("to_hold"), "over where the cup stood"):
        arm.travel(q_lift, "over where the cup stood", carrying=True, level=True)
    arm.straight(*P["inside_top"], float(G.lift_mps), "set the cup down", torque_stop=False)
    grip(float(G.gripper_closed))                        # the fingers close: it lets go
    save_session({})
    arm.hold(Gripper(tz, h))
    arm.straight(*P["inside_bottom"], float(G.approach_mps), "down out of the handle", torque_stop=False)
    arm.straight(*P["pre"], float(G.approach_mps), "back out of the handle", torque_stop=False)
    arm.straight(P["pre"][0] + np.array([0.0, 0.0, float(G.rise_m)]), P["pre"][1], float(G.lift_mps), "up from the cup")
    step("home", 0.7, "Going home")
    if not (arm.retrace(s.get("to_pre"), "back to drink_gaze") and arm.retrace(s.get("to_gaze"), "home")):
        arm.travel(captured("home"), "home")
    record.event("put_down", handle=h.to_json())
    return dict(put_down=True)


def home(robot, arm: Arm) -> None:
    arm.travel(captured("home"), "home")


def recover(robot, arm: Arm, last: Handle | None) -> None:
    """Clear of the user and of the cup's handle before a planned move, after an action stopped mid-way (the planner
    refuses a start inside its world): at the user, back to drink_deliver; the empty fingers in the handle of the cup
    `last` (still standing), straight back out in front of it."""
    s = session()
    if s.get("holding"):
        T0, _ = arm.tool()
        try:
            Te, Re = arm.tool(captured("drink_deliver"))
        except FileNotFoundError:
            return
        if float((T0 - Te) @ axis(Re)) > float(cfg.motion.arrived_m) and np.linalg.norm(T0 - Te) < 0.5:
            arm.straight(Te, Re, float(cfg.delivery.retract.back_off_mps), "back to drink_deliver", torque_stop=False)
        return
    if last is not None:
        T0, _ = arm.tool()
        x = last.R.T @ (T0 - last.p)                     # the tool in the handle's frame
        if abs(x[1]) < 0.06 and -0.05 < x[0] < 0.30 and 0.0 < x[2] < 0.12:   # (in front of the handle, closer than pre)
            G, P = D().grasp, grasp_poses(last)
            arm.straight(*P["pre"], float(G.approach_mps), "back out of the handle", torque_stop=False)
            arm.straight(P["pre"][0] + np.array([0.0, 0.0, float(G.rise_m)]), P["pre"][1], float(G.lift_mps),
                         "up from the cup")
