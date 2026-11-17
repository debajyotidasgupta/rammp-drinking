"""rammp-drink: the drinking module's steps by hand (setting up a site, checking it, running a step on its own). Never
alongside sheppy's drinking node: one program commands the arm at a time.

    rammp-drink check                   every node, measurement and capture the module needs; never moves
    rammp-drink state                   where the arm is, the gripper, and the cup this session; never moves
    rammp-drink capture NAME            the arm's joints now as that pose (home | drink_gaze | drink_hold | drink_deliver);
                                        the two that hold the cup are levelled exactly first (its tilt within level_tol_deg)
    rammp-drink gripper open|close      the gripper (a cup put in or taken out by hand: open holds a handle from inside)
    rammp-drink find-handle             the cup's handle from where the arm is (at drink_gaze); never moves
    rammp-drink pick                    pick the cup up (actions.pick_up)
    rammp-drink sip [--no-wait-open]    one sip (actions.give_sip)
    rammp-drink put-down                set the cup back and go home (actions.put_down)
    rammp-drink home                    a planned path home (the gripper empty)
"""

from __future__ import annotations

import argparse
import logging
import sys

from rammp_drinking import config  # noqa: I001  (first: rammp_feeding loads with drinking's defaults)
from rammp_drinking.config import cfg

import numpy as np

log = logging.getLogger("rammp")
COMMANDS = ("check", "state", "capture", "gripper", "find-handle", "pick", "sip", "put-down", "home")


class _Ctx:
    """The engine library's action context, by hand: never cancelled (Ctrl-C stops the command)."""

    def cancelled(self) -> bool:
        return False

    def checkpoint(self) -> None:
        pass


def gripper(robot):
    """(grip(position), close()): the gripper through the engine library's ArmInterface, on this command's node."""
    from capability_engine.arm import ArmInterface
    from rammp_drinking import actions
    from rammp_feeding.arm import MoveError
    from rclpy.callback_groups import ReentrantCallbackGroup
    a = ArmInterface(robot.node, ReentrantCallbackGroup(), standalone=True)
    a.begin_action(_Ctx(), bytes(robot.token))

    def grip(position: float, speed: float | None = None, force: float | None = None) -> float | None:
        if not a.gripper(float(position), float(config.D().grasp.gripper_timeout_s), force=force, speed=speed):
            raise MoveError("the gripper: %s" % (a.last_error or "refused"))
        return actions.fingers(a)
    return grip, a.end_action


def report(robot, arm) -> None:
    q = arm.q()
    T, R = arm.tool(q)
    log.info("  joints  %s", " ".join("%7.2f" % v for v in q))
    log.info("  tool    (%.4f %.4f %.4f); %s %.1f deg off vertical", *T, arm.held_name, arm.tilt(R))
    s = config.session()
    log.info("  cup     %s", "held (picked up at %s)" % np.round(s["handle"]["p"], 3).tolist()
             if s.get("holding") and s.get("handle") else "held" if s.get("holding") else "not held")


def check(robot, arm) -> int:
    """What the module needs from the stack, each as it is served, and from the site. Never moves."""
    from rammp_feeding.config import state_dir
    bad = 0

    def row(ok, name, text, warn=False):
        nonlocal bad
        bad += not ok and not warn
        log.info("  %-4s %-14s %s", "OK" if ok else "WARN" if warn else "FAIL", name, text)
    try:
        report(robot, arm)
        row(not robot.estopped, "arm", "e-stop ENGAGED" if robot.estopped else "joint states streaming, e-stop clear")
    except Exception as e:  # noqa: BLE001
        row(False, "arm", str(e))
    row(True, "robot model", "%s, rooted at %s" % (cfg.ros.robot_description, arm.kin.base))
    for name, client in (("trajectories", robot._exec), ("planner", robot._plan)):
        row(client.wait_for_server(timeout_sec=2.0), name, "action server up")
    for name, client in (("planner world", robot._world), ("sam3", robot._detect)):
        row(client.wait_for_service(timeout_sec=2.0), name, "service up")
    row(robot.planner_listens(), "planner reset", "the planner listens on %s (its own world back after ours)"
        % cfg.ros.planner_reset)
    missing = config.unmeasured()
    row(not missing, "measured", "the cup, the table and the wrist camera's mount" if not missing else
        "not yet: %s (local.yaml; the wrist camera's mount: rammp_feeding's wrist_mount.json, docs/porting.md)"
        % ", ".join(missing))
    poses = [n for n in cfg.drinking.poses if not (state_dir() / (n + ".json")).exists()]
    row(not poses, "poses", "captured: %s" % ", ".join(cfg.drinking.poses) if not poses else
        "not captured: %s (rammp-drink capture)" % ", ".join(poses))
    for u in config.feeding_config.UNREADABLE:
        row(False, "state file", "unreadable, ignored: %s" % u)
    for role, cam in robot.cameras.items():
        try:
            import time
            cam.frame(wait_s=5.0)
            cam.depth_median(3, time.time() - 1.0, wait_s=5.0, min_frames=1)
            m = robot.camera_model(role)
            row(True, role, "colour + depth streaming; camera_info %dx%d" % (m.colour.width, m.colour.height))
        except Exception as e:  # noqa: BLE001
            row(False, role, str(e))
    return 1 if bad else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="rammp-drink", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=COMMANDS)
    ap.add_argument("name", nargs="*", help="capture: " + " | ".join(cfg.drinking.poses) + "; gripper: open | close")
    ap.add_argument("--no-wait-open", action="store_true", help="sip: no wait for an open mouth")
    a, _ = ap.parse_known_args(argv)               # (ros2 run appends its own --ros-args)
    if a.command == "capture" and (len(a.name) != 1 or a.name[0] not in cfg.drinking.poses):
        ap.error("rammp-drink capture " + " | ".join(cfg.drinking.poses))
    if a.command == "gripper" and (len(a.name) != 1 or a.name[0] not in ("open", "close")):
        ap.error("rammp-drink gripper open | close")
    import signal

    def _stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _stop)           # docker stop: as Ctrl-C, the move in flight cancelled
    from rammp_drinking import actions
    from rammp_drinking.mug import Gripper, Mug
    from rammp_feeding import record, scene
    from rammp_feeding.arm import Arm, MoveError
    from rammp_feeding.ros import Robot
    record.setup_logging()
    record.session("drink_" + a.command.replace("-", "_"))
    robot = Robot(name="rammp_drinking", roles=("camera",))
    rc, end = 0, None
    try:
        c = a.command
        arm = Arm(robot, held=Gripper(camera=c in ("find-handle", "pick", "sip")))   # (the wrist camera where it is used)
        s = config.session()
        if s.get("holding") and not config.unmeasured():
            arm.hold(Mug(*actions._held(s)))
        if c in ("gripper", "pick", "put-down"):
            grip, end = gripper(robot)
        if c == "check":
            rc = check(robot, arm)
        elif c == "state":
            report(robot, arm)
        elif c == "capture":
            q = arm.q()
            if a.name[0] in ("drink_hold", "drink_deliver"):  # holding the cup: level, as the grasp holds it (the
                from rammp_drinking.handle import _level      # planner's HOLD_LEVEL keeps the tilt it starts with)
                T, R = arm.tool(q)
                Rl = _level(R)
                off = float(np.degrees(np.arccos(np.clip((np.trace(Rl.T @ R) - 1) / 2, -1, 1))))
                if off > float(cfg.drinking.level_tol_deg):
                    raise MoveError("the gripper is %.0f deg from level with the cup upright (at most %.0f): pose it "
                                    "level and capture again" % (off, float(cfg.drinking.level_tol_deg)))
                q = arm.ik(T, Rl, q)
                if q is None:
                    raise MoveError("no joints hold this pose level -- pose it a little differently")
                log.info("  levelled by %.1f deg", off)
            p = config.capture(a.name[0], q)
            log.info("  captured %s -> %s", a.name[0], p)
        elif c == "gripper":
            G = cfg.drinking.grasp
            grip(float(G.gripper_open if a.name[0] == "open" else G.gripper_closed))
        elif c == "find-handle":
            from rammp_drinking.handle import find, grasp_poses
            h = find(robot, arm, record.stage("find"))
            for k, (T, R) in grasp_poses(h).items():
                log.info("    %-14s tool (%.3f %.3f %.3f)%s", k, *T, "" if arm.ik(T, R, arm.q()) else "  OUT OF REACH")
        elif c == "pick":
            log.info("  %s", actions.pick_up(robot, arm, grip, out=record.stage("pick")))
        elif c == "sip":
            from rammp_feeding.face import Face
            rc = actions.give_sip(robot, arm, Face(), out=record.stage("sip"), wait_open=not a.no_wait_open)
        elif c == "put-down":
            log.info("  %s", actions.put_down(robot, arm, grip))
        elif c == "home":
            if config.session().get("holding"):
                raise MoveError("the gripper holds the cup -- rammp-drink put-down")
            actions.home(robot, arm)
        if c in ("pick", "sip", "put-down", "home"):
            report(robot, arm)
    except (MoveError, RuntimeError, OSError, ValueError) as e:      # (FileNotFoundError is an OSError)
        log.error("\n  STOPPED: %s", e)
        record.event("stopped", command=a.command, reason=str(e))
        rc = 1
    except KeyboardInterrupt:
        log.error("\n  interrupted -- the move in flight was cancelled")
        rc = 1
    finally:
        if end is not None:
            end()
        if getattr(robot, "world_pushed", False):  # the shared planner's own world back, for its other clients
            try:
                scene.restore_world(robot)
            except Exception as e:  # noqa: BLE001
                log.error("  the planner's own world was not restored (%s): it still holds ours", e)
        robot.close()
    record.event("done", command=a.command, rc=rc)
    return rc


if __name__ == "__main__":
    sys.exit(main())
