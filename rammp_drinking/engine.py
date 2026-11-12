"""rammp_drinking as a RAMMP capability engine (rammp-kitchen-capability-engines' CapabilityEngine): registered with the
orchestrator, it runs an action only when the orchestrator starts one -- nothing moves otherwise.

  pick_up_drink   the wrist camera finds the cup's handle from drink_gaze; the gripper picks the cup up by it and holds it
                  at drink_hold
  give_sip        the cup to the user's mouth (an open mouth, delivery.stop_m short of it), held there sip_s, and back
  put_down_drink  the cup set back where it was picked up, the gripper lets go, and the arm goes home

The gripper is commanded here (open holds the handle from inside, closed lets go: the old system's grasp), through the
engine library's self.arm. Each action builds its own ros.Robot on this node (rammp_feeding's): its subscriptions,
clients and publishers live only while it runs. Our world goes into the shared planner while the action plans and the
planner's own comes back as it ends. A cancel stops the move in flight and ends the action as cancelled; the next one
first backs away from the user or out of the handle (actions.recover).

    ros2 launch rammp_drinking drinking.launch.py                          # with the orchestrator
    ros2 launch rammp_drinking drinking.launch.py use_orchestrator:=false  # standalone
"""

from __future__ import annotations

import logging

from rammp_drinking import config  # noqa: I001  (first: rammp_feeding loads with drinking's defaults)
from rammp_drinking.config import PARAMS, apply_parameters

from capability_engine import Action, Cancelled, CapabilityEngine, EngineError, run
from rammp_orchestrator_interfaces.msg import ActionAvailability

log = logging.getLogger("rammp")
CODES = {"gaze": ("pickup_failed", False), "find": ("cup_not_found", True), "grasp": ("pickup_failed", False),
         "hold": ("pickup_failed", False), "swing": ("delivery_failed", False), "present": ("delivery_failed", False),
         "sip": ("delivery_failed", False), "back": ("delivery_failed", False), "place": ("putdown_failed", False),
         "home": ("move_failed", False)}


class Drinking(CapabilityEngine):
    display_name = "Drinking"
    perception_classes = ["cup", "mug"]
    required_resources = ["arm"]
    actions = [
        Action("pick_up_drink", "Pick up the drink", sticky_followup=True),
        Action("give_sip", "Take a sip", sticky_followup=True),
        Action("put_down_drink", "Put the drink down"),
    ]

    def __init__(self, engine_id: str = "drinking"):
        super().__init__(engine_id)
        values = {name: self.declare_parameter(name, default).value for name, (default, _) in PARAMS.items()}
        apply_parameters(values)
        from rammp_feeding import record
        record.setup_logging()
        for level, line in config.state_report():
            log.log(level, "  %s", line)
        from std_msgs.msg import Empty
        self._reset = self.create_publisher(Empty, str(config.cfg.ros.planner_reset), 1)
        from rammp_common_interfaces.srv import AcquireControl, ReleaseControl
        self._token = [0] * 16                               # the orchestrator's execution token: on every arm goal
        self.create_service(AcquireControl, "~/acquire_control", self._on_acquire_control, callback_group=self._group)
        self.create_service(ReleaseControl, "~/release_control", self._on_release_control, callback_group=self._group)
        self._face = None                                    # MediaPipe, loaded by the first sip and kept
        self._last = None                                    # the handle last found (recover: out of it)
        self._mock_holding = False                           # (mock: no state written)

    def _holding(self) -> bool:
        return self._mock_holding if self.mock else bool(config.session().get("holding"))

    # ------------------------------------------------------------------ the engine contract
    def availability(self, instance_id, world_state):
        """Fast, no side effects: a cup in the gripper (drink_state.json) or not."""
        holding = self._holding()
        return [ActionAvailability(action_id="pick_up_drink", available=not holding,
                                   reason="the gripper already holds the cup" if holding else ""),
                ActionAvailability(action_id="give_sip", available=holding,
                                   reason="" if holding else "pick up the drink first"),
                ActionAvailability(action_id="put_down_drink", available=holding,
                                   reason="" if holding else "the gripper holds no cup")]

    def _on_acquire_control(self, request, response):
        """The orchestrator hands the execution token over before it starts an action: it goes on every arm goal and
        gripper command of that action, with this engine's id as the sender."""
        self._token = [int(b) for b in request.token]
        response.accepted, response.token, response.message = True, list(self._token), "ok"
        return response

    def _on_release_control(self, request, response):
        """...and takes it back once the action has ended."""
        if [int(b) for b in request.token] == self._token:
            self._token = [0] * 16
            response.released, response.message = True, "released"
        else:
            response.released, response.message = False, "not the token this engine holds"
        return response

    def execute(self, action_id, params, ctx):
        if self.mock:                                        # (no hardware: the actions' results, nothing moves)
            return self._mock(action_id, ctx)
        from rammp_drinking import actions
        from rammp_feeding import scene
        from rammp_feeding.arm import Arm, MoveError
        from rammp_feeding.ros import RobotError
        if config.feeding_config.UNREADABLE or config.unmeasured():
            raise EngineError("not_set_up", "; ".join(config.feeding_config.UNREADABLE or
                                                        ["not measured: " + ", ".join(config.unmeasured())]),
                              recoverable=False)
        holding = self._holding()
        if action_id == "pick_up_drink" and holding or action_id != "pick_up_drink" and not holding:
            raise EngineError("already_holding" if holding else "not_holding",
                              "the gripper already holds the cup" if holding else "the gripper holds no cup",
                              recoverable=False)

        def check():                                         # polled by every wait and every move in flight
            if ctx.cancelled():
                raise Cancelled()
        stage = ["starting"]

        def step(name, pct, message):
            stage[0] = name
            ctx.checkpoint()                                 # pause and cancel between steps
            ctx.progress(name, pct, message, pausable=False)
            log.info("  [%s] %s", action_id, message)

        def grip(position):
            if not self.arm.gripper(float(position)):
                raise MoveError("the gripper: %s" % (self.arm.last_error or "refused"))
        robot = None
        try:
            self.arm.begin_action(ctx, bytes(self._token))   # (the gripper's commands carry the token too)
            robot = self._robot(check)
            step("setup", 0.0, "Getting the arm ready")
            from rammp_drinking.mug import Gripper
            arm = Arm(robot, held=Gripper())
            actions.recover(robot, arm, self._last)
            if action_id == "pick_up_drink":
                out = actions.pick_up(robot, arm, grip, step)
                self._last = None
                return out
            if action_id == "give_sip":
                if self._face is None:
                    from rammp_feeding.face import Face
                    self._face = Face()
                rc = actions.give_sip(robot, arm, self._face, step,
                                      open_timeout_s=float(config.cfg.node.open_mouth_timeout_s))
                return {"sip_given": True} if rc == 0 else {"sip_given": False, "declined": True}
            out = actions.put_down(robot, arm, grip, step)
            return out
        except Cancelled:
            raise
        except (MoveError, RobotError, RuntimeError, OSError, ValueError) as e:
            code, recoverable = CODES.get(stage[0], ("move_failed", False))
            if isinstance(e, FileNotFoundError):
                code, recoverable = "not_set_up", False
            elif isinstance(e, RobotError):
                code, recoverable = "robot_unavailable", True
            elif stage[0] == "present" and "mouth" in str(e):
                code, recoverable = "presentation_refused", True
            raise EngineError(code, str(e), recoverable=recoverable)
        finally:
            if robot is not None:
                if getattr(robot, "world_pushed", False):     # the shared planner's own world back, for its other users
                    try:
                        scene.restore_world(robot)
                    except Exception as e:  # noqa: BLE001
                        log.error("  the planner's own world was not restored (%s): it still holds ours", e)
                robot.close()

    def _mock(self, action_id, ctx) -> dict:
        ctx.checkpoint()
        ctx.progress(action_id, 1.0, "Done (mock)", pausable=False)
        self._mock_holding = action_id == "pick_up_drink" or (action_id == "give_sip" and self._mock_holding)
        return {"pick_up_drink": {"holding": True}, "give_sip": {"sip_given": True},
                "put_down_drink": {"put_down": True}}[action_id]

    # ------------------------------------------------------------------ the robot, per action
    def _robot(self, check):
        from rammp_feeding.ros import Robot
        return Robot(node=self, group=self._group, roles=("camera",), check=check, static_tf=lambda msg: None,
                     reset=self._reset, token=self._token)


def main():
    run(Drinking)


if __name__ == "__main__":
    main()
