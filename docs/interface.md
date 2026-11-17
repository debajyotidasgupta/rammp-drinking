# rammp_drinking: interface

**Rung:** ad-hoc · **Tier:** experimental

## What it does

One node, a capability engine (`rammp-kitchen-capability-engines`' `CapabilityEngine`, engine id `drinking`). It
registers with the orchestrator and moves the arm only in an action the orchestrator starts, at the user's request:
pick the cup up, give one sip, put the cup down. Nothing moves when it starts, and nothing moves between actions.

## Publishes

| Topic | Type | Rate | Meaning |
| --- | --- | --- | --- |
| `/drinking/status` | `std_msgs/String` | 1 Hz and on change | Health: JSON `state` (idle, executing, paused), `action`, `instance_id`, `mode`, `registered`, `mock` |
| `/setpoint/gripper` | `rammp_arm_interfaces/GripperSetpoint` | 50 Hz while it moves the fingers | The gripper, through the engine library (`self.arm.gripper`): 1 closed (into the handle, and letting go), 0 open (holding it from inside) |
| `/reset` | `std_msgs/Empty` | once per action | The planner's own world back, as each action ends |

## Subscribes

Only while an action runs (each action's subscriptions are destroyed as it ends).

| Topic | Type | Required | Meaning |
| --- | --- | --- | --- |
| `/arm/robot_description` | `std_msgs/String` | yes | The arm's model, rooted at `arm_mount` |
| `/joint_states` | `sensor_msgs/JointState` | yes | `arm_joint_1..7` |
| `/control_status` | `rammp_common_interfaces/ControlStatus` | yes | The e-stop |
| `/gripper_state` | `rammp_arm_interfaces/GripperState` | yes | The fingers, while the gripper moves |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | yes | The wrist camera's frames |
| `/wrist_camera/color/image_raw`, `.../depth/image_rect_raw`, `.../color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | yes | The wrist camera (topics: the selected camera profile) |

## Calls

| Name | Type | Served by | Use |
| --- | --- | --- | --- |
| `/execute_joint_trajectory` | `rammp_arm_interfaces/action/ExecuteJointTrajectory` | `arm` | Every move, in position control; cancelled to stop |
| `/rammp_curobo/plan_to_joints` | `rammp_curobo_interfaces/action/PlanToJoints` | `planner` | Free-space paths with the gripper empty |
| `/rammp_curobo/plan_to_pose` | `rammp_curobo_interfaces/action/PlanToPose` | `planner` | Every path that carries the cup, `HOLD_LEVEL` |
| `/rammp_curobo/set_world` | `rammp_curobo_interfaces/srv/SetWorld` | `planner` | Our world while an action plans: `/config/drink_world.yaml`, read by the planner at `/feeding/drink_world.yaml` |
| `/perception/sam3/detect` | `rammp_perception_interfaces/srv/Detect` | `sam3` | The cup's handle (boxes and masks) |

## Orchestrator interface

Served under `/drinking/`, types from `rammp_orchestrator_interfaces`: `activate`, `execute_action`, `control`,
`hibernate`; and `acquire_control`, `release_control` (`rammp_common_interfaces/srv/AcquireControl`,
`ReleaseControl`). It registers through `/rammp_orchestrator/register_engine`. Activated by the perception classes
`cup` and `mug`. Locks the `arm`.

| Action | Label | Available | Result payload |
| --- | --- | --- | --- |
| `pick_up_drink` | Pick up the drink | the gripper holds no cup | `holding`, `cup_xyz_m` |
| `give_sip` | Take a sip | the gripper holds the cup | `sip_given`; `declined` when the user did not open their mouth |
| `put_down_drink` | Put the drink down | the gripper holds the cup | `put_down` |

Whether the gripper holds the cup is `drink_state.json` in the state folder: written as the gripper takes the cup,
removed as it lets go, so a restarted node still knows. A sip takes `drinking.sip_s` at the mouth; the wait for an open
mouth is bounded (`node.open_mouth_timeout_s`). Error codes: `cup_not_found`, `pickup_failed`, `delivery_failed`,
`presentation_refused`, `putdown_failed`, `move_failed`, `robot_unavailable`, `not_set_up`, `already_holding`,
`not_holding`. A cancel stops the move in flight and waits for the arm to settle; the next action first backs away from
the user, or out of the handle.

## Parameters

| Name | Default | Meaning |
| --- | --- | --- |
| `use_orchestrator` | `true` | `false`: standalone (drive it with `ros2 action send_goal /drinking/execute_action ...`) |
| `mock` | `false` | No hardware: every action answers at once, nothing moves |
| `state_dir` | `/config` | The state folder shared with rammp-feeding (below) |
| `runs_dir` | `/runs` | The run record's folder |
| `record` | `false` | The run record: a debug setting |
| `camera_profile` | `oak_d_pro` in the fragment | rammp_feeding's `config/camera/<name>.yaml`: the wrist camera |
| `config` | `""` | Any other setting from `config/drinking.yaml` or rammp_feeding's, as `"a.b=1,c=[1, 2]"` |

## Arbitration

The orchestrator hands its execution token to `/drinking/acquire_control` before it starts an action and takes it back
through `/drinking/release_control` once the action ends. Every arm goal and gripper setpoint of the action carries that
token and the engine's id (`drinking`).

## Files (in the container, shared with rammp-feeding)

| Path | Contents |
| --- | --- |
| `/config/local.yaml` | The site file: the table (`workspace.table_extent_m`, `workspace.table_height_m`; the cup's own: `drinking.table`), the profiles, the cup (`drinking.mug`) |
| `/config/wrist_mount.json` | The wrist camera's mount (rammp-feeding's `rammp calibrate-wrist`) |
| `/config/{home,drink_gaze,drink_hold,drink_deliver}.json` | Captured poses (`home` is rammp-feeding's) |
| `/config/drink_state.json` | The cup this session: where it was picked up, while the gripper holds it |
| `/runs/` | The run record, with `record` on |

Host networking and IPC; run as the owner of `/config` and `/runs`; CPU only.
