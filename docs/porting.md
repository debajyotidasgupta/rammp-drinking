---
title: Deployment checklist
---

# Deployment checklist

On a robot already set up for rammp-feeding (its deployment checklist: the Jetson, the network, the arm stack, the
wrist camera's mount), in order. Drinking shares feeding's state folder, `/srv/rammp_feeding/config` (`/config` in the
container): the site file, the table, the wrist camera's mount and the home pose are set once for both.
- Positions are metres in `arm_mount`, the arm model's gravity-aligned root (x forward, y left, z up).
- Keep someone at the e-stop for every command that moves the arm.
- The gripper must be empty for drinking. Take the fork out by hand before, and put it back after (then check
  feeding's 10 mm hover: a re-seated fork moves its tip).

## 1. The image and the manifest
```bash
docker pull debajyotidasgupta/rammp-drinking:0.1.2
```
- Save this as `/usr/local/bin/rammp-drink`, make it executable, and never run it while sheppy's `drinking` node is up:
  ```bash
  #!/usr/bin/env bash
  tty=-i; [ -t 0 ] && tty=-it
  exec docker run --rm $tty --network host --ipc host --user 1000:1000 -e ROS_DOMAIN_ID=0 -e HOME=/tmp \
    -v /srv/rammp_feeding/config:/config -v /srv/rammp_feeding/runs:/runs \
    debajyotidasgupta/rammp-drinking:0.1.2 ros2 run rammp_drinking rammp-drink "$@"
  ```
- rammp-deployments' manifest: the `drinking` node, copied from this repo's `rammp-alternative.yaml`. The planner needs
  nothing new: our world goes into the folder both `curobo_planner` alternatives already mount at `/feeding`.

## 2. The site file
Add to `/srv/rammp_feeding/config/local.yaml` (from [`config/machines/template.yaml`](../config/machines/template.yaml)):

| Key | How |
| --- | --- |
| `workspace.table_height_m` | Tape from the arm's base to the table's top (z). Feeding uses it too. Unset, the cup's handle gives it. |
| `drinking.table.height_m`, `extent_m` | Only when the cup stands on a table of its own: its top and edges (its near edge clear of the arm's base) |
| `drinking.mug.tip_in_tool_m` | Ruler, the cup in the gripper as picked up: the tool frame (between the finger pads; y up, z toward the user) to what goes to the mouth (the straw's end, or the rim nearest the user) |
| `drinking.mug.radius_m`, `height_m` | Ruler |
| `drinking.mug.handle_centre_above_bottom_m` | Ruler: the handle's centre (where the fingers go) above the cup's bottom |
| `drinking.mug.handle_to_axis_m` | Ruler: the handle's centre to the cup's axis behind its face; for a handle beside the cup (its opening toward the camera), `[x, y, z]` in the handle's frame (x up, y right to left, z out of its face) |
| `drinking.grasp.inside_bottom_m`, `inside_top_m`, `lifted_m` | The fingers' depth in the handle (z: 0.008 puts their tips about 4 cm past its face) and how far up under its top bar they open (x) |

`drinking.handle.prompts` are SAM3's words for the handle (a coloured handle: name its colour first). The fingers aim at
the opening inside the handle's outline; `handle.inlier_m` must take in the whole handle (a C 17 cm tall: 0.12).

## 3. Capture
`sheppy up <robot>`, without `drinking`; the gripper empty.
1. `rammp-drink capture drink_gaze`: the wrist camera on the cup where the operator puts it, about 0.2 m in front of
   its handle and 0.25 m above it, the whole handle in view. `rammp-drink find-handle` from there shows what it finds
   and whether the grasp's poses are in reach. The gripper goes into the handle level, so where the handle faces
   decides the reach: on an arm leaning forward on its mount, a handle facing the arm's base is out of reach; turn it to
   face the arm's side (the side away from the user).
2. Put the cup in the gripper by hand: `rammp-drink gripper close`, the handle over the closed fingers,
   `rammp-drink gripper open`. Then `rammp-drink capture drink_hold` (the cup upright, clear of the table and of the
   user) and `rammp-drink capture drink_deliver` (the cup upright facing the seated user from in front of them,
   the wrist camera on their face: the planner has no path to a look that reaches over the robot's model of its
   occupant). Both are levelled exactly as they are captured (`drinking.level_tol_deg`). Then take the cup out:
   `rammp-drink gripper close`.
3. `rammp-drink check`: every row OK.

## 4. Verify, then drink
1. `rammp-drink pick`, with the cup at its place. Watch the fingers go into the handle and lift it.
2. `rammp-drink sip`, seated facing `drink_deliver`. The cup's tip stops `delivery.stop_m` (`drinking.delivery`)
   short of the mouth.
3. `rammp-drink put-down`. Then add `drinking` to the sheppy profile: the orchestrator starts its actions (standalone,
   `ros2 action send_goal /drinking/execute_action rammp_orchestrator_interfaces/action/ExecuteAction
   '{engine_id: drinking, action_id: pick_up_drink}'`, then `give_sip`, `put_down_drink`).

## When the setup changes

| Change | Redo |
| --- | --- |
| A different cup | `drinking.mug.*`, then `rammp-drink check` |
| Where the cup is put | `rammp-drink capture drink_gaze` |
| The user's seat | `rammp-drink capture drink_deliver` |
| The table | `workspace.*` (shared with feeding) |
| The wrist camera moved | rammp-feeding's `rammp calibrate-wrist` (with the fork) |
