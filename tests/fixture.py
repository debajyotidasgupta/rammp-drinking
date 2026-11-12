"""The drinking tests' site, set up before rammp_drinking is imported: a temporary state_dir with the arm poses
(tests/poses, from tests/make_poses.py), the wrist camera's mount as rammp_feeding's calibrate-wrist leaves it (rammp_feeding's
test D405), the site's values (the table, the cup) and the run record beside it. rammp_feeding's own test pieces (its
fake robot and face, the arm's model) come from its source, in the image at /module_ws/src/rammp_feeding (FEEDING_SRC)."""

from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FEEDING = Path(os.environ.get("FEEDING_SRC", "/module_ws/src/rammp_feeding"))
sys.path[:0] = [str(ROOT), str(ROOT / "tests"), str(FEEDING / "tests")]
WORK = Path(tempfile.mkdtemp(prefix="rammp_drink_test_"))
STATE = WORK / "state"
STATE.mkdir(parents=True)
for p in (ROOT / "tests" / "poses").glob("*.json"):
    shutil.copy(p, STATE / p.name)
(STATE / "wrist_mount.json").write_text(json.dumps(dict(
    camera="realsense_d405", utensil="fork", camera_in_tool_m=[0.0, 0.075, -0.125], rotation_quat_xyzw=[0.0, 0.0, 1.0, 0.0],
    tip_px=[456, 374], image_region_px=[280, 365, 660, 480])))
MUG = dict(tip_in_tool_m=[0.07, 0.10, 0.18], radius_m=0.04, height_m=0.10, handle_centre_above_bottom_m=0.05,
           handle_to_axis_m=0.06)
os.environ.update(RAMMP_NO_LOCAL="1", RAMMP_STATE_DIR=str(STATE),
                  RAMMP_SET="workspace.table_extent_m.x=[0.30, 0.95],workspace.table_extent_m.y=[-0.30, 0.80],"
                            + ",".join("drinking.mug.%s=%s" % kv for kv in MUG.items()) + ","
                            "drinking.sip_s=0.5,deployment.runs_dir=%s,record.enabled=true,"
                            "planner.world_file=%s,planner.world_file_in_planner=" % (WORK / "runs", STATE / "drink_world.yaml"))
