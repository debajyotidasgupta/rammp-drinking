"""Drinking's settings: rammp_feeding's configuration (its layers and its loader), with this module's config/drinking.yaml
as RAMMP_DEFAULTS -- set before anything of rammp_feeding is imported, which reads its settings as it loads. Import this
module first.

    from rammp_drinking.config import cfg, D        # D(): cfg.drinking, read afresh (a reload replaces it)
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path


def _root() -> Path:
    """Where config/ is: the source tree, or the installed package's share."""
    src = Path(__file__).resolve().parent.parent
    if (src / "config" / "drinking.yaml").exists():
        return src
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("rammp_drinking"))


ROOT = _root()
DEFAULTS = ROOT / "config" / "drinking.yaml"
_given = [v for v in os.environ.get("RAMMP_DEFAULTS", "").split(",") if v and Path(v) != DEFAULTS]
os.environ["RAMMP_DEFAULTS"] = ",".join([str(DEFAULTS)] + _given)

from rammp_feeding import config as feeding_config  # noqa: E402  (loads with the defaults above)
from rammp_feeding.config import PARAMS, cfg, read_json, state_dir  # noqa: E402,F401


def _overlay() -> None:
    """drinking.delivery over rammp_feeding's delivery: the shared site file's delivery settings are the fork's."""
    feeding_config._merge(cfg["delivery"], dict(cfg["drinking"].get("delivery") or {}))


def apply_parameters(values: dict) -> None:
    """The node's parameters, as rammp_feeding's (config.apply_parameters), then drinking's delivery over its own."""
    feeding_config.apply_parameters(values)
    _overlay()


_overlay()

# The site's own values, null until they are measured or set (docs/porting.md): nothing that needs them moves the arm
# without them. The shared site file holds the table and the wrist camera's mount (rammp_feeding's calibrate-wrist
# measures it, with the fork); workspace.table_height_m, where set, is the table's top (else the cup's handle gives it).
MEASURED = ("camera.mount.camera_in_tool_m", "camera.mount.rotation_quat_xyzw", "workspace.table_extent_m",
            "drinking.mug.tip_in_tool_m", "drinking.mug.radius_m", "drinking.mug.height_m",
            "drinking.mug.handle_centre_above_bottom_m", "drinking.mug.handle_to_axis_m")
STATE = "drink_state.json"      # the cup this session: where it was picked up, whether the gripper holds it


def D():
    """cfg.drinking, as loaded now."""
    return cfg.drinking


def unmeasured() -> list[str]:
    return [k for k in MEASURED if cfg.get(k, None) is None]


def captured(name: str) -> list[float]:
    """A captured pose's joints (deg): state_dir/<name>.json, rammp-drink capture <name>."""
    p = state_dir() / (name + ".json")
    if not p.exists():
        raise FileNotFoundError("no %s -- pose the arm and rammp-drink capture %s (docs/porting.md)" % (p, name))
    return [float(v) for v in read_json(p)["joints_deg"]]


def capture(name: str, joints: list[float]) -> Path:
    p = state_dir() / (name + ".json")
    p.write_text(json.dumps(dict(label=name, captured=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                                 joints_deg=[round(float(v), 3) for v in joints]), indent=1))
    return p


def session() -> dict:
    """The cup this session (drink_state.json): {} when the gripper holds nothing of ours."""
    p = state_dir() / STATE
    return read_json(p) if p.exists() else {}


def save_session(s: dict) -> None:
    p = state_dir() / STATE
    if s:
        p.write_text(json.dumps(s, indent=1))
    elif p.exists():
        p.unlink()


def state_report() -> list[tuple[int, str]]:
    """The node's start in its log, as (level, line): the state folder and what it holds, then each file ignored."""
    import logging
    d = Path(cfg.deployment.state_dir)
    names = ["local.yaml", "wrist_mount.json", *(n + ".json" for n in cfg.drinking.poses), STATE]
    line = "state %s: wrist camera %s; files: %s" % (d, cfg.profiles.camera, ", ".join(n for n in names if (d / n).exists())
                                                    or "none")
    if unmeasured():
        line += "; not yet measured: " + ", ".join(unmeasured())
    return [(logging.INFO, line)] + [(logging.WARNING, "ignored, as if missing: %s; the actions answer not_set_up until "
                                      "it is fixed" % u) for u in feeding_config.UNREADABLE]
