"""Load the PACE-identified Revo3 hand dynamics from assets/training/revo3_pace_sysid.json.

The file holds one parameter set: the fit_03 baseline with the 7 fields of report section 17 replaced.

Usage:
    python3 scripts/revo3_pace_profile.py              # table of the 21 hand joints
    python3 scripts/revo3_pace_profile.py --isaaclab   # per-field dicts for DelayedPDActuatorCfg

Library use:
    from revo3_pace_profile import load_profile
    joints = load_profile()   # {joint_name: {kp, kd, armature, friction, viscous_damping, effort_limit}}
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

SYSID_PATH = Path(__file__).resolve().parents[1] / "assets/training/revo3_pace_sysid.json"
FIELDS = ("kp", "kd", "armature", "friction", "viscous_damping", "effort_limit")
ISAACLAB_FIELDS = {"kp": "stiffness", "kd": "damping", "armature": "armature", "friction": "friction",
                   "viscous_damping": "viscous_friction", "effort_limit": "effort_limit"}


def load_sysid(path: Path = SYSID_PATH) -> dict:
    return json.loads(path.read_text())


def load_profile(path: Path = SYSID_PATH) -> dict[str, dict[str, float]]:
    """Return {joint_name: {field: value}} for the 21 Revo3 hand joints."""
    return {joint: dict(values) for joint, values in load_sysid(path)["parameters"]["joints"].items()}


def isaaclab_actuator_kwargs(path: Path = SYSID_PATH) -> dict:
    """Per-joint dicts keyed by DelayedPDActuatorCfg field name, plus the delay in seconds.

    The caller converts command_delay_s into physics steps (min_delay = max_delay = round(delay / sim.dt)).
    """
    joints = load_profile(path)
    kwargs = {cfg_field: {joint: values[field] for joint, values in joints.items()}
              for field, cfg_field in ISAACLAB_FIELDS.items()}
    kwargs["command_delay_s"] = load_sysid(path)["model_assumptions"]["command_delay_s"]
    return kwargs


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--isaaclab", action="store_true", help="print DelayedPDActuatorCfg field dicts as JSON")
    args = parser.parse_args(argv)
    if args.isaaclab:
        print(json.dumps(isaaclab_actuator_kwargs(), indent=2))
        return 0
    joints = load_profile()
    print(f"{'joint':<24}" + "".join(f"{field:>17}" for field in FIELDS))
    for joint, values in joints.items():
        print(f"{joint:<24}" + "".join(f"{values[field]:>17.6g}" for field in FIELDS))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
