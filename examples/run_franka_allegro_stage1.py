"""Run the model-only acceptance smoke for the Panda+Allegro backend."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time
from typing import Any, Callable, Iterable

import numpy as np

from allegro_probe.backends import FrankaAllegroMujocoBackend
from allegro_probe.franka_scene import (
    CANONICAL_ALLEGRO_OPEN,
    CANONICAL_PANDA_HOME,
    DEFAULT_MENAGERIE_ROOT,
    FrankaAllegroScene,
)


def _pair_payload(pair: Any) -> dict[str, Any]:
    return {
        "geom1": pair.geom1,
        "geom2": pair.geom2,
        "body1": pair.body1,
        "body2": pair.body2,
        "signed_distance_m": float(pair.signed_distance_m),
        "normal_force_N": float(pair.normal_force_N),
        "policy_filter_reason": pair.policy_filter_reason,
    }


def _advance(scene: FrankaAllegroScene, steps: int, viewer: Any = None) -> None:
    for _ in range(steps):
        scene.step()
        if viewer is not None:
            viewer.sync()
            time.sleep(scene.dt)


def _actuator_smoke(
    scene: FrankaAllegroScene,
    *,
    label: str,
    canonical: np.ndarray,
    qpos_indices: np.ndarray,
    actuator_ids: np.ndarray,
    other_actuator_ids: np.ndarray,
    command: Callable[[Iterable[float]], np.ndarray],
    delta_rad: float,
    steps: int,
    viewer: Any,
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for local_index in range(len(canonical)):
        for direction in (-1.0, 1.0):
            scene.reset()
            if viewer is not None:
                viewer.sync()
            q_before = scene.data.qpos.copy()
            ctrl_before = scene.data.ctrl.copy()
            target = canonical.copy()
            target[local_index] += direction * delta_rad
            command(target)
            control_isolated = bool(
                np.array_equal(
                    scene.data.ctrl[other_actuator_ids],
                    ctrl_before[other_actuator_ids],
                )
            )
            _advance(scene, steps, viewer)
            movement = float(
                scene.data.qpos[qpos_indices[local_index]]
                - q_before[qpos_indices[local_index]]
            )
            contact = scene.collision_snapshot()
            distance = scene.distance_audit(max_distance_m=0.030)
            finite = bool(
                np.all(np.isfinite(scene.data.qpos))
                and np.all(np.isfinite(scene.data.qvel))
                and np.all(np.isfinite(scene.data.ctrl))
            )
            passed = bool(
                direction * movement > 1e-3
                and control_isolated
                and finite
                and not contact.has_self_collision
                and not distance.has_forbidden_penetration
            )
            records.append(
                {
                    "group": label,
                    "actuator": scene.model.actuator(
                        int(actuator_ids[local_index])
                    ).name,
                    "direction": int(direction),
                    "command_delta_rad": direction * delta_rad,
                    "actual_delta_rad": movement,
                    "control_slice_isolated": control_isolated,
                    "finite_state": finite,
                    "forbidden_contact_count": len(contact.forbidden_contacts),
                    "forbidden_penetration_count": len(
                        distance.forbidden_penetrations
                    ),
                    "passed": passed,
                }
            )
    scene.reset()
    if viewer is not None:
        viewer.sync()
    return records


def run_stage1_smoke(
    scene: FrankaAllegroScene,
    *,
    arm_delta_rad: float = 0.010,
    hand_delta_rad: float = 0.010,
    steps: int = 40,
    viewer: Any = None,
) -> dict[str, Any]:
    """Return a JSON-safe report for all four stage-1 acceptance gates."""

    scene.reset()
    canonical_contact = scene.collision_snapshot()
    canonical_distance = scene.distance_audit(max_distance_m=0.100)
    frames = {
        name: {
            "position_m": scene.frame_pose(name).position_m.tolist(),
            "quaternion_wxyz": scene.frame_pose(name).quaternion_wxyz.tolist(),
        }
        for name in scene.frame_names
    }
    actuator_records = _actuator_smoke(
        scene,
        label="arm",
        canonical=CANONICAL_PANDA_HOME.copy(),
        qpos_indices=scene.arm_qpos_indices,
        actuator_ids=scene.arm_actuator_ids,
        other_actuator_ids=scene.hand_actuator_ids,
        command=scene.command_arm_joints,
        delta_rad=arm_delta_rad,
        steps=steps,
        viewer=viewer,
    )
    actuator_records.extend(
        _actuator_smoke(
            scene,
            label="hand",
            canonical=CANONICAL_ALLEGRO_OPEN.copy(),
            qpos_indices=scene.hand_qpos_indices,
            actuator_ids=scene.hand_actuator_ids,
            other_actuator_ids=scene.arm_actuator_ids,
            command=scene.command_hand_joints,
            delta_rad=hand_delta_rad,
            steps=steps,
            viewer=viewer,
        )
    )
    canonical_passed = bool(
        not canonical_contact.has_self_collision
        and not canonical_distance.has_forbidden_penetration
    )
    actuator_passed = all(record["passed"] for record in actuator_records)
    provenance = scene.model_provenance
    joint_margin = np.minimum(
        scene.data.qpos - scene.model.jnt_range[:, 0],
        scene.model.jnt_range[:, 1] - scene.data.qpos,
    )
    ctrl_margin = np.minimum(
        scene.data.ctrl - scene.model.actuator_ctrlrange[:, 0],
        scene.model.actuator_ctrlrange[:, 1] - scene.data.ctrl,
    )
    return {
        "passed": bool(
            (scene.model.nq, scene.model.nv, scene.model.nu) == (23, 23, 23)
            and canonical_passed
            and actuator_passed
        ),
        "backend": scene.config.backend,
        "mount_profile_id": scene.mount_profile.profile_id,
        "model_provenance": {
            "menagerie_root": str(provenance.menagerie_root),
            "panda_xml_path": str(provenance.panda_xml_path),
            "panda_xml_sha256": provenance.panda_xml_sha256,
            "allegro_xml_path": str(provenance.allegro_xml_path),
            "allegro_xml_sha256": provenance.allegro_xml_sha256,
            "mujoco_version": provenance.mujoco_version,
            "timestep_s": provenance.timestep_s,
            "cone": provenance.cone,
            "integrator": provenance.integrator,
            "solver": provenance.solver,
            "controller_profile_id": provenance.controller_profile_id,
            "hand_kp": provenance.hand_kp,
        },
        "dimensions": {
            "nq": int(scene.model.nq),
            "nv": int(scene.model.nv),
            "nu": int(scene.model.nu),
        },
        "canonical": {
            "passed": canonical_passed,
            "minimum_joint_limit_margin_rad": float(np.min(joint_margin)),
            "minimum_ctrl_limit_margin_rad": float(np.min(ctrl_margin)),
            "forbidden_contacts": [
                _pair_payload(pair) for pair in canonical_contact.forbidden_contacts
            ],
            "forbidden_penetrations": [
                _pair_payload(pair)
                for pair in canonical_distance.forbidden_penetrations
            ],
            "minimum_forbidden_distance_m": (
                canonical_distance.minimum_forbidden_distance_m
            ),
        },
        "frames": frames,
        "actuator_smoke": {
            "passed": actuator_passed,
            "checks": actuator_records,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Panda+Allegro stage-1 model/FK/collision/actuator smoke"
    )
    parser.add_argument(
        "--menagerie-root", type=Path, default=DEFAULT_MENAGERIE_ROOT
    )
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--arm-delta-rad", type=float, default=0.010)
    parser.add_argument("--hand-delta-rad", type=float, default=0.010)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--hold-open", action="store_true")
    args = parser.parse_args()

    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.arm_delta_rad <= 0.0 or args.hand_delta_rad <= 0.0:
        parser.error("actuator deltas must be positive")

    backend = FrankaAllegroMujocoBackend.create(
        menagerie_root=args.menagerie_root
    )
    scene = backend.scene
    viewer_context = nullcontext(None)
    if args.viewer:
        try:
            import mujoco.viewer  # type: ignore
        except Exception as exc:
            raise SystemExit(f"mujoco.viewer is unavailable: {exc}") from exc
        viewer_context = mujoco.viewer.launch_passive(scene.model, scene.data)

    with viewer_context as viewer:
        report = run_stage1_smoke(
            scene,
            arm_delta_rad=float(args.arm_delta_rad),
            hand_delta_rad=float(args.hand_delta_rad),
            steps=int(args.steps),
            viewer=viewer,
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        if viewer is not None and args.hold_open:
            while viewer.is_running():
                viewer.sync()
                time.sleep(scene.dt)
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
