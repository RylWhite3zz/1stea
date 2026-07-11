"""Probe-aware state machines and feature extraction."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Tuple

import numpy as np

from allegro_probe.backends import ProbeBackend, as_backend
from allegro_probe.models import ProbeResult, canonical_family
from allegro_probe.protocols import (
    FEATURE_SCHEMA_VERSION,
    PROBE_PROTOCOL_ID,
    V1_DEFAULTS,
    canonical_probe_mode,
    validate_protocol_id,
)
from allegro_probe.scene import (
    AllegroProbeScene,
    ContactSnapshot,
    ProbeContactSnapshot,
)


_TOP_PINCH_GROUPS = frozenset({"mf", "th"})
_TOP_PINCH_LINK_TOKENS = (
    "fingertip_collision",
    "thumbtip_collision",
    "distal_collision",
)
_GRASP_CONTACT_PHASES = frozenset(
    {
        "contact_establish",
        "contact_quality_gate",
        "wrist_ft_baseline",
        "height_stabilization",
        "dynamic_baseline",
        "lift",
        "primitive_execution",
        "measurement",
        "post_check",
        "return_to_zero",
        "post_zero_check",
        "reorient_for_place",
        "place_descent",
        "release",
    }
)
_PROBE_CONTACT_PHASES = frozenset(
    {
        "guarded_contact",
        "contact_quality_gate",
        "primitive_execution",
        "retreat",
    }
)
_ALLEGRO_SURFACE_FINGERTIP_GEOM = "ff_tip_fingertip_collision"
_REFERENCE_SLIDE_PAD_GEOM = "ref_left_slide_pad_geom"
_FINGERTIP_PROBE_PHASES = _PROBE_CONTACT_PHASES
_ALLEGRO_POKE_PRESHAPE = np.asarray(
    [
        0.00, 0.80, 0.80, 0.50,
        0.00, 0.10, 0.05, 0.05,
        0.00, 0.10, 0.05, 0.05,
        0.45, 0.10, 0.08, 0.08,
    ],
    dtype=float,
)


def primitive_for_family(family: str) -> str:
    fam = canonical_family(family)
    return {
        "stiffness": "poke",
        "mass": "heft",
        "fill": "shake",
        "material": "slide",
    }[fam]


@dataclass
class _Run:
    backend: ProbeBackend
    primitive: str
    target: int
    mode: str = ""
    protocol_id: str = PROBE_PROTOCOL_ID
    feature_schema: str = FEATURE_SCHEMA_VERSION
    sensor_profile_id: str = ""
    phase: str = "init"
    violations: List[str] = field(default_factory=list)
    quality: Dict[str, float] = field(default_factory=dict)
    trace: Dict[str, List[Any]] = field(default_factory=dict)
    target_penetration_limit_m: float = float("inf")
    forbidden_penetration_limit_m: float = 0.00025
    max_hand_target_penetration_m: float = 0.0
    max_probe_target_penetration_m: float = 0.0
    max_forbidden_penetration_m: float = 0.0
    peak_hand_target_force_N: float = 0.0
    peak_probe_target_force_N: float = 0.0
    peak_forbidden_force_N: float = 0.0
    deepest_forbidden_pair: Tuple[str, str] = ("", "")
    phase_collision_maxima: Dict[str, Dict[str, float]] = field(
        default_factory=dict
    )
    sample_index: int = 0

    @property
    def scene(self) -> AllegroProbeScene:
        return self.backend.scene

    def enter(self, phase: str) -> None:
        self.phase = phase
        self.trace.setdefault("phase", []).append(phase)

    def fail(self, violation: str) -> None:
        if violation not in self.violations:
            self.violations.append(violation)
            self.trace.setdefault("violation", []).append(violation)
            self.trace.setdefault("violation_phase", []).append(self.phase)

    def sample(self, **values: Any) -> None:
        for name, value in values.items():
            if isinstance(value, np.ndarray):
                value = value.copy()
            self.trace.setdefault(name, []).append(value)

    def observe(self) -> Tuple[ContactSnapshot, ProbeContactSnapshot]:
        """Record one collision-buffer sample and enforce phase contact policy."""

        hand = self.scene.contact_snapshot(self.target)
        probe = self.scene.probe_contact_snapshot(self.target)
        self.max_hand_target_penetration_m = max(
            self.max_hand_target_penetration_m,
            hand.hand_max_penetration_m,
        )
        self.max_probe_target_penetration_m = max(
            self.max_probe_target_penetration_m,
            probe.target_max_penetration_m,
        )
        forbidden_penetration = max(
            hand.forbidden_max_penetration_m,
            probe.forbidden_max_penetration_m,
        )
        if forbidden_penetration > self.max_forbidden_penetration_m:
            self.max_forbidden_penetration_m = forbidden_penetration
            self.deepest_forbidden_pair = (
                probe.deepest_pair
                if probe.forbidden_max_penetration_m
                >= hand.forbidden_max_penetration_m
                else hand.deepest_forbidden_pair
            )
        self.peak_hand_target_force_N = max(
            self.peak_hand_target_force_N,
            hand.hand_normal_force_N + hand.palm_object_normal_force_N,
        )
        self.peak_probe_target_force_N = max(
            self.peak_probe_target_force_N,
            probe.target_normal_force_N,
        )
        forbidden_force = max(
            hand.palm_object_normal_force_N,
            hand.hand_table_normal_force_N,
            hand.hand_support_normal_force_N,
            hand.hand_other_object_normal_force_N,
            hand.object_other_object_normal_force_N,
            probe.forbidden_normal_force_N,
        )
        self.peak_forbidden_force_N = max(
            self.peak_forbidden_force_N, forbidden_force
        )
        phase_max = self.phase_collision_maxima.setdefault(
            self.phase,
            {
                "hand_target_penetration_m": 0.0,
                "probe_target_penetration_m": 0.0,
                "forbidden_penetration_m": 0.0,
                "hand_target_force_N": 0.0,
                "probe_target_force_N": 0.0,
                "forbidden_force_N": 0.0,
            },
        )
        phase_max["hand_target_penetration_m"] = max(
            phase_max["hand_target_penetration_m"],
            hand.hand_max_penetration_m,
        )
        phase_max["probe_target_penetration_m"] = max(
            phase_max["probe_target_penetration_m"],
            probe.target_max_penetration_m,
        )
        phase_max["forbidden_penetration_m"] = max(
            phase_max["forbidden_penetration_m"], forbidden_penetration
        )
        phase_max["hand_target_force_N"] = max(
            phase_max["hand_target_force_N"],
            hand.hand_normal_force_N + hand.palm_object_normal_force_N,
        )
        phase_max["probe_target_force_N"] = max(
            phase_max["probe_target_force_N"], probe.target_normal_force_N
        )
        phase_max["forbidden_force_N"] = max(
            phase_max["forbidden_force_N"], forbidden_force
        )

        if self.sample_index % 10 == 0:
            self.sample(
                collision_phase=self.phase,
                hand_target_penetration_m=hand.hand_max_penetration_m,
                probe_target_penetration_m=probe.target_max_penetration_m,
                forbidden_penetration_m=forbidden_penetration,
                hand_target_force_N=(
                    hand.hand_normal_force_N + hand.palm_object_normal_force_N
                ),
                probe_target_force_N=probe.target_normal_force_N,
                forbidden_force_N=forbidden_force,
                hand_groups=list(hand.hand_groups),
            )
        self.sample_index += 1

        # Contacts with the environment or another candidate are never part of
        # any probe primitive.  Palm/object contact is also always forbidden.
        if hand.palm_object_contact:
            self.fail("palm_object_collision")
        if hand.hand_table_contact:
            self.fail("hand_table_collision")
        if hand.hand_support_contact:
            self.fail("hand_support_collision")
        if hand.hand_other_object_contact or hand.object_other_object_contact:
            self.fail("other_object_collision")
        if (
            probe.table_contact
            or probe.support_contact
            or probe.other_object_contact
            or probe.other_contact
        ):
            self.fail("probe_forbidden_collision")
        if forbidden_penetration > self.forbidden_penetration_limit_m:
            self.fail("forbidden_penetration_limit")

        hand_target_contact = bool(hand.hand_contact_geoms)
        fingertip_surface_probe = bool(
            (self.primitive == "poke" and self.backend.name == "allegro")
            or self.primitive == "slide"
        )
        if fingertip_surface_probe:
            if probe.target_contact:
                self.fail("probe_target_collision")
            if hand_target_contact:
                if self.phase not in _FINGERTIP_PROBE_PHASES:
                    self.fail("unexpected_hand_contact")
                # MuJoCo can retain a zero-force contact row for one step while
                # separating; in that case contact_geoms is populated but the
                # force-thresholded hand_groups tuple is empty.
                allowed_group = (
                    "ff" if self.backend.name == "allegro" else "left"
                )
                allowed_geom = (
                    _ALLEGRO_SURFACE_FINGERTIP_GEOM
                    if self.backend.name == "allegro"
                    else _REFERENCE_SLIDE_PAD_GEOM
                )
                if set(hand.hand_groups) - {allowed_group}:
                    self.fail("inactive_finger_contact")
                if set(hand.hand_contact_geoms) != {allowed_geom}:
                    self.fail("forbidden_hand_link_contact")
            if hand.hand_max_penetration_m > self.target_penetration_limit_m:
                self.fail("fingertip_penetration_limit")
        elif self.primitive == "poke":
            if hand_target_contact:
                self.fail("hand_target_collision")
            if probe.target_contact and self.phase not in _PROBE_CONTACT_PHASES:
                self.fail("unexpected_probe_contact")
            if (
                probe.target_max_penetration_m
                > self.target_penetration_limit_m
            ):
                self.fail("probe_penetration_limit")
        else:
            if probe.target_contact:
                self.fail("probe_target_collision")
            if hand_target_contact and self.phase not in _GRASP_CONTACT_PHASES:
                self.fail("unexpected_hand_contact")
            if hand.hand_max_penetration_m > self.target_penetration_limit_m:
                self.fail("penetration_limit")
            if (
                self.backend.name == "allegro"
                and hand_target_contact
                and self.phase != "release"
            ):
                if set(hand.hand_groups) - _TOP_PINCH_GROUPS:
                    self.fail("inactive_finger_contact")
                if any(
                    not any(token in geom for token in _TOP_PINCH_LINK_TOKENS)
                    for geom in hand.hand_contact_geoms
                ):
                    self.fail("forbidden_hand_link_contact")
        return hand, probe

    def step(self, steps: int = 1) -> bool:
        """Step while auditing every state; return false on a new violation."""

        before = len(self.violations)
        for _ in range(max(int(steps), 0)):
            self.scene.step(1)
            self.observe()
            if len(self.violations) > before:
                return False
        return True

    def result(
        self,
        *,
        features: Dict[str, float],
        contact_seconds: float,
        params: Dict[str, Any],
        raw_summary: Dict[str, Any],
        valid: bool,
        status: str | None = None,
    ) -> ProbeResult:
        self.quality.update(
            {
                "max_hand_target_penetration_m": (
                    self.max_hand_target_penetration_m
                ),
                "max_probe_target_penetration_m": (
                    self.max_probe_target_penetration_m
                ),
                "max_forbidden_penetration_m": self.max_forbidden_penetration_m,
                "peak_hand_target_force_N": self.peak_hand_target_force_N,
                "peak_probe_target_force_N": self.peak_probe_target_force_N,
                "peak_forbidden_force_N": self.peak_forbidden_force_N,
            }
        )
        raw_summary = dict(raw_summary)
        raw_summary["collision_audit"] = {
            "target_penetration_limit_m": self.target_penetration_limit_m,
            "forbidden_penetration_limit_m": (
                self.forbidden_penetration_limit_m
            ),
            "deepest_forbidden_pair": list(self.deepest_forbidden_pair),
            "phase_maxima": self.phase_collision_maxima,
        }
        controller_status = status or (
            "ok" if valid else (self.violations[0] if self.violations else "invalid")
        )
        return ProbeResult(
            object_id=f"obj{self.target}",
            target=self.target,
            primitive=self.primitive,
            scene_id=self.scene.task.scene_id,
            protocol_id=self.protocol_id,
            feature_schema=self.feature_schema,
            mode=self.mode,
            sensor_profile_id=self.sensor_profile_id,
            backend=self.backend.name,
            status=controller_status,
            valid=bool(valid),
            controller_status=controller_status,
            phase_reached=self.phase,
            violations=list(self.violations),
            quality=dict(self.quality),
            features=features,
            contact_seconds=float(contact_seconds),
            params={"mode": self.mode, **params},
            raw_summary=raw_summary,
            trace=self.trace,
        )


def _mean_vec(
    run: _Run,
    reader: Callable[[], np.ndarray],
    steps: int = 60,
) -> np.ndarray:
    values = []
    for _ in range(int(steps)):
        if not run.step(1):
            break
        values.append(np.asarray(reader(), dtype=float).copy())
    return np.mean(values, axis=0) if values else np.zeros(3, dtype=float)


def _ctrl(scene: AllegroProbeScene, actuator: str) -> float:
    return float(scene.data.ctrl[scene.act[actuator]])


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _validate_lift_admission(
    *,
    lift_height: float,
    target_object_lift: float,
    lift_tolerance: float,
    max_lift_speed: float,
    support_loss_dwell: float,
    lift_stable_dwell: float,
    max_object_lift: float,
    penetration_limit: float,
    min_grasp_force: float,
) -> None:
    values = {
        "lift_height": lift_height,
        "target_object_lift": target_object_lift,
        "lift_tolerance": lift_tolerance,
        "max_lift_speed": max_lift_speed,
        "support_loss_dwell": support_loss_dwell,
        "lift_stable_dwell": lift_stable_dwell,
        "max_object_lift": max_object_lift,
        "penetration_limit": penetration_limit,
        "min_grasp_force": min_grasp_force,
    }
    checked = {name: _finite(name, value) for name, value in values.items()}
    if checked["lift_height"] <= 0.0:
        raise ValueError("lift_height must be positive")
    if checked["lift_height"] > 0.050:
        raise ValueError("lift_height exceeds the 50 mm admission cap")
    if checked["target_object_lift"] <= 0.0:
        raise ValueError("target_object_lift must be positive")
    if checked["target_object_lift"] > checked["lift_height"]:
        raise ValueError("target_object_lift must not exceed lift_height")
    if not 0.0 < checked["lift_tolerance"] < checked["target_object_lift"]:
        raise ValueError("lift_tolerance must be in (0, target_object_lift)")
    if not 0.005 <= checked["max_lift_speed"] <= 0.10:
        raise ValueError("max_lift_speed must be in [0.005, 0.10] m/s")
    if checked["lift_height"] / checked["max_lift_speed"] > 10.0:
        raise ValueError("lift timeout exceeds the 10 s admission cap")
    if checked["support_loss_dwell"] <= 0.0:
        raise ValueError("support_loss_dwell must be positive")
    if checked["support_loss_dwell"] > 2.0:
        raise ValueError("support_loss_dwell exceeds 2 s")
    if checked["lift_stable_dwell"] <= 0.0:
        raise ValueError("lift_stable_dwell must be positive")
    if checked["lift_stable_dwell"] > 2.0:
        raise ValueError("lift_stable_dwell exceeds 2 s")
    if (
        checked["max_object_lift"]
        < checked["target_object_lift"] + checked["lift_tolerance"]
    ):
        raise ValueError(
            "max_object_lift must include the complete target tolerance band"
        )
    if checked["max_object_lift"] > 0.015:
        raise ValueError("max_object_lift exceeds the 15 mm protocol cap")
    if checked["penetration_limit"] <= 0.0:
        raise ValueError("penetration_limit must be positive")
    if checked["penetration_limit"] > 0.020:
        raise ValueError("penetration_limit exceeds 20 mm")
    if checked["min_grasp_force"] <= 0.0:
        raise ValueError("min_grasp_force must be positive")


def _validate_poke_admission(
    *,
    depth: float,
    target_force: float,
    force_limit: float,
    contact_threshold: float,
    lateral_ratio_limit: float,
    hold_time: float,
) -> None:
    checked = {
        name: _finite(name, value)
        for name, value in {
            "depth": depth,
            "target_force": target_force,
            "force_limit": force_limit,
            "contact_threshold": contact_threshold,
            "lateral_ratio_limit": lateral_ratio_limit,
            "hold_time": hold_time,
        }.items()
    }
    if checked["depth"] <= 0.0:
        raise ValueError("depth must be positive")
    if checked["depth"] > 0.020:
        raise ValueError("depth exceeds 20 mm")
    if checked["target_force"] <= 0.0:
        raise ValueError("target_force must be positive")
    if checked["force_limit"] < checked["target_force"]:
        raise ValueError("force_limit must be at least target_force")
    if checked["force_limit"] > 100.0:
        raise ValueError("force_limit exceeds 100 N")
    if checked["contact_threshold"] <= 0.0:
        raise ValueError("contact_threshold must be positive")
    if checked["lateral_ratio_limit"] <= 0.0:
        raise ValueError("lateral_ratio_limit must be positive")
    if checked["hold_time"] < 0.0:
        raise ValueError("hold_time must be non-negative")
    if checked["hold_time"] > 1.0:
        raise ValueError("hold_time exceeds 1 s")


def _rotation_distance_rad(first: np.ndarray, second: np.ndarray) -> float:
    return float(
        np.arccos(
            np.clip(
                (np.trace(np.asarray(first).T @ np.asarray(second)) - 1.0) / 2.0,
                -1.0,
                1.0,
            )
        )
    )


def _force_along_world_z(
    scene: AllegroProbeScene, force_in_wrist_frame: np.ndarray
) -> float:
    from allegro_probe.geometry import quaternion_wxyz_to_matrix

    world_force = quaternion_wxyz_to_matrix(scene.wrist_quat()) @ np.asarray(
        force_in_wrist_frame, dtype=float
    )
    return float(world_force[2])


def _object_axis_tilt_rad(scene: AllegroProbeScene, target: int) -> float:
    """Return an axis-symmetric object's actual tilt from world vertical."""

    from allegro_probe.geometry import quaternion_wxyz_to_matrix

    rotation = quaternion_wxyz_to_matrix(scene.object_quat(target))
    vertical_alignment = abs(float(rotation[2, 2]))
    return float(np.arccos(np.clip(vertical_alignment, -1.0, 1.0)))


def _lockin_coefficient(
    values: np.ndarray, times: np.ndarray, frequency_Hz: float
) -> complex:
    signal = np.asarray(values, dtype=float)
    time_values = np.asarray(times, dtype=float)
    if signal.ndim != 1 or signal.size != time_values.size or signal.size < 4:
        return 0.0j
    centred = signal - float(np.mean(signal))
    carrier = np.exp(-1j * 2.0 * np.pi * float(frequency_Hz) * time_values)
    return complex(2.0 / signal.size * np.sum(centred * carrier))


def _lockin_snr_db(
    values: np.ndarray,
    coefficient: complex,
    times: np.ndarray,
    frequency_Hz: float,
) -> float:
    signal = np.asarray(values, dtype=float)
    time_values = np.asarray(times, dtype=float)
    if signal.size < 4:
        return float("-inf")
    centred = signal - float(np.mean(signal))
    fitted = np.real(
        coefficient
        * np.exp(1j * 2.0 * np.pi * float(frequency_Hz) * time_values)
    )
    residual = centred - fitted
    ratio = float(
        np.sum(fitted * fitted)
        / max(float(np.sum(residual * residual)), 1e-12)
    )
    return float(10.0 * np.log10(max(ratio, 1e-12)))


def _move_wrist(
    run: _Run,
    *,
    steps: int,
    x: float | None = None,
    y: float | None = None,
    z: float | None = None,
    roll: float | None = None,
    tilt: float | None = None,
    yaw: float | None = None,
    probe: float | None = None,
) -> bool:
    """Smoothly interpolate carriage controls; this is not arm planning."""

    scene = run.scene
    targets = {
        "x": x,
        "y": y,
        "z": z,
        "roll": roll,
        "tilt": tilt,
        "yaw": yaw,
        "probe": probe,
    }
    actuator = {
        "x": "act_wx",
        "y": "act_wy",
        "z": "act_wz",
        "roll": "act_wr",
        "tilt": "act_wt",
        "yaw": "act_wyaw",
        "probe": "act_wp",
    }
    starts = {
        key: _ctrl(scene, actuator[key])
        for key, target in targets.items()
        if target is not None
    }
    n = max(int(steps), 1)
    for index in range(n):
        u = (index + 1) / n
        alpha = u * u * (3.0 - 2.0 * u)
        command = {
            key: starts[key] + float(alpha) * (float(target) - starts[key])
            for key, target in targets.items()
            if target is not None
        }
        scene.command(**command)
        if not run.step(1):
            return False
    return True


def _probe_above(
    run: _Run,
    *,
    x: float,
    clearance: float,
) -> bool:
    scene = run.scene
    top = scene.object_top_z(run.target)
    if not _move_wrist(
        run,
        steps=140,
        x=x,
        y=0.0,
        z=scene.wz_for_tip_z(top + clearance),
        roll=0.0,
        tilt=0.0,
        yaw=0.0,
        probe=0.0,
    ):
        return False
    # Position actuators need to settle laterally before guarded descent; otherwise
    # an edge candidate is contacted off-centre and produces a spurious side load.
    return run.step(120)


def _safe_probe_retreat(run: _Run) -> bool:
    run.enter("retreat")
    moved = _move_wrist(
        run,
        steps=100,
        z=0.10,
        roll=0.0,
        tilt=0.0,
        yaw=0.0,
        probe=0.0,
    )
    run.enter("post_retreat_check")
    settled = run.step(20)
    clear = not run.scene.probe_contact_snapshot(run.target).target_contact
    if not clear:
        run.fail("probe_not_clear_after_retreat")
    return moved and settled and clear


def _contact_stable(
    run: _Run,
    *,
    steps: int,
    require_support_free: bool,
    penetration_limit: float,
    quality_prefix: str,
    lift_start_center_z: float | None = None,
    target_object_lift: float | None = None,
    lift_tolerance: float = 0.0,
    max_wrist_z: float | None = None,
    lift_correction_speed: float = 0.005,
    grasp: _Grasp | None = None,
) -> Tuple[bool, ContactSnapshot, float]:
    scene = run.scene
    rel = []
    rel_rotation = []
    valid_steps = 0
    last = ContactSnapshot()
    max_penetration = 0.0
    for _ in range(int(steps)):
        if lift_start_center_z is not None and target_object_lift is not None:
            actual_lift = float(
                scene.object_center_pos(run.target)[2] - lift_start_center_z
            )
            if actual_lift < target_object_lift - lift_tolerance:
                corrected_z = (
                    _ctrl(scene, "act_wz") + lift_correction_speed * scene.dt
                )
                if max_wrist_z is not None:
                    corrected_z = min(corrected_z, max_wrist_z)
                scene.command(z=corrected_z)
        if not run.step(1):
            break
        last = scene.contact_snapshot(run.target)
        if grasp is not None and grasp.top_entry:
            grasp.close_alpha, safe = _regulate_top_pinch(
                run,
                grasp.close_alpha,
                last,
                target_force_N=grasp.target_force_N,
            )
            if not safe:
                break
        max_penetration = max(max_penetration, last.max_penetration_m)
        opposing = _legal_grasp(run, last)
        support_ok = not require_support_free or (
            not last.support_contact and not last.table_contact
        )
        penetration_ok = last.max_penetration_m <= penetration_limit
        if opposing and support_ok and penetration_ok:
            valid_steps += 1
        translation, rotation = scene.relative_object_pose_in_wrist(run.target)
        rel.append(translation.copy())
        rel_rotation.append(rotation.copy())
    rel_arr = np.asarray(rel, dtype=float)
    drift = (
        float(np.max(np.linalg.norm(rel_arr - rel_arr[0], axis=1)))
        if len(rel_arr)
        else float("inf")
    )
    run.quality[f"{quality_prefix}_relative_drift_m"] = drift
    if rel_rotation:
        initial_rotation = rel_rotation[0]
        rotation_drift = max(
            _rotation_distance_rad(initial_rotation, rotation)
            for rotation in rel_rotation
        )
    else:
        rotation_drift = float("inf")
    run.quality[f"{quality_prefix}_relative_rotation_drift_rad"] = rotation_drift
    run.quality[f"{quality_prefix}_max_penetration_m"] = max_penetration
    run.quality[f"{quality_prefix}_stable_fraction"] = valid_steps / max(
        int(steps), 1
    )
    return valid_steps >= int(0.85 * steps), last, drift


@dataclass
class _Grasp:
    established: bool
    baseline_force: np.ndarray
    baseline_torque: np.ndarray
    close_alpha: float
    snapshot: ContactSnapshot
    wrist_goal: Dict[str, float] = field(default_factory=dict)
    initial_object_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    target_force_N: float = 0.0
    top_entry: bool = False


def _legal_grasp(run: _Run, snapshot: ContactSnapshot) -> bool:
    if run.backend.name == "reference":
        return run.scene.has_opposing_grasp(snapshot)
    forces = dict(snapshot.hand_force_by_group_N)
    required = _TOP_PINCH_GROUPS.issubset(set(snapshot.hand_groups)) and all(
        float(forces.get(group, 0.0)) >= 0.20 for group in _TOP_PINCH_GROUPS
    )
    top_lip = f"obj{run.target}_top_lip" in set(snapshot.hand_object_geoms)
    links = all(
        any(token in geom for token in _TOP_PINCH_LINK_TOKENS)
        for geom in snapshot.hand_contact_geoms
    )
    return bool(required and top_lip and links)


def _top_pinch_pose(scene: AllegroProbeScene, progress: float) -> np.ndarray:
    progress = float(np.clip(progress, 0.0, 1.0))
    preshape = scene.allegro_grip_pose(0.10)
    contact = scene.allegro_grip_pose(0.80)
    squeeze = scene.allegro_grip_pose(0.98)
    if progress <= 0.80:
        alpha = progress / 0.80
        return (1.0 - alpha) * preshape + alpha * contact
    alpha = (progress - 0.80) / 0.20
    return (1.0 - alpha) * contact + alpha * squeeze


def _move_allegro_hand(run: _Run, goal: np.ndarray, steps: int) -> bool:
    start = run.scene.allegro_joint_targets()
    n = max(int(steps), 1)
    for index in range(n):
        u = (index + 1) / n
        alpha = u * u * (3.0 - 2.0 * u)
        run.scene.command_allegro_joints((1.0 - alpha) * start + alpha * goal)
        if not run.step(1):
            return False
    return True


def _regulate_top_pinch(
    run: _Run,
    progress: float,
    snapshot: ContactSnapshot,
    *,
    target_force_N: float,
    force_limit_N: float = 20.0,
) -> Tuple[float, bool]:
    violation_count = len(run.violations)
    if snapshot.hand_normal_force_N > force_limit_N:
        run.fail("hand_force_limit")
        return progress, False
    if snapshot.hand_normal_force_N < 0.70 * target_force_N:
        progress = min(progress + 0.0015, 1.0)
    elif snapshot.hand_normal_force_N > 1.35 * target_force_N:
        progress = max(progress - 0.0010, 0.0)
    run.scene.command_allegro_joints(_top_pinch_pose(run.scene, progress))
    return progress, len(run.violations) == violation_count


def _empty_grasp(run: _Run, *, top_entry: bool = False) -> _Grasp:
    return _Grasp(
        established=False,
        baseline_force=np.zeros(3),
        baseline_torque=np.zeros(3),
        close_alpha=0.0,
        snapshot=run.scene.contact_snapshot(run.target),
        initial_object_pos=run.scene.object_pos(run.target).copy(),
        top_entry=top_entry,
    )


def _prepare_reference_grasp(
    run: _Run,
    *,
    penetration_limit: float,
    min_grasp_force: float,
) -> _Grasp:
    scene = run.scene
    initial_object_pos = scene.object_pos(run.target).copy()
    x = scene.candidate_x(run.target)
    y = 0.0
    z_grasp = float(
        np.clip(
            scene.object_mid_z(run.target) - scene.config.palm_height,
            -0.55,
            0.14,
        )
    )

    run.enter("approach")
    if not _move_wrist(
        run,
        steps=180,
        x=x,
        y=y,
        z=min(z_grasp + 0.08, 0.02),
        roll=0.0,
        tilt=0.0,
        yaw=0.0,
        probe=0.0,
    ) or not run.step(120):
        return _empty_grasp(run)

    run.enter("guarded_descent")
    if not _move_wrist(run, steps=180, z=z_grasp) or not run.step(100):
        return _empty_grasp(run)

    run.enter("contact_establish")
    close_alpha = 0.0
    snapshot = scene.contact_snapshot(run.target)
    established = False
    for close_alpha in np.linspace(0.0, 1.0, 101):
        scene.command(grip=float(close_alpha))
        if not run.step(8):
            break
        snapshot = scene.contact_snapshot(run.target)
        run.sample(
            grasp_alpha=float(close_alpha),
            grasp_groups=list(snapshot.hand_groups),
            grasp_force_N=snapshot.hand_normal_force_N,
            grasp_penetration_m=snapshot.hand_max_penetration_m,
        )
        if (
            _legal_grasp(run, snapshot)
            and snapshot.hand_normal_force_N >= min_grasp_force
        ):
            established = True
            break

    run.enter("contact_quality_gate")
    if not established:
        if not _legal_grasp(run, snapshot):
            run.fail("no_opposing_contact")
        elif snapshot.hand_normal_force_N < min_grasp_force:
            run.fail("insufficient_grasp_force")
    else:
        safe_alpha = float(close_alpha)
        for squeeze_alpha in np.linspace(
            close_alpha, min(float(close_alpha) + 0.09, 1.0), 8
        ):
            scene.command(grip=float(squeeze_alpha))
            if not run.step(4):
                break
            safe_alpha = float(squeeze_alpha)
            snapshot = scene.contact_snapshot(run.target)
        close_alpha = safe_alpha
        if not run.step(40):
            established = False
        stable, snapshot, drift = _contact_stable(
            run,
            steps=60,
            require_support_free=False,
            penetration_limit=penetration_limit,
            quality_prefix="pregrasp",
        )
        if not stable:
            established = False
            run.fail("unstable_pregrasp")
        if drift > 0.006:
            established = False
            run.fail("pregrasp_slip")

    run.enter("wrist_ft_baseline")
    baseline_force = _mean_vec(run, scene.wrist_force_vec, 50)
    baseline_torque = _mean_vec(run, scene.wrist_torque_vec, 50)
    run.quality["grasp_group_count"] = float(len(snapshot.hand_groups))
    run.quality["grasp_normal_force_N"] = snapshot.hand_normal_force_N
    run.quality["prelift_support_contact"] = float(snapshot.support_contact)
    return _Grasp(
        established=established and not run.violations,
        baseline_force=baseline_force,
        baseline_torque=baseline_torque,
        close_alpha=float(close_alpha),
        snapshot=snapshot,
        wrist_goal={"x": x, "y": y, "z": z_grasp, "roll": 0.0},
        initial_object_pos=initial_object_pos,
        target_force_N=min_grasp_force,
        top_entry=False,
    )


def _prepare_allegro_top_grasp(
    run: _Run,
    *,
    min_grasp_force: float,
) -> _Grasp:
    scene = run.scene
    initial_object_pos = scene.object_pos(run.target).copy()
    if not scene.full_hand_collisions_compiled():
        run.fail("full_hand_collisions_required")
        return _empty_grasp(run, top_entry=True)
    if f"obj{run.target}_pedestal" in scene.geom:
        run.fail("support_free_table_scene_required")
        return _empty_grasp(run, top_entry=True)

    obj = scene.task.objects[run.target]
    center = scene.object_center_pos(run.target).copy()
    x_goal = float(center[0])
    y_goal = float(center[1] - 0.020)
    physical_grasp_z = float(center[2] + obj.size[2] + 0.094)
    z_grasp = physical_grasp_z - scene.config.palm_height
    z_pregrasp = z_grasp + 0.075

    run.enter("approach")
    if not _move_allegro_hand(run, scene.allegro_grip_pose(0.10), 120):
        return _empty_grasp(run, top_entry=True)
    if not _move_wrist(
        run, steps=200, x=x_goal, y=y_goal, z=0.12, probe=0.0
    ):
        return _empty_grasp(run, top_entry=True)
    if not _move_wrist(
        run, steps=300, roll=np.pi, tilt=0.0, yaw=0.0
    ):
        return _empty_grasp(run, top_entry=True)
    if not _move_wrist(run, steps=450, z=z_pregrasp):
        return _empty_grasp(run, top_entry=True)

    run.enter("guarded_descent")
    if not _move_wrist(run, steps=300, z=z_grasp):
        return _empty_grasp(run, top_entry=True)

    run.enter("contact_establish")
    snapshot = scene.contact_snapshot(run.target)
    progress = 0.0
    established = False
    first_legal_progress = -1.0
    for progress in np.linspace(0.0, 1.0, 181):
        scene.command_allegro_joints(_top_pinch_pose(scene, float(progress)))
        if not run.step(5):
            break
        snapshot = scene.contact_snapshot(run.target)
        legal = _legal_grasp(run, snapshot)
        if legal and first_legal_progress < 0.0:
            first_legal_progress = float(progress)
        run.sample(
            grasp_alpha=float(progress),
            grasp_groups=list(snapshot.hand_groups),
            grasp_force_N=snapshot.hand_normal_force_N,
            grasp_penetration_m=snapshot.hand_max_penetration_m,
        )
        if snapshot.hand_normal_force_N > 20.0:
            run.fail("hand_force_limit")
            break
        if legal and snapshot.hand_normal_force_N >= min_grasp_force:
            established = True
            break

    run.quality["first_legal_contact_progress"] = first_legal_progress
    run.enter("contact_quality_gate")
    stable_steps = 0
    if established:
        for _ in range(100):
            if not run.step(1):
                established = False
                break
            snapshot = scene.contact_snapshot(run.target)
            progress, safe = _regulate_top_pinch(
                run,
                float(progress),
                snapshot,
                target_force_N=min_grasp_force,
            )
            if not safe:
                established = False
                break
            if (
                _legal_grasp(run, snapshot)
                and snapshot.hand_normal_force_N >= 0.80 * min_grasp_force
            ):
                stable_steps += 1
            else:
                stable_steps = 0
        if stable_steps < 80:
            established = False
            run.fail("unstable_pregrasp")
    else:
        if not _legal_grasp(run, snapshot):
            run.fail("no_legal_top_pinch")
        elif snapshot.hand_normal_force_N < min_grasp_force:
            run.fail("insufficient_grasp_force")

    run.enter("wrist_ft_baseline")
    baseline_force = _mean_vec(run, scene.wrist_force_vec, 50)
    baseline_torque = _mean_vec(run, scene.wrist_torque_vec, 50)
    snapshot = scene.contact_snapshot(run.target)
    run.quality["grasp_group_count"] = float(len(snapshot.hand_groups))
    run.quality["grasp_normal_force_N"] = snapshot.hand_normal_force_N
    run.quality["prelift_support_contact"] = float(
        snapshot.support_contact or snapshot.table_contact
    )
    return _Grasp(
        established=established and not run.violations,
        baseline_force=baseline_force,
        baseline_torque=baseline_torque,
        close_alpha=float(progress),
        snapshot=snapshot,
        wrist_goal={
            "x": x_goal,
            "y": y_goal,
            "z": z_grasp,
            "roll": float(np.pi),
            "tilt": 0.0,
            "yaw": 0.0,
        },
        initial_object_pos=initial_object_pos,
        target_force_N=min_grasp_force,
        top_entry=True,
    )


def _prepare_grasp(
    run: _Run,
    *,
    penetration_limit: float,
    min_grasp_force: float,
) -> _Grasp:
    if run.backend.name == "allegro":
        return _prepare_allegro_top_grasp(
            run, min_grasp_force=min_grasp_force
        )
    return _prepare_reference_grasp(
        run,
        penetration_limit=penetration_limit,
        min_grasp_force=min_grasp_force,
    )


def _lift_and_gate(
    run: _Run,
    grasp: _Grasp,
    *,
    lift_height: float,
    target_object_lift: float,
    lift_tolerance: float,
    max_lift_speed: float,
    support_loss_dwell: float,
    lift_stable_dwell: float,
    max_object_lift: float,
    penetration_limit: float,
) -> Tuple[bool, ContactSnapshot]:
    scene = run.scene
    if not grasp.established:
        run.enter("post_check")
        return False, scene.contact_snapshot(run.target)
    if lift_height <= 0.0:
        raise ValueError("lift_height must be positive")
    if target_object_lift <= 0.0 or target_object_lift > lift_height:
        raise ValueError("target_object_lift must be in (0, lift_height]")
    if lift_tolerance <= 0.0 or lift_tolerance >= target_object_lift:
        raise ValueError("lift_tolerance must be in (0, target_object_lift)")
    if max_lift_speed <= 0.0:
        raise ValueError("max_lift_speed must be positive")
    if support_loss_dwell <= 0.0:
        raise ValueError("support_loss_dwell must be positive")
    if lift_stable_dwell <= 0.0:
        raise ValueError("lift_stable_dwell must be positive")
    if max_object_lift < target_object_lift + lift_tolerance:
        raise ValueError(
            "max_object_lift must include the complete target tolerance band"
        )

    run.enter("lift")
    z0 = _ctrl(scene, "act_wz")
    z_command = z0
    lift_start_center_z = float(scene.object_center_pos(run.target)[2])
    lift_start_lowest_exterior_z = scene.object_lowest_exterior_z(run.target)
    lift_source_support_z = scene.object_source_support_z(run.target)
    max_steps = max(
        1,
        int(np.ceil(lift_height / (max_lift_speed * scene.dt))) + 120,
    )
    support_loss_steps_required = max(
        1, int(np.ceil(support_loss_dwell / scene.dt))
    )
    target_stable_steps_required = max(
        1, int(np.ceil(lift_stable_dwell / scene.dt))
    )
    lost_steps = 0
    support_free_steps = 0
    target_stable_steps = 0
    ever_support_free = False
    reached_target_band = False
    wrist_limit_steps = 0
    progress = grasp.close_alpha
    snapshot = scene.contact_snapshot(run.target)
    for _ in range(max_steps):
        object_lift = float(
            scene.object_center_pos(run.target)[2] - lift_start_center_z
        )
        if object_lift > max_object_lift:
            run.fail("object_lift_limit")
            break
        if object_lift < target_object_lift - lift_tolerance:
            z_command = min(
                z_command + max_lift_speed * scene.dt,
                z0 + lift_height,
            )
        scene.command(z=z_command)
        if not run.step(1):
            break
        snapshot = scene.contact_snapshot(run.target)
        if grasp.top_entry:
            progress, safe = _regulate_top_pinch(
                run,
                progress,
                snapshot,
                target_force_N=grasp.target_force_N,
            )
            if not safe:
                break
        if _legal_grasp(run, snapshot):
            lost_steps = 0
        else:
            lost_steps += 1
        if lost_steps > 80:
            run.fail("lost_grasp_during_lift")
            break
        support_free = not snapshot.support_contact and not snapshot.table_contact
        if support_free:
            support_free_steps += 1
            ever_support_free = True
        else:
            support_free_steps = 0
        object_lift = float(
            scene.object_center_pos(run.target)[2] - lift_start_center_z
        )
        in_target_band = bool(
            target_object_lift - lift_tolerance
            <= object_lift
            <= target_object_lift + lift_tolerance
        )
        if in_target_band and support_free_steps >= support_loss_steps_required:
            target_stable_steps += 1
            reached_target_band = True
        else:
            target_stable_steps = 0
        run.sample(
            object_lift_m=object_lift,
            wrist_lift_command_m=z_command - z0,
            support_free=float(support_free),
        )
        if target_stable_steps >= target_stable_steps_required:
            break
        if (
            z_command >= z0 + lift_height - 1e-9
            and object_lift < target_object_lift - lift_tolerance
        ):
            wrist_limit_steps += 1
        else:
            wrist_limit_steps = 0
        if wrist_limit_steps > 100:
            run.fail("object_lift_target_not_reached")
            break
    grasp.close_alpha = progress

    run.enter("post_check")
    stable, snapshot, drift = _contact_stable(
        run,
        steps=80,
        require_support_free=True,
        penetration_limit=penetration_limit,
        quality_prefix="postlift",
        lift_start_center_z=lift_start_center_z,
        target_object_lift=target_object_lift,
        lift_tolerance=lift_tolerance,
        max_wrist_z=z0 + lift_height,
        lift_correction_speed=max_lift_speed,
        grasp=grasp,
    )
    lifted_distance = float(
        scene.object_center_pos(run.target)[2] - lift_start_center_z
    )
    run.quality["lift_distance_m"] = lifted_distance
    run.quality["lift_start_center_z_m"] = lift_start_center_z
    run.quality["lift_start_lowest_exterior_z_m"] = (
        lift_start_lowest_exterior_z
    )
    run.quality["lift_source_support_z_m"] = lift_source_support_z
    run.quality["target_object_lift_m"] = float(target_object_lift)
    run.quality["lift_target_error_m"] = float(
        lifted_distance - target_object_lift
    )
    run.quality["wrist_lift_command_m"] = float(
        _ctrl(scene, "act_wz") - z0
    )
    run.quality["support_free_dwell_s"] = float(
        support_free_steps * scene.dt
    )
    run.quality["lift_target_stable_dwell_s"] = float(
        target_stable_steps * scene.dt
    )
    run.quality["ever_support_free"] = float(ever_support_free)
    run.quality["lift_started"] = float(z_command > z0 + 1e-6)
    run.quality["support_contact_after_lift"] = float(snapshot.support_contact)
    run.quality["table_contact_after_lift"] = float(snapshot.table_contact)
    run.quality["postlift_group_count"] = float(len(snapshot.hand_groups))

    valid = grasp.established and stable and not run.violations
    if not reached_target_band or not (
        target_object_lift - lift_tolerance
        <= lifted_distance
        <= target_object_lift + lift_tolerance
    ):
        valid = False
        run.fail("object_lift_target_not_stable")
    if support_free_steps < support_loss_steps_required:
        valid = False
        run.fail("support_loss_dwell_not_met")
    if snapshot.support_contact or snapshot.table_contact:
        valid = False
        run.fail("support_contact_after_lift")
    if not _legal_grasp(run, snapshot):
        valid = False
        run.fail("lost_grasp")
    if drift > 0.0035:
        valid = False
        run.fail("postlift_slip")
    if run.quality.get("postlift_relative_rotation_drift_rad", 0.0) > np.deg2rad(5.0):
        valid = False
        run.fail("postlift_rotation_slip")
    return valid, snapshot


def _move_while_holding(
    run: _Run,
    grasp: _Grasp,
    *,
    steps: int,
    **targets: float,
) -> bool:
    scene = run.scene
    actuators = {
        "x": "act_wx",
        "y": "act_wy",
        "z": "act_wz",
        "roll": "act_wr",
        "tilt": "act_wt",
        "yaw": "act_wyaw",
    }
    starts = {key: _ctrl(scene, actuators[key]) for key in targets}
    n = max(int(steps), 1)
    lost_steps = 0
    for index in range(n):
        u = (index + 1) / n
        alpha = u * u * (3.0 - 2.0 * u)
        scene.command(
            **{
                key: starts[key] + alpha * (float(value) - starts[key])
                for key, value in targets.items()
            }
        )
        if not run.step(1):
            return False
        snapshot = scene.contact_snapshot(run.target)
        if grasp.top_entry:
            grasp.close_alpha, safe = _regulate_top_pinch(
                run,
                grasp.close_alpha,
                snapshot,
                target_force_N=grasp.target_force_N,
            )
            if not safe:
                return False
        if _legal_grasp(run, snapshot):
            lost_steps = 0
        else:
            lost_steps += 1
        if lost_steps > 80:
            run.fail("lost_grasp_during_place")
            return False
    return True


def _release_grasp(run: _Run, grasp: _Grasp) -> bool:
    scene = run.scene
    run.enter("release")
    if grasp.top_entry:
        start = scene.allegro_joint_targets()
        goal = scene.allegro_grip_pose(0.0)
        clear_steps = 0
        scene.set_allegro_position_kp(0.1)
        try:
            for index in range(600):
                u = (index + 1) / 600
                alpha = u * u * (3.0 - 2.0 * u)
                scene.command_allegro_joints((1.0 - alpha) * start + alpha * goal)
                if not run.step(1):
                    return False
                snapshot = scene.contact_snapshot(run.target)
                if snapshot.hand_normal_force_N > 30.0:
                    run.fail("release_force_limit")
                    return False
                if not snapshot.hand_groups and not snapshot.palm_object_contact:
                    clear_steps += 1
                else:
                    clear_steps = 0
            grasp.close_alpha = 0.0
            return clear_steps >= 12
        finally:
            scene.set_allegro_position_kp(8.0)
    else:
        for alpha in np.linspace(scene._grip_alpha, 0.0, 80):
            scene.command(grip=float(alpha))
            if not run.step(2):
                return False
        if not scene.contact_snapshot(run.target).hand_groups:
            grasp.close_alpha = 0.0
            return True
    run.fail("release_incomplete")
    return False


def _place_and_retreat(run: _Run, grasp: _Grasp, *, object_lifted: bool) -> bool:
    """Return the object to its source support before opening and retreating."""

    scene = run.scene
    cleanup_violation_count = len(run.violations)
    placed = not object_lifted
    if object_lifted:
        run.enter("reorient_for_place")
        if not _move_while_holding(
            run,
            grasp,
            steps=180,
            tilt=0.0,
            yaw=0.0,
            roll=(np.pi if grasp.top_entry else 0.0),
        ):
            placed = False
        else:
            run.enter("place_descent")
            target_z = float(grasp.wrist_goal.get("z", _ctrl(scene, "act_wz")))
            if _move_while_holding(
                run, grasp, steps=(700 if grasp.top_entry else 260), z=target_z
            ):
                stable_support_steps = 0
                for _ in range(180):
                    snapshot = scene.contact_snapshot(run.target)
                    supported = snapshot.table_contact or snapshot.support_contact
                    if supported:
                        stable_support_steps += 1
                    else:
                        stable_support_steps = 0
                        scene.command(z=_ctrl(scene, "act_wz") - 0.00015)
                    if not run.step(1):
                        break
                    if grasp.top_entry:
                        grasp.close_alpha, safe = _regulate_top_pinch(
                            run,
                            grasp.close_alpha,
                            scene.contact_snapshot(run.target),
                            target_force_N=grasp.target_force_N,
                        )
                        if not safe:
                            break
                    if stable_support_steps >= 20:
                        placed = True
                        break
            if not placed:
                run.fail("source_support_not_reestablished")

    # Even after an earlier measurement failure, attempt a guarded release so
    # cleanup does not turn into the old high-air drop.
    released = _release_grasp(run, grasp)
    run.enter("retreat")
    retreat_z = 0.12 if grasp.top_entry else 0.10
    retreated = _move_wrist(
        run,
        steps=(300 if grasp.top_entry else 140),
        z=retreat_z,
        probe=0.0,
    )
    if grasp.top_entry and retreated:
        retreated = _move_wrist(
            run,
            steps=300,
            roll=0.0,
            tilt=0.0,
            yaw=0.0,
        )
    run.enter("post_release_check")
    supported_steps = 0
    for _ in range(80):
        if not run.step(1):
            break
        snapshot = scene.contact_snapshot(run.target)
        if (
            (snapshot.table_contact or snapshot.support_contact)
            and not snapshot.hand_groups
            and not snapshot.palm_object_contact
        ):
            supported_steps += 1
        else:
            supported_steps = 0
    if supported_steps < 20:
        run.fail("object_not_supported_after_release")
    run.quality["cleanup_new_violation_count"] = float(
        max(len(run.violations) - cleanup_violation_count, 0)
    )
    return bool(
        placed
        and released
        and supported_steps >= 20
        and retreated
        and len(run.violations) == cleanup_violation_count
    )


def _legal_fingertip_poke_contact(snapshot: ContactSnapshot) -> bool:
    return bool(
        set(snapshot.hand_groups) == {"ff"}
        and set(snapshot.hand_contact_geoms)
        == {_ALLEGRO_SURFACE_FINGERTIP_GEOM}
        and not snapshot.palm_object_contact
    )


def _legal_fingertip_slide_contact(
    run: _Run, snapshot: ContactSnapshot
) -> bool:
    if run.backend.name == "allegro":
        return bool(
            set(snapshot.hand_groups) == {"ff"}
            and set(snapshot.hand_contact_geoms)
            == {_ALLEGRO_SURFACE_FINGERTIP_GEOM}
            and not snapshot.palm_object_contact
        )
    return bool(
        set(snapshot.hand_groups) == {"left"}
        and set(snapshot.hand_contact_geoms) == {_REFERENCE_SLIDE_PAD_GEOM}
        and not snapshot.palm_object_contact
    )


def _safe_fingertip_retreat(run: _Run) -> bool:
    """Retract vertically before rotating or opening the downward-facing hand."""

    scene = run.scene
    run.enter("retreat")
    z_clear = min(_ctrl(scene, "act_wz") + 0.060, 0.12)
    moved = _move_wrist(
        run,
        steps=300,
        z=z_clear,
        roll=float(np.pi),
        tilt=0.0,
        yaw=0.0,
        probe=0.0,
    )
    if moved and z_clear < 0.119:
        moved = _move_wrist(
            run,
            steps=220,
            z=0.12,
            roll=float(np.pi),
            tilt=0.0,
            yaw=0.0,
            probe=0.0,
        )
    clear = bool(
        scene.fingertip_touch("ff") <= 1e-4
        and not scene.contact_snapshot(run.target).hand_groups
    )
    if not clear:
        run.fail("fingertip_not_clear_after_retreat")
    if moved and clear:
        moved = _move_wrist(
            run,
            steps=300,
            roll=0.0,
            tilt=0.0,
            yaw=0.0,
        )
    if moved:
        moved = _move_allegro_hand(run, scene.allegro_grip_pose(0.0), 120)
    run.enter("post_retreat_check")
    settled = run.step(20)
    return bool(moved and clear and settled)


def _poke_with_allegro_fingertip(
    run: _Run,
    *,
    depth: float,
    target_force: float,
    force_limit: float,
    contact_threshold: float,
    lateral_ratio_limit: float,
    hold_time: float,
) -> ProbeResult:
    """Execute a guarded index-fingertip indentation using real touch feedback."""

    scene = run.scene
    run.sensor_profile_id = "sim.allegro_ff_tip_touch+wrist_ft.v1"
    run.target_penetration_limit_m = 0.0005
    target_top = scene.object_pos(run.target).copy()

    run.enter("approach")
    if not scene.full_hand_collisions_compiled():
        run.fail("full_hand_collisions_required")
    if not run.violations:
        _move_allegro_hand(run, _ALLEGRO_POKE_PRESHAPE, 160)
    if not run.violations:
        _move_wrist(
            run,
            steps=220,
            x=float(target_top[0]),
            y=float(target_top[1]),
            z=0.12,
            probe=0.0,
        )
    if not run.violations:
        _move_wrist(
            run,
            steps=300,
            roll=float(np.pi),
            tilt=0.0,
            yaw=0.0,
        )

    # Calibrate the wrist target from the live site pose.  This remains valid if
    # the Menagerie kinematics or preshape changes; no fingertip offset is baked in.
    desired_tip = target_top.copy()
    desired_tip[2] += 0.040
    for _ in range(2):
        if run.violations:
            break
        tip = scene.fingertip_positions()["ff"]
        _move_wrist(
            run,
            steps=260,
            x=_ctrl(scene, "act_wx") + float(desired_tip[0] - tip[0]),
            y=_ctrl(scene, "act_wy") + float(desired_tip[1] - tip[1]),
            z=_ctrl(scene, "act_wz") + float(desired_tip[2] - tip[2]),
            roll=float(np.pi),
            tilt=0.0,
            yaw=0.0,
            probe=0.0,
        )
    force_baseline = (
        _mean_vec(run, scene.wrist_force_vec, 40)
        if not run.violations
        else np.zeros(3, dtype=float)
    )

    run.enter("guarded_contact")
    contact_z = None
    object_z = None
    z_command = _ctrl(scene, "act_wz")
    max_guard_steps = max(1, int(np.ceil((0.050 + depth) / 0.00015)))
    for _ in range(max_guard_steps if not run.violations else 0):
        z_command -= 0.00015
        scene.command(z=z_command)
        if not run.step(3):
            break
        touch = scene.fingertip_touch("ff")
        snapshot = scene.contact_snapshot(run.target)
        if touch > force_limit:
            run.fail("force_limit")
            break
        if touch >= contact_threshold and _legal_fingertip_poke_contact(snapshot):
            contact_z = float(scene.fingertip_positions()["ff"][2])
            object_z = float(scene.object_pos(run.target)[2])
            break
    if contact_z is None and not run.violations:
        run.fail("no_contact")

    run.enter("contact_quality_gate")
    stable_identity_steps = 0
    if contact_z is not None:
        for _ in range(20):
            if not run.step(1):
                break
            snapshot = scene.contact_snapshot(run.target)
            if _legal_fingertip_poke_contact(snapshot):
                stable_identity_steps += 1
            else:
                stable_identity_steps = 0
    run.quality["contact_identity_stable_fraction"] = stable_identity_steps / 20.0
    if contact_z is not None and stable_identity_steps < 16:
        run.fail("unstable_target_contact")

    loading_trace: List[Tuple[float, float, float, float]] = []
    hold_force: List[float] = []
    contact_steps = 0
    if contact_z is not None and not run.violations:
        run.enter("primitive_execution")
        integral = 0.0
        force_stable_steps = 0
        touch_lost_steps = 0
        for _ in range(max(80, int(0.8 / scene.dt))):
            touch = scene.fingertip_touch("ff")
            error = target_force - touch
            integral = float(
                np.clip(integral + error * scene.dt, -0.2, 0.2)
            )
            dz = float(
                np.clip(2.5e-5 * error + 4.0e-6 * integral, -1.5e-5, 2.5e-5)
            )
            z_command -= dz
            scene.command(z=z_command)
            if not run.step(1):
                break
            tip_z = float(scene.fingertip_positions()["ff"][2])
            indentation = max(contact_z - tip_z, 0.0)
            compression = max(
                float(object_z) - float(scene.object_pos(run.target)[2]), 0.0
            )
            touch = scene.fingertip_touch("ff")
            fvec = scene.wrist_force_vec() - force_baseline
            ft = float(np.linalg.norm(fvec[:2]))
            loading_trace.append((indentation, compression, touch, ft))
            run.sample(
                effector="ff_tip",
                force_N=touch,
                touch_force_N=touch,
                wrist_force_delta_N=fvec,
                indentation_m=indentation,
                compression_m=compression,
                compression_joint_q=float(
                    scene.data.qpos[
                        scene.joint_qadr[f"obj{run.target}_compress"]
                    ]
                ),
                lateral_force_N=ft,
            )
            if touch > force_limit:
                run.fail("force_limit")
                break
            snapshot = scene.contact_snapshot(run.target)
            if not _legal_fingertip_poke_contact(snapshot):
                touch_lost_steps += 1
            else:
                touch_lost_steps = 0
            if touch_lost_steps > 20:
                run.fail("lost_contact")
                break
            if (
                touch > 0.5 * target_force
                and ft / max(touch, 1e-6) > lateral_ratio_limit
            ):
                run.fail("excess_lateral_force")
                break
            contact_steps += 1
            if abs(touch - target_force) <= 0.08 * target_force:
                force_stable_steps += 1
            else:
                force_stable_steps = 0
            if compression >= depth or force_stable_steps >= 30:
                break

        # This spring-damper object has no independent viscoelastic ground truth.
        # The hold ratio is retained as controller diagnostics, not labelled as
        # material relaxation.
        n_hold = max(0, int(hold_time / scene.dt))
        for _ in range(n_hold):
            touch = scene.fingertip_touch("ff")
            error = target_force - touch
            integral = float(
                np.clip(integral + error * scene.dt, -0.2, 0.2)
            )
            dz = float(
                np.clip(2.5e-5 * error + 4.0e-6 * integral, -1.5e-5, 2.5e-5)
            )
            z_command -= dz
            scene.command(z=z_command)
            if not run.step(1):
                break
            touch = scene.fingertip_touch("ff")
            hold_force.append(touch)
            contact_steps += 1
            run.sample(hold_force_N=touch)
            if touch > force_limit:
                run.fail("force_limit")
                break
            if not _legal_fingertip_poke_contact(
                scene.contact_snapshot(run.target)
            ):
                run.fail("lost_contact_during_hold")
                break

    run.enter("post_check")
    if len(loading_trace) >= 3:
        probe_x = np.asarray([p[0] for p in loading_trace])
        comp_x = np.asarray([p[1] for p in loading_trace])
        force = np.asarray([p[2] for p in loading_trace])
        x_fit = comp_x if np.ptp(comp_x) > 1e-5 else probe_x
        mask = (x_fit > 5e-5) & (force > contact_threshold)
        if mask.sum() >= 2 and np.ptp(x_fit[mask]) > 1e-5:
            k_est = float(np.polyfit(x_fit[mask], force[mask], 1)[0])
        else:
            k_est = 0.0
        peak = float(force.max())
        indentation_max = float(probe_x.max())
        compression_max = float(comp_x.max())
    else:
        k_est = peak = indentation_max = compression_max = 0.0
    effective = compression_max if compression_max > 1e-5 else indentation_max
    if k_est <= 0.0 and effective > 1e-6 and peak > 1e-6:
        k_est = peak / effective
    compliance = effective / peak * 1000.0 if peak > 1e-6 else 0.0
    path_fraction = min(effective / max(depth, 1e-9), 1.0)
    hold_ratio = 0.0
    if hold_force:
        window = max(1, min(len(hold_force) // 4, 25))
        hold_ratio = float(
            np.mean(hold_force[-window:])
            / max(float(np.mean(hold_force[:window])), 1e-9)
        )
    run.quality["path_completion_ratio"] = path_fraction
    run.quality["peak_force_N"] = peak
    run.quality["target_force_ratio"] = peak / max(target_force, 1e-9)
    run.quality["hold_force_ratio"] = hold_ratio
    if peak < 0.80 * target_force and path_fraction < 0.80:
        run.fail("target_force_not_reached")
    if len(loading_trace) < 3:
        run.fail("insufficient_samples")
    valid = not run.violations

    retreat_ok = _safe_fingertip_retreat(run)
    valid = valid and retreat_ok and not run.violations
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "k_est_N_per_m": max(k_est, 0.0) if valid else 0.0,
            "k_est_N_per_mm": max(k_est / 1000.0, 0.0) if valid else 0.0,
            "compliance_mm_per_N": max(compliance, 0.0) if valid else 0.0,
            "peak_force_N": peak,
            "indentation_mm": indentation_max * 1000.0,
            "compression_mm": compression_max * 1000.0,
            "hold_force_ratio": hold_ratio,
        },
        contact_seconds=contact_steps * scene.dt,
        params={
            "depth": depth,
            "target_force": target_force,
            "force_limit": force_limit,
            "hold_time": hold_time,
            "effector": "ff_tip",
            "sensor": "ff_tip_touch",
        },
        raw_summary={
            "n_loading_samples": len(loading_trace),
            "n_hold_samples": len(hold_force),
            "contact_z": contact_z,
            "force_baseline": force_baseline,
            "contact_geom_whitelist": [
                _ALLEGRO_SURFACE_FINGERTIP_GEOM
            ],
        },
        valid=valid,
    )


def poke(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    depth: float | None = None,
    target_force: float | None = None,
    force_limit: float | None = None,
    contact_threshold: float = 0.05,
    lateral_ratio_limit: float = 0.35,
    *,
    hold_time: float | None = None,
    mode: str | None = None,
    protocol_id: str = PROBE_PROTOCOL_ID,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(
        backend,
        "poke",
        int(target),
        mode=canonical_probe_mode("poke", mode),
        protocol_id=validate_protocol_id(protocol_id),
    )
    if backend.name == "allegro":
        resolved_depth = float(
            V1_DEFAULTS.allegro_poke_max_depth_m if depth is None else depth
        )
        resolved_target_force = float(
            V1_DEFAULTS.allegro_poke_target_force_N
            if target_force is None
            else target_force
        )
        resolved_force_limit = float(
            V1_DEFAULTS.allegro_poke_force_limit_N
            if force_limit is None
            else force_limit
        )
        resolved_hold_time = float(
            V1_DEFAULTS.allegro_poke_hold_s
            if hold_time is None
            else hold_time
        )
        _validate_poke_admission(
            depth=resolved_depth,
            target_force=resolved_target_force,
            force_limit=resolved_force_limit,
            contact_threshold=contact_threshold,
            lateral_ratio_limit=lateral_ratio_limit,
            hold_time=resolved_hold_time,
        )
        return _poke_with_allegro_fingertip(
            run,
            depth=resolved_depth,
            target_force=resolved_target_force,
            force_limit=resolved_force_limit,
            contact_threshold=float(contact_threshold),
            lateral_ratio_limit=float(lateral_ratio_limit),
            hold_time=resolved_hold_time,
        )

    depth = float(V1_DEFAULTS.poke_max_depth_m if depth is None else depth)
    target_force = float(
        V1_DEFAULTS.poke_target_force_N if target_force is None else target_force
    )
    force_limit = float(
        V1_DEFAULTS.poke_force_limit_N if force_limit is None else force_limit
    )
    hold_time = float(0.0 if hold_time is None else hold_time)
    _validate_poke_admission(
        depth=depth,
        target_force=target_force,
        force_limit=force_limit,
        contact_threshold=contact_threshold,
        lateral_ratio_limit=lateral_ratio_limit,
        hold_time=hold_time,
    )
    run.sensor_profile_id = "sim.central_probe_touch+probe_ft.v1"
    run.target_penetration_limit_m = 0.001
    scene = run.scene
    x = scene.candidate_x(run.target)

    run.enter("approach")
    approached = _probe_above(run, x=x, clearance=0.030)
    force_baseline = (
        _mean_vec(run, scene.probe_force_vec, 40)
        if approached and not run.violations
        else np.zeros(3, dtype=float)
    )

    run.enter("guarded_contact")
    contact_z = None
    object_z = None
    extension = 0.0
    for extension in (
        np.linspace(0.0, 0.17, 120) if not run.violations else ()
    ):
        scene.command(probe=float(extension))
        if not run.step(4):
            break
        fn = scene.probe_touch()
        contact = scene.probe_contact_snapshot(run.target)
        if fn > force_limit:
            run.fail("force_limit")
            break
        if fn >= contact_threshold and contact.target_contact:
            contact_z = float(scene.probe_tip_pos()[2])
            object_z = float(scene.object_pos(run.target)[2])
            break
    if contact_z is None:
        run.fail("no_contact")

    run.enter("contact_quality_gate")
    trace: List[Tuple[float, float, float, float]] = []
    contact_steps = 0
    if contact_z is not None:
        stable_identity_steps = 0
        for _ in range(20):
            if not run.step(1):
                break
            contact = scene.probe_contact_snapshot(run.target)
            if contact.target_contact:
                stable_identity_steps += 1
            else:
                stable_identity_steps = 0
        run.quality["contact_identity_stable_fraction"] = (
            stable_identity_steps / 20.0
        )
        if stable_identity_steps < 16:
            run.fail("unstable_target_contact")
        run.enter("primitive_execution")
        integral = 0.0
        force_stable_steps = 0
        touch_lost_steps = 0
        for _ in range(max(60, int(0.7 / scene.dt))):
            fvec = scene.probe_force_vec() - force_baseline
            fn = float(abs(fvec[2]))
            ft = float(np.linalg.norm(fvec[:2]))
            integral = float(np.clip(integral + (target_force - fn) * scene.dt, -1, 1))
            extension = float(
                np.clip(
                    extension + 0.00028 * (target_force - fn) + 0.00008 * integral,
                    0.0,
                    0.17,
                )
            )
            scene.command(probe=extension)
            if not run.step(1):
                break
            z = float(scene.probe_tip_pos()[2])
            indentation = max(contact_z - z, 0.0)
            compression = max(float(object_z) - float(scene.object_pos(run.target)[2]), 0.0)
            touch = scene.probe_touch()
            fvec = scene.probe_force_vec() - force_baseline
            fn = float(abs(fvec[2]))
            ft = float(np.linalg.norm(fvec[:2]))
            trace.append((indentation, compression, fn, ft))
            run.sample(
                force_N=fn,
                touch_force_N=touch,
                probe_force_vector_N=fvec,
                indentation_m=indentation,
                compression_m=compression,
                compression_joint_q=float(
                    scene.data.qpos[
                        scene.joint_qadr[f"obj{run.target}_compress"]
                    ]
                ),
                lateral_force_N=ft,
            )
            if fn > force_limit:
                run.fail("force_limit")
                break
            if touch < 0.2 * contact_threshold and fn < 0.2 * target_force:
                touch_lost_steps += 1
            else:
                touch_lost_steps = 0
            if touch_lost_steps > 20:
                run.fail("lost_contact")
                break
            if fn > 0.5 * target_force and ft / max(fn, 1e-6) > lateral_ratio_limit:
                run.fail("excess_lateral_force")
                break
            contact_steps += 1
            if abs(fn - target_force) <= 0.12 * target_force:
                force_stable_steps += 1
            else:
                force_stable_steps = 0
            if compression >= depth or force_stable_steps >= 30:
                break

    run.enter("post_check")
    if len(trace) >= 3:
        probe_x = np.asarray([p[0] for p in trace])
        comp_x = np.asarray([p[1] for p in trace])
        force = np.asarray([p[2] for p in trace])
        x_fit = comp_x if np.ptp(comp_x) > 1e-5 else probe_x
        mask = (x_fit > 5e-5) & (force > contact_threshold)
        if mask.sum() >= 2 and np.ptp(x_fit[mask]) > 1e-5:
            k_est = float(np.polyfit(x_fit[mask], force[mask], 1)[0])
        else:
            k_est = 0.0
        peak = float(force.max())
        indentation_max = float(probe_x.max())
        compression_max = float(comp_x.max())
    else:
        k_est = peak = indentation_max = compression_max = 0.0

    effective = compression_max if compression_max > 1e-5 else indentation_max
    if k_est <= 0 and effective > 1e-6 and peak > 1e-6:
        k_est = peak / effective
    compliance = effective / peak * 1000.0 if peak > 1e-6 else 0.0
    path_fraction = min(effective / max(depth, 1e-9), 1.0)
    run.quality["path_completion_ratio"] = path_fraction
    run.quality["peak_force_N"] = peak
    run.quality["target_force_ratio"] = peak / max(target_force, 1e-9)
    # Poke is force-controlled: a stiff object is expected to reach the requested
    # force before the nominal maximum depth.  Depth is a safety/feature bound,
    # not a required path length.
    if peak < 0.80 * target_force and path_fraction < 0.80:
        run.fail("target_force_not_reached")
    if len(trace) < 3:
        run.fail("insufficient_samples")
    valid = not run.violations

    retreat_ok = _safe_probe_retreat(run)
    valid = valid and retreat_ok and not run.violations
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "k_est_N_per_m": max(k_est, 0.0) if valid else 0.0,
            "k_est_N_per_mm": max(k_est / 1000.0, 0.0) if valid else 0.0,
            "compliance_mm_per_N": max(compliance, 0.0) if valid else 0.0,
            "peak_force_N": peak,
            "indentation_mm": indentation_max * 1000.0,
            "compression_mm": compression_max * 1000.0,
        },
        contact_seconds=contact_steps * scene.dt,
        params={
            "depth": depth,
            "target_force": target_force,
            "force_limit": force_limit,
            "hold_time": hold_time,
            "effector": "central_probe",
            "sensor": "probe_touch",
        },
        raw_summary={
            "n_samples": len(trace),
            "contact_z": contact_z,
            "force_baseline": force_baseline,
        },
        valid=valid,
    )


def heft(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    lift_height: float | None = None,
    hold_time: float = V1_DEFAULTS.heft_hold_s,
    osc_amp: float = 0.0,
    osc_freq: float = 1.5,
    penetration_limit: float = 0.0055,
    min_grasp_force: float | None = None,
    *,
    target_object_lift: float = V1_DEFAULTS.micro_lift_target_m,
    lift_tolerance: float = V1_DEFAULTS.micro_lift_tolerance_m,
    max_lift_speed: float = V1_DEFAULTS.micro_lift_speed_m_per_s,
    support_loss_dwell: float = V1_DEFAULTS.support_loss_dwell_s,
    lift_stable_dwell: float = V1_DEFAULTS.micro_lift_stable_dwell_s,
    max_object_lift: float = V1_DEFAULTS.max_object_lift_m,
    mode: str | None = None,
    protocol_id: str = PROBE_PROTOCOL_ID,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(
        backend,
        "heft",
        int(target),
        mode=canonical_probe_mode("heft", mode),
        protocol_id=validate_protocol_id(protocol_id),
    )
    run.sensor_profile_id = "sim.wrist_ft+collision_contact.v1"
    run.target_penetration_limit_m = float(penetration_limit)
    scene = run.scene
    commanded_lift = float(
        lift_height
        if lift_height is not None
        else V1_DEFAULTS.micro_lift_max_wrist_travel_m
    )
    required_force = (
        float(min_grasp_force)
        if min_grasp_force is not None
        else (7.0 if backend.name == "allegro" else 3.0)
    )
    hold_time = _finite("hold_time", hold_time)
    if hold_time <= 0.0:
        raise ValueError("hold_time must be positive")
    if hold_time > 1.0:
        raise ValueError("hold_time exceeds the 1 s protocol cap")
    if _finite("osc_amp", osc_amp) != 0.0:
        raise ValueError(
            "unsupported_micro_lift is a static protocol; osc_amp must be zero"
        )
    if _finite("osc_freq", osc_freq) <= 0.0:
        raise ValueError("osc_freq must be positive")
    _validate_lift_admission(
        lift_height=commanded_lift,
        target_object_lift=target_object_lift,
        lift_tolerance=lift_tolerance,
        max_lift_speed=max_lift_speed,
        support_loss_dwell=support_loss_dwell,
        lift_stable_dwell=lift_stable_dwell,
        max_object_lift=max_object_lift,
        penetration_limit=penetration_limit,
        min_grasp_force=required_force,
    )
    measurement_steps = max(1, int(hold_time / scene.dt))
    grasp = _prepare_grasp(
        run,
        penetration_limit=penetration_limit,
        min_grasp_force=required_force,
    )
    valid, snapshot = _lift_and_gate(
        run,
        grasp,
        lift_height=commanded_lift,
        target_object_lift=float(target_object_lift),
        lift_tolerance=float(lift_tolerance),
        max_lift_speed=float(max_lift_speed),
        support_loss_dwell=float(support_loss_dwell),
        lift_stable_dwell=float(lift_stable_dwell),
        max_object_lift=float(max_object_lift),
        penetration_limit=penetration_limit,
    )

    force_trace = []
    gravity_force_trace = []
    measurement_min_lift = float(run.quality.get("lift_distance_m", 0.0))
    measurement_max_lift = measurement_min_lift
    if valid:
        run.enter("measurement")
        z0 = _ctrl(scene, "act_wz")
        z_hold = z0
        n = measurement_steps
        invalid_contact_steps = 0
        low_height_steps = 0
        high_height_steps = 0
        for k in range(n):
            actual_lift = float(
                scene.object_center_pos(run.target)[2]
                - run.quality["lift_start_center_z_m"]
            )
            if actual_lift < target_object_lift - lift_tolerance:
                z_hold = min(
                    z_hold + 0.005 * scene.dt,
                    z0 + 0.003,
                )
            # unsupported_micro_lift is deliberately static. Do not even
            # evaluate the deprecated oscillation expression: 0*sin(inf) is
            # NaN and could otherwise poison MuJoCo control for an extreme but
            # finite legacy osc_freq value.
            scene.command(z=z_hold)
            if not run.step(1):
                valid = False
                break
            actual_lift = float(
                scene.object_center_pos(run.target)[2]
                - run.quality["lift_start_center_z_m"]
            )
            measurement_min_lift = min(measurement_min_lift, actual_lift)
            measurement_max_lift = max(measurement_max_lift, actual_lift)
            current = scene.contact_snapshot(run.target)
            if grasp.top_entry:
                grasp.close_alpha, safe = _regulate_top_pinch(
                    run,
                    grasp.close_alpha,
                    current,
                    target_force_N=grasp.target_force_N,
                )
                if not safe:
                    valid = False
                    break
            sample = scene.wrist_force_vec() - grasp.baseline_force
            force_trace.append(sample.copy())
            gravity_sample = _force_along_world_z(scene, sample)
            gravity_force_trace.append(gravity_sample)
            run.sample(
                wrist_force_delta_N=sample,
                gravity_axis_force_delta_N=gravity_sample,
                object_lift_m=actual_lift,
            )
            if actual_lift > max_object_lift:
                valid = False
                run.fail("object_lift_limit_during_measurement")
                break
            if actual_lift < target_object_lift - lift_tolerance:
                low_height_steps += 1
            else:
                low_height_steps = 0
            if actual_lift > target_object_lift + lift_tolerance:
                high_height_steps += 1
            else:
                high_height_steps = 0
            if low_height_steps > 20:
                valid = False
                run.fail("object_lift_below_measurement_band")
                break
            if high_height_steps > 20:
                valid = False
                run.fail("object_lift_above_measurement_band")
                break
            if current.support_contact or current.table_contact:
                valid = False
                run.fail("support_contact_during_measurement")
                break
            if not _legal_grasp(run, current):
                invalid_contact_steps += 1
            else:
                invalid_contact_steps = 0
            if invalid_contact_steps > 20:
                valid = False
                run.fail("grasp_lost_during_measurement")
                break
    run.quality["measurement_min_object_lift_m"] = measurement_min_lift
    run.quality["measurement_max_object_lift_m"] = measurement_max_lift
    if force_trace:
        gravity_arr = np.asarray(gravity_force_trace)
        weight_signal = float(abs(np.median(gravity_arr))) if valid else 0.0
        force_std = float(np.std(gravity_arr))
    else:
        weight_signal = force_std = 0.0
    m_est = weight_signal / 9.81

    object_lifted = bool(
        grasp.established and run.quality.get("lift_started", 0.0) > 0.5
    )
    cleanup_ok = _place_and_retreat(run, grasp, object_lifted=object_lifted)
    valid = valid and cleanup_ok and not run.violations
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "m_est_kg": m_est if valid else 0.0,
            "weight_signal_N": weight_signal if valid else 0.0,
            "Fz_delta_median_N": weight_signal if valid else 0.0,
            "Fz_delta_std_N": force_std,
            "lifted": float(valid),
            "hand_contact_group_count": float(len(snapshot.hand_groups)),
        },
        contact_seconds=len(force_trace) * scene.dt,
        params={
            "lift_height": commanded_lift,
            "target_object_lift": target_object_lift,
            "lift_tolerance": lift_tolerance,
            "max_lift_speed": max_lift_speed,
            "support_loss_dwell": support_loss_dwell,
            "lift_stable_dwell": lift_stable_dwell,
            "max_object_lift": max_object_lift,
            "hold_time": hold_time,
            "osc_amp": osc_amp,
            "osc_freq": osc_freq,
            "penetration_limit": penetration_limit,
        },
        raw_summary={
            "baseline_wrist_force": grasp.baseline_force,
            "baseline_wrist_torque": grasp.baseline_torque,
            "close_alpha": grasp.close_alpha,
        },
        valid=valid,
    )


def shake(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    lift_height: float | None = None,
    tilt_amp: float = V1_DEFAULTS.shake_tilt_amplitude_rad,
    yaw_amp: float = V1_DEFAULTS.shake_yaw_amplitude_rad,
    freq: float = V1_DEFAULTS.shake_frequency_Hz,
    duration: float = V1_DEFAULTS.shake_duration_s,
    penetration_limit: float = 0.0055,
    min_grasp_force: float | None = None,
    *,
    dynamic_baseline_time: float = 0.200,
    ramp_cycles: float = 0.25,
    analysis_cycles: int | None = None,
    post_zero_hold_time: float = 0.120,
    max_dynamic_translation_drift: float = 0.005,
    max_dynamic_rotation_drift: float = float(np.deg2rad(6.1)),
    minimum_bottom_clearance: float = (
        V1_DEFAULTS.shake_min_bottom_clearance_m
    ),
    target_object_lift: float | None = None,
    lift_tolerance: float = V1_DEFAULTS.micro_lift_tolerance_m,
    max_lift_speed: float = V1_DEFAULTS.micro_lift_speed_m_per_s,
    support_loss_dwell: float = V1_DEFAULTS.support_loss_dwell_s,
    lift_stable_dwell: float = V1_DEFAULTS.shake_lift_stable_dwell_s,
    max_object_lift: float = V1_DEFAULTS.max_object_lift_m,
    mode: str | None = None,
    protocol_id: str = PROBE_PROTOCOL_ID,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(
        backend,
        "shake",
        int(target),
        mode=canonical_probe_mode("shake", mode),
        protocol_id=validate_protocol_id(protocol_id),
    )
    run.sensor_profile_id = "sim.wrist_ft+collision_contact.v1"
    run.target_penetration_limit_m = float(penetration_limit)
    scene = run.scene
    freq = _finite("freq", freq)
    duration = _finite("duration", duration)
    tilt_amp = _finite("tilt_amp", tilt_amp)
    yaw_amp = _finite("yaw_amp", yaw_amp)
    ramp_cycles = _finite("ramp_cycles", ramp_cycles)
    dynamic_baseline_time = _finite(
        "dynamic_baseline_time", dynamic_baseline_time
    )
    post_zero_hold_time = _finite(
        "post_zero_hold_time", post_zero_hold_time
    )
    max_dynamic_translation_drift = _finite(
        "max_dynamic_translation_drift", max_dynamic_translation_drift
    )
    max_dynamic_rotation_drift = _finite(
        "max_dynamic_rotation_drift", max_dynamic_rotation_drift
    )
    minimum_bottom_clearance = _finite(
        "minimum_bottom_clearance", minimum_bottom_clearance
    )
    if not 0.5 <= freq <= 5.0:
        raise ValueError("freq must be in [0.5, 5.0] Hz")
    if not 0.1 <= duration <= 2.0:
        raise ValueError("duration must be in [0.1, 2.0] s")
    if not 0.0 <= ramp_cycles <= 1.0:
        raise ValueError("ramp_cycles must be in [0, 1]")
    if dynamic_baseline_time <= 0.0 or post_zero_hold_time <= 0.0:
        raise ValueError("baseline and post-zero hold times must be positive")
    if dynamic_baseline_time > 1.0 or post_zero_hold_time > 1.0:
        raise ValueError("baseline and post-zero hold times must not exceed 1 s")
    if (
        max_dynamic_translation_drift <= 0.0
        or max_dynamic_rotation_drift <= 0.0
    ):
        raise ValueError("dynamic drift limits must be positive")
    if max_dynamic_translation_drift > 0.020:
        raise ValueError("dynamic translation drift limit exceeds 20 mm")
    if max_dynamic_rotation_drift > np.deg2rad(30.0):
        raise ValueError("dynamic rotation drift limit exceeds 30 degrees")
    if not (
        V1_DEFAULTS.shake_min_bottom_clearance_m
        <= minimum_bottom_clearance
        <= 0.010
    ):
        raise ValueError(
            "minimum_bottom_clearance must be in [1.5, 10] mm"
        )
    if analysis_cycles is not None and (
        not isinstance(analysis_cycles, (int, np.integer))
        or int(analysis_cycles) <= 0
    ):
        raise ValueError("analysis_cycles must be a positive integer")
    if abs(tilt_amp) <= 1e-9:
        raise ValueError("unsupported_micro_shake requires non-zero tilt_amp")
    if abs(tilt_amp) > np.deg2rad(10.0):
        raise ValueError("tilt_amp exceeds the 10 degree protocol cap")
    if abs(yaw_amp) > 1e-9:
        raise ValueError(
            "v1 lock-in uses one tilt input; yaw excitation must be run "
            "as a separate future protocol"
        )
    if not scene.task.objects[run.target].container_sealed:
        raise ValueError("unsupported_micro_shake requires a sealed container track")
    commanded_lift = float(
        lift_height
        if lift_height is not None
        else V1_DEFAULTS.micro_lift_max_wrist_travel_m
    )
    obj = scene.task.objects[run.target]
    max_angle = max(abs(tilt_amp), abs(yaw_amp))
    required_shake_clearance = float(
        V1_DEFAULTS.micro_lift_target_m
        + obj.size[0] * np.sin(max_angle)
        + obj.size[2] * (1.0 - np.cos(max_angle))
        + V1_DEFAULTS.shake_geometric_margin_m
    )
    if target_object_lift is None:
        resolved_with_band = (
            required_shake_clearance
            + float(lift_tolerance)
            + V1_DEFAULTS.shake_dynamic_sag_reserve_m
        )
        if resolved_with_band > max_object_lift:
            raise ValueError(
                "shake clearance requires more than the 15 mm v1 object-lift cap; "
                "reduce tilt_amp"
            )
        # The geometric sweep is a minimum clearance.  Centre the controller
        # band one tolerance above it so the lower edge remains collision-free.
        resolved_object_lift = float(resolved_with_band)
    else:
        resolved_object_lift = _finite(
            "target_object_lift", target_object_lift
        )
        if (
            resolved_object_lift - float(lift_tolerance)
            < required_shake_clearance
            + V1_DEFAULTS.shake_dynamic_sag_reserve_m
        ):
            raise ValueError(
                "target_object_lift tolerance band does not provide shake "
                "clearance plus the calibrated dynamic-sag reserve"
            )
    if resolved_object_lift >= max_object_lift:
        raise ValueError("target_object_lift reaches the shake object-lift safety cap")
    required_force = (
        float(min_grasp_force)
        if min_grasp_force is not None
        else (7.0 if backend.name == "allegro" else 3.0)
    )
    _validate_lift_admission(
        lift_height=commanded_lift,
        target_object_lift=resolved_object_lift,
        lift_tolerance=lift_tolerance,
        max_lift_speed=max_lift_speed,
        support_loss_dwell=support_loss_dwell,
        lift_stable_dwell=lift_stable_dwell,
        max_object_lift=max_object_lift,
        penetration_limit=penetration_limit,
        min_grasp_force=required_force,
    )
    resolved_analysis_cycles = (
        max(1, int(round(duration * freq)))
        if analysis_cycles is None
        else int(analysis_cycles)
    )
    if resolved_analysis_cycles > 8:
        raise ValueError("analysis_cycles exceeds the protocol cap of 8")
    n_baseline = max(4, int(dynamic_baseline_time / scene.dt))
    ramp_steps = max(0, int(round(ramp_cycles / (freq * scene.dt))))
    analysis_steps = max(
        8, int(round(resolved_analysis_cycles / (freq * scene.dt)))
    )
    n_zero = max(1, int(post_zero_hold_time / scene.dt))
    if max(n_baseline, ramp_steps, analysis_steps, n_zero) > 20_000:
        raise ValueError("derived shake step count exceeds the admission cap")
    grasp = _prepare_grasp(
        run,
        penetration_limit=penetration_limit,
        min_grasp_force=required_force,
    )
    heft_valid, snapshot = _lift_and_gate(
        run,
        grasp,
        lift_height=commanded_lift,
        target_object_lift=resolved_object_lift,
        lift_tolerance=float(lift_tolerance),
        max_lift_speed=float(max_lift_speed),
        support_loss_dwell=float(support_loss_dwell),
        lift_stable_dwell=float(lift_stable_dwell),
        max_object_lift=float(max_object_lift),
        penetration_limit=penetration_limit,
    )
    if not heft_valid:
        run.fail("heft_invalid")

    valid = heft_valid
    dynamic_baseline_force = np.zeros(3, dtype=float)
    dynamic_baseline_torque = np.zeros(3, dtype=float)
    baseline_force_samples: List[np.ndarray] = []
    baseline_torque_samples: List[np.ndarray] = []
    baseline_relative_positions: List[np.ndarray] = []
    baseline_relative_rotations: List[np.ndarray] = []
    snapshot = scene.contact_snapshot(run.target)
    shake_min_lift = float(run.quality.get("lift_distance_m", 0.0))
    shake_max_lift = shake_min_lift
    shake_min_geometric_margin = float("inf")
    shake_min_bottom_clearance = float("inf")
    shake_max_pose_predicted_center_requirement = 0.0
    shake_max_object_axis_tilt = 0.0
    initial_wrist_lift_command = float(
        run.quality.get("wrist_lift_command_m", 0.0)
    )
    shake_wrist_origin = _ctrl(scene, "act_wz") - initial_wrist_lift_command
    shake_wrist_limit = shake_wrist_origin + commanded_lift
    shake_z_command = _ctrl(scene, "act_wz")
    shake_max_extra_wrist_correction = 0.0
    shake_height_correction_steps = 0

    def shake_height_ok(phase_name: str) -> bool:
        nonlocal shake_min_lift, shake_max_lift
        nonlocal shake_min_geometric_margin
        nonlocal shake_min_bottom_clearance
        nonlocal shake_max_pose_predicted_center_requirement
        nonlocal shake_max_object_axis_tilt
        actual_lift = float(
            scene.object_center_pos(run.target)[2]
            - run.quality["lift_start_center_z_m"]
        )
        actual_wrist_tilt = abs(float(scene.joint_position("wt")))
        object_axis_tilt = _object_axis_tilt_rad(scene, run.target)
        shake_max_object_axis_tilt = max(
            shake_max_object_axis_tilt, object_axis_tilt
        )
        instantaneous_sweep = float(
            obj.size[0] * np.sin(object_axis_tilt)
            + obj.size[2] * (1.0 - np.cos(object_axis_tilt))
        )
        pose_predicted_center_requirement = float(
            V1_DEFAULTS.micro_lift_target_m
            + instantaneous_sweep
            + V1_DEFAULTS.shake_geometric_margin_m
        )
        bottom_clearance = float(
            scene.object_lowest_exterior_z(run.target)
            - run.quality["lift_source_support_z_m"]
        )
        geometric_margin = bottom_clearance - minimum_bottom_clearance
        shake_min_lift = min(shake_min_lift, actual_lift)
        shake_max_lift = max(shake_max_lift, actual_lift)
        shake_min_bottom_clearance = min(
            shake_min_bottom_clearance, bottom_clearance
        )
        shake_min_geometric_margin = min(
            shake_min_geometric_margin, geometric_margin
        )
        shake_max_pose_predicted_center_requirement = max(
            shake_max_pose_predicted_center_requirement,
            pose_predicted_center_requirement,
        )
        run.sample(
            shake_object_lift_m=actual_lift,
            shake_actual_wrist_tilt_rad=actual_wrist_tilt,
            shake_object_pose_sweep_bound_m=instantaneous_sweep,
            shake_bottom_clearance_m=bottom_clearance,
            shake_geometric_margin_m=geometric_margin,
            shake_object_axis_tilt_rad=object_axis_tilt,
            shake_wrist_lift_command_m=(
                shake_z_command - shake_wrist_origin
            ),
        )
        if actual_lift > max_object_lift:
            run.fail(f"object_lift_limit_during_{phase_name}")
            return False
        if bottom_clearance < minimum_bottom_clearance:
            run.fail(f"object_below_clearance_during_{phase_name}")
            return False
        return True

    # Raise and settle before acquiring the dynamic baseline. During the
    # baseline/drive/return windows z is frozen, preserving a single commanded
    # input (tilt) for the lock-in transfer estimate.
    if valid:
        run.enter("height_stabilization")
        stabilization_tolerance = 0.00025
        stable_steps_required = max(1, int(np.ceil(0.080 / scene.dt)))
        remaining_wrist_travel = max(
            shake_wrist_limit - shake_z_command, 0.0
        )
        max_stabilization_steps = (
            int(
                np.ceil(
                    remaining_wrist_travel
                    / (float(max_lift_speed) * scene.dt)
                )
            )
            + stable_steps_required
            + 100
        )
        stable_steps = 0
        invalid_contact_steps = 0
        for _ in range(max_stabilization_steps):
            actual_lift = float(
                scene.object_center_pos(run.target)[2]
                - run.quality["lift_start_center_z_m"]
            )
            if actual_lift < resolved_object_lift - stabilization_tolerance:
                shake_height_correction_steps += 1
                shake_z_command = min(
                    shake_z_command + float(max_lift_speed) * scene.dt,
                    shake_wrist_limit,
                )
            scene.command(z=shake_z_command, tilt=0.0, yaw=0.0)
            shake_max_extra_wrist_correction = max(
                shake_max_extra_wrist_correction,
                shake_z_command
                - shake_wrist_origin
                - initial_wrist_lift_command,
            )
            if not run.step(1):
                valid = False
                break
            if not shake_height_ok("height_stabilization"):
                valid = False
                break
            snapshot = scene.contact_snapshot(run.target)
            if grasp.top_entry:
                grasp.close_alpha, safe = _regulate_top_pinch(
                    run,
                    grasp.close_alpha,
                    snapshot,
                    target_force_N=grasp.target_force_N,
                )
                if not safe:
                    valid = False
                    break
            if snapshot.support_contact or snapshot.table_contact:
                run.fail("support_contact_during_height_stabilization")
                valid = False
                break
            if _legal_grasp(run, snapshot):
                invalid_contact_steps = 0
            else:
                invalid_contact_steps += 1
            if invalid_contact_steps > 20:
                run.fail("lost_grasp_during_height_stabilization")
                valid = False
                break
            actual_lift = float(
                scene.object_center_pos(run.target)[2]
                - run.quality["lift_start_center_z_m"]
            )
            if (
                resolved_object_lift - stabilization_tolerance
                <= actual_lift
                <= resolved_object_lift + stabilization_tolerance
            ):
                stable_steps += 1
            else:
                stable_steps = 0
            if stable_steps >= stable_steps_required:
                break
            if (
                shake_z_command >= shake_wrist_limit - 1e-9
                and actual_lift
                < resolved_object_lift - stabilization_tolerance
            ):
                run.fail("shake_height_stabilization_not_reached")
                valid = False
                break
        if stable_steps < stable_steps_required and not run.violations:
            run.fail("shake_height_stabilization_not_stable")
            valid = False

    frozen_shake_z = shake_z_command

    if valid:
        run.enter("dynamic_baseline")
        invalid_contact_steps = 0
        for _ in range(n_baseline):
            scene.command(z=frozen_shake_z, tilt=0.0, yaw=0.0)
            if not run.step(1):
                valid = False
                break
            if not shake_height_ok("dynamic_baseline"):
                valid = False
                break
            snapshot = scene.contact_snapshot(run.target)
            if grasp.top_entry:
                grasp.close_alpha, safe = _regulate_top_pinch(
                    run,
                    grasp.close_alpha,
                    snapshot,
                    target_force_N=grasp.target_force_N,
                )
                if not safe:
                    valid = False
                    break
            if snapshot.support_contact or snapshot.table_contact:
                run.fail("support_contact_during_dynamic_baseline")
                valid = False
                break
            if _legal_grasp(run, snapshot):
                invalid_contact_steps = 0
            else:
                invalid_contact_steps += 1
            if invalid_contact_steps > 20:
                run.fail("lost_grasp_during_dynamic_baseline")
                valid = False
                break
            baseline_force_samples.append(scene.wrist_force_vec().copy())
            baseline_torque_samples.append(scene.wrist_torque_vec().copy())
            relative_position, relative_rotation = (
                scene.relative_object_pose_in_wrist(run.target)
            )
            baseline_relative_positions.append(relative_position.copy())
            baseline_relative_rotations.append(relative_rotation.copy())

    if baseline_force_samples:
        dynamic_baseline_force = np.median(
            np.asarray(baseline_force_samples), axis=0
        )
        dynamic_baseline_torque = np.median(
            np.asarray(baseline_torque_samples), axis=0
        )
        baseline_relative_position = np.median(
            np.asarray(baseline_relative_positions), axis=0
        )
        baseline_relative_rotation = baseline_relative_rotations[-1]
        baseline_torque_std = float(
            np.std(np.asarray(baseline_torque_samples)[:, 1])
        )
        supported_delta = [
            _force_along_world_z(scene, sample - grasp.baseline_force)
            for sample in baseline_force_samples
        ]
        weight_proxy = float(abs(np.median(supported_delta)))
    else:
        baseline_relative_position = np.zeros(3, dtype=float)
        baseline_relative_rotation = np.eye(3, dtype=float)
        baseline_torque_std = 0.0
        weight_proxy = 0.0
        if heft_valid and not run.violations:
            run.fail("insufficient_dynamic_baseline")
            valid = False

    dynamic_force_trace: List[np.ndarray] = []
    dynamic_torque_trace: List[np.ndarray] = []
    analysis_times: List[float] = []
    analysis_tilt: List[float] = []
    analysis_torque: List[np.ndarray] = []
    analysis_wrist_z: List[float] = []
    max_translation_drift = 0.0
    max_rotation_drift = 0.0
    max_actual_tilt = 0.0

    if valid:
        run.enter("measurement")
        invalid_contact_steps = 0
        total_steps = 2 * ramp_steps + analysis_steps
        for index in range(total_steps):
            time_value = (index + 1) * scene.dt
            if ramp_steps and index < ramp_steps:
                u = (index + 1) / ramp_steps
                envelope = u * u * (3.0 - 2.0 * u)
                in_analysis = False
            elif index < ramp_steps + analysis_steps:
                envelope = 1.0
                in_analysis = True
            elif ramp_steps:
                u = (index - ramp_steps - analysis_steps + 1) / ramp_steps
                inverse = 1.0 - u
                envelope = inverse * inverse * (3.0 - 2.0 * inverse)
                in_analysis = False
            else:
                envelope = 1.0
                in_analysis = True
            commanded_tilt = float(
                tilt_amp
                * envelope
                * np.sin(2.0 * np.pi * freq * time_value)
            )
            scene.command(
                z=frozen_shake_z, tilt=commanded_tilt, yaw=0.0
            )
            if not run.step(1):
                valid = False
                break
            if not shake_height_ok("shake"):
                valid = False
                break
            snapshot = scene.contact_snapshot(run.target)
            if grasp.top_entry:
                grasp.close_alpha, safe = _regulate_top_pinch(
                    run,
                    grasp.close_alpha,
                    snapshot,
                    target_force_N=grasp.target_force_N,
                )
                if not safe:
                    valid = False
                    break
            if snapshot.support_contact or snapshot.table_contact:
                run.fail("support_contact_during_shake")
                valid = False
                break
            if _legal_grasp(run, snapshot):
                invalid_contact_steps = 0
            else:
                invalid_contact_steps += 1
            if invalid_contact_steps > 20:
                run.fail("lost_grasp_during_shake")
                valid = False
                break

            force = scene.wrist_force_vec() - dynamic_baseline_force
            torque = scene.wrist_torque_vec() - dynamic_baseline_torque
            actual_tilt = scene.joint_position("wt")
            max_actual_tilt = max(max_actual_tilt, abs(actual_tilt))
            relative_position, relative_rotation = (
                scene.relative_object_pose_in_wrist(run.target)
            )
            translation_drift = float(
                np.linalg.norm(relative_position - baseline_relative_position)
            )
            rotation_drift = _rotation_distance_rad(
                baseline_relative_rotation, relative_rotation
            )
            max_translation_drift = max(
                max_translation_drift, translation_drift
            )
            max_rotation_drift = max(max_rotation_drift, rotation_drift)
            dynamic_force_trace.append(force.copy())
            dynamic_torque_trace.append(torque.copy())
            run.sample(
                commanded_tilt_rad=commanded_tilt,
                actual_tilt_rad=actual_tilt,
                wrist_force_dynamic_N=force,
                wrist_torque_dynamic_Nm=torque,
                relative_translation_drift_m=translation_drift,
                relative_rotation_drift_rad=rotation_drift,
                drive_window="analysis" if in_analysis else "ramp",
            )
            if translation_drift > max_dynamic_translation_drift:
                run.fail("dynamic_translation_drift")
                valid = False
                break
            if rotation_drift > max_dynamic_rotation_drift:
                run.fail("dynamic_rotation_drift")
                valid = False
                break
            if in_analysis:
                analysis_times.append(time_value)
                analysis_tilt.append(actual_tilt)
                analysis_torque.append(torque.copy())
                analysis_wrist_z.append(scene.joint_position("wz"))

    ringdown_torque_y: List[float] = []
    if valid:
        run.enter("return_to_zero")
        invalid_contact_steps = 0
        for _ in range(n_zero):
            scene.command(z=frozen_shake_z, tilt=0.0, yaw=0.0)
            if not run.step(1):
                valid = False
                break
            if not shake_height_ok("return_to_zero"):
                valid = False
                break
            snapshot = scene.contact_snapshot(run.target)
            if grasp.top_entry:
                grasp.close_alpha, safe = _regulate_top_pinch(
                    run,
                    grasp.close_alpha,
                    snapshot,
                    target_force_N=grasp.target_force_N,
                )
                if not safe:
                    valid = False
                    break
            if snapshot.support_contact or snapshot.table_contact:
                run.fail("support_contact_after_shake")
                valid = False
                break
            if _legal_grasp(run, snapshot):
                invalid_contact_steps = 0
            else:
                invalid_contact_steps += 1
            if invalid_contact_steps > 20:
                run.fail("lost_grasp_after_shake")
                valid = False
                break
            torque = scene.wrist_torque_vec() - dynamic_baseline_torque
            ringdown_torque_y.append(float(torque[1]))
            relative_position, relative_rotation = (
                scene.relative_object_pose_in_wrist(run.target)
            )
            translation_drift = float(
                np.linalg.norm(relative_position - baseline_relative_position)
            )
            rotation_drift = _rotation_distance_rad(
                baseline_relative_rotation, relative_rotation
            )
            max_translation_drift = max(
                max_translation_drift, translation_drift
            )
            max_rotation_drift = max(max_rotation_drift, rotation_drift)
            if translation_drift > max_dynamic_translation_drift:
                run.fail("post_zero_translation_drift")
                valid = False
                break
            if rotation_drift > max_dynamic_rotation_drift:
                run.fail("post_zero_rotation_drift")
                valid = False
                break

    run.enter("post_zero_check")
    final_tilt_error = abs(scene.joint_position("wt"))
    if valid and final_tilt_error > np.deg2rad(0.5):
        run.fail("wrist_not_returned_to_zero")
        valid = False
    run.quality["dynamic_relative_translation_drift_m"] = (
        max_translation_drift
    )
    run.quality["dynamic_relative_rotation_drift_rad"] = max_rotation_drift
    run.quality["max_actual_tilt_rad"] = max_actual_tilt
    run.quality["final_tilt_error_rad"] = final_tilt_error
    run.quality["dynamic_baseline_torque_std_Nm"] = baseline_torque_std
    run.quality["required_shake_clearance_m"] = required_shake_clearance
    run.quality["planned_shake_center_lift_m"] = required_shake_clearance
    run.quality["shake_dynamic_sag_reserve_m"] = (
        V1_DEFAULTS.shake_dynamic_sag_reserve_m
    )
    run.quality["shake_min_object_lift_m"] = shake_min_lift
    run.quality["shake_max_object_lift_m"] = shake_max_lift
    run.quality["shake_min_geometric_margin_m"] = (
        shake_min_geometric_margin
    )
    run.quality["shake_min_bottom_clearance_m"] = (
        shake_min_bottom_clearance
    )
    run.quality["shake_minimum_bottom_clearance_gate_m"] = (
        minimum_bottom_clearance
    )
    run.quality["shake_max_pose_predicted_center_requirement_m"] = (
        shake_max_pose_predicted_center_requirement
    )
    run.quality["shake_max_object_axis_tilt_rad"] = (
        shake_max_object_axis_tilt
    )
    run.quality["shake_extra_wrist_correction_m"] = (
        shake_max_extra_wrist_correction
    )
    run.quality["shake_max_wrist_lift_command_m"] = float(
        shake_z_command - shake_wrist_origin
    )
    run.quality["shake_height_correction_steps"] = float(
        shake_height_correction_steps
    )

    if len(analysis_times) >= 8:
        time_arr = np.asarray(analysis_times, dtype=float)
        tilt_arr = np.asarray(analysis_tilt, dtype=float)
        torque_arr = np.asarray(analysis_torque, dtype=float)
        c_tilt = _lockin_coefficient(tilt_arr, time_arr, freq)
        c_torque = _lockin_coefficient(torque_arr[:, 1], time_arr, freq)
        wrist_z_arr = np.asarray(analysis_wrist_z, dtype=float)
        c_wrist_z = _lockin_coefficient(wrist_z_arr, time_arr, freq)
        if abs(c_tilt) > 1e-9:
            transfer = c_torque / c_tilt
            dynamic_gain = float(abs(transfer))
            dynamic_phase = float(np.angle(transfer))
        else:
            dynamic_gain = 0.0
            dynamic_phase = 0.0
        input_amplitude = float(abs(c_tilt))
        torque_amplitude = float(abs(c_torque))
        angle_tracking_ratio = input_amplitude / max(abs(tilt_amp), 1e-9)
        angle_snr_db = _lockin_snr_db(tilt_arr, c_tilt, time_arr, freq)
        dynamic_snr_db = _lockin_snr_db(
            torque_arr[:, 1], c_torque, time_arr, freq
        )
        torque_rms = np.sqrt(np.mean(torque_arr * torque_arr, axis=0))
        torque_peak = float(np.max(np.linalg.norm(torque_arr, axis=1)))
        torque_axis_amplitudes = np.asarray(
            [
                abs(_lockin_coefficient(torque_arr[:, axis], time_arr, freq))
                for axis in range(3)
            ],
            dtype=float,
        )
        wrist_z_response_amplitude = float(abs(c_wrist_z))
        wrist_z_response_span = float(np.ptp(wrist_z_arr))
    else:
        dynamic_gain = dynamic_phase = input_amplitude = torque_amplitude = 0.0
        angle_tracking_ratio = 0.0
        angle_snr_db = dynamic_snr_db = float("-inf")
        torque_rms = np.zeros(3, dtype=float)
        torque_peak = 0.0
        torque_axis_amplitudes = np.zeros(3, dtype=float)
        wrist_z_response_amplitude = 0.0
        wrist_z_response_span = 0.0
        if heft_valid and not run.violations:
            run.fail("insufficient_lockin_samples")
            valid = False
    if valid and angle_tracking_ratio < 0.75:
        run.fail("insufficient_angle_tracking")
        valid = False
    if valid and angle_snr_db < 15.0:
        run.fail("insufficient_angle_snr")
        valid = False
    if valid and max_actual_tilt > 1.10 * abs(tilt_amp) + 0.002:
        run.fail("tilt_amplitude_limit")
        valid = False
    run.quality["analysis_wrist_z_command_span_m"] = 0.0
    run.quality["analysis_wrist_z_response_span_m"] = wrist_z_response_span
    run.quality["analysis_wrist_z_response_amplitude_m"] = (
        wrist_z_response_amplitude
    )

    ringdown_rms = float(
        np.sqrt(np.mean(np.square(ringdown_torque_y)))
    ) if ringdown_torque_y else 0.0
    # Compatibility aliases are retained, but only dynamic_torque_gain is the
    # uncalibrated v2 feature.  A ProbeBench locked-content calibration must be
    # subtracted before anything is named a pure slosh gain.
    slosh_proxy = torque_amplitude
    fill_proxy = dynamic_gain

    run.enter("post_check")

    object_lifted = bool(
        grasp.established and run.quality.get("lift_started", 0.0) > 0.5
    )
    cleanup_ok = _place_and_retreat(run, grasp, object_lifted=object_lifted)
    valid = valid and cleanup_ok and not run.violations
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "weight_proxy_N": weight_proxy if valid else 0.0,
            "fill_proxy": fill_proxy if valid else 0.0,
            "slosh_proxy": slosh_proxy if valid else 0.0,
            "dynamic_torque_gain_Nm_per_rad": dynamic_gain if valid else 0.0,
            "dynamic_torque_gain_y_Nm_per_rad": dynamic_gain if valid else 0.0,
            "dynamic_phase_lag_rad": dynamic_phase if valid else 0.0,
            "dynamic_torque_phase_y_rad": dynamic_phase if valid else 0.0,
            "dynamic_snr": dynamic_snr_db if np.isfinite(dynamic_snr_db) else -120.0,
            "dynamic_lockin_snr_db": (
                dynamic_snr_db if np.isfinite(dynamic_snr_db) else -120.0
            ),
            "angle_tracking_ratio": angle_tracking_ratio,
            "angle_snr_db": angle_snr_db if np.isfinite(angle_snr_db) else -120.0,
            "dynamic_input_amplitude_rad": input_amplitude,
            "dynamic_torque_amplitude_Nm": torque_amplitude,
            "dynamic_torque_amp_x_Nm": float(torque_axis_amplitudes[0]),
            "dynamic_torque_amp_y_Nm": float(torque_axis_amplitudes[1]),
            "dynamic_torque_amp_z_Nm": float(torque_axis_amplitudes[2]),
            "post_zero_ringdown_rms_y_Nm": ringdown_rms,
            "torque_peak_Nm": torque_peak,
            "torque_rms_x_Nm": float(torque_rms[0]),
            "torque_rms_y_Nm": float(torque_rms[1]),
            "torque_rms_z_Nm": float(torque_rms[2]),
            "lifted": float(heft_valid),
            "hand_contact_group_count": float(len(snapshot.hand_groups)),
        },
        contact_seconds=(
            len(baseline_force_samples)
            + len(dynamic_force_trace)
            + len(ringdown_torque_y)
        )
        * scene.dt,
        params={
            "lift_height": commanded_lift,
            "target_object_lift": resolved_object_lift,
            "lift_tolerance": lift_tolerance,
            "max_lift_speed": max_lift_speed,
            "support_loss_dwell": support_loss_dwell,
            "lift_stable_dwell": lift_stable_dwell,
            "max_object_lift": max_object_lift,
            "tilt_amp": tilt_amp,
            "yaw_amp": yaw_amp,
            "freq": freq,
            "duration": duration,
            "analysis_cycles": resolved_analysis_cycles,
            "analysis_duration": resolved_analysis_cycles / freq,
            "ramp_cycles": ramp_cycles,
            "dynamic_baseline_time": dynamic_baseline_time,
            "post_zero_hold_time": post_zero_hold_time,
            "max_dynamic_translation_drift": max_dynamic_translation_drift,
            "max_dynamic_rotation_drift": max_dynamic_rotation_drift,
            "minimum_bottom_clearance": minimum_bottom_clearance,
            "penetration_limit": penetration_limit,
        },
        raw_summary={
            "supported_baseline_wrist_force": grasp.baseline_force,
            "supported_baseline_wrist_torque": grasp.baseline_torque,
            "lifted_dynamic_baseline_wrist_force": dynamic_baseline_force,
            "lifted_dynamic_baseline_wrist_torque": dynamic_baseline_torque,
            "close_alpha": grasp.close_alpha,
            "heft_valid": heft_valid,
            "lockin_axis": "tilt_to_wrist_torque_y",
            "content_proxy_version": scene.task.objects[
                run.target
            ].content_proxy_version,
        },
        valid=valid,
    )


def _slide_fingertip_touch(run: _Run) -> float:
    scene = run.scene
    if run.backend.name == "allegro":
        return scene.fingertip_touch("ff")
    return scene.reference_slide_touch()


def _slide_fingertip_pos(run: _Run) -> np.ndarray:
    scene = run.scene
    if run.backend.name == "allegro":
        return scene.fingertip_positions()["ff"]
    return scene.reference_slide_pad_pos()


def _prepare_fingertip_slide(
    run: _Run, *, start_x: float
) -> Tuple[bool, np.ndarray]:
    """Place one physical fingertip/pad above the slide start point."""

    scene = run.scene
    target_top = scene.object_pos(run.target).copy()
    desired_tip = target_top.copy()
    desired_tip[0] = float(start_x)
    desired_tip[1] = 0.0
    desired_tip[2] += 0.035

    run.enter("approach")
    if run.backend.name == "allegro":
        if not scene.full_hand_collisions_compiled():
            run.fail("full_hand_collisions_required")
        if not run.violations:
            _move_allegro_hand(run, _ALLEGRO_POKE_PRESHAPE, 160)
        if not run.violations:
            _move_wrist(
                run,
                steps=220,
                x=float(start_x),
                y=0.0,
                z=0.12,
                probe=0.0,
            )
        if not run.violations:
            _move_wrist(
                run,
                steps=300,
                roll=float(np.pi),
                tilt=0.0,
                yaw=0.0,
            )
    else:
        # The reference pad is mounted 60 mm to the left of the wrist. Shift
        # the wrist so that this single pad, not the opposite jaw, is centred.
        scene.command(grip=0.0, probe=0.0)
        _move_wrist(
            run,
            steps=260,
            x=float(start_x),
            y=0.060,
            z=0.10,
            roll=0.0,
            tilt=0.0,
            yaw=0.0,
            probe=0.0,
        )

    # Calibrate from the live contact-site pose so changes to Allegro or the
    # reference-pad mount do not create a hidden hard-coded fingertip offset.
    for _ in range(2):
        if run.violations:
            break
        tip = _slide_fingertip_pos(run)
        _move_wrist(
            run,
            steps=260,
            x=_ctrl(scene, "act_wx") + float(desired_tip[0] - tip[0]),
            y=_ctrl(scene, "act_wy") + float(desired_tip[1] - tip[1]),
            z=_ctrl(scene, "act_wz") + float(desired_tip[2] - tip[2]),
            roll=(float(np.pi) if run.backend.name == "allegro" else 0.0),
            tilt=0.0,
            yaw=0.0,
            probe=0.0,
        )
    force_baseline = (
        _mean_vec(run, scene.wrist_force_vec, 40)
        if not run.violations
        else np.zeros(3, dtype=float)
    )
    return not run.violations, force_baseline


def _safe_fingertip_slide_retreat(run: _Run) -> bool:
    if run.backend.name == "allegro":
        return _safe_fingertip_retreat(run)

    scene = run.scene
    run.enter("retreat")
    moved = _move_wrist(
        run,
        steps=240,
        z=0.10,
        roll=0.0,
        tilt=0.0,
        yaw=0.0,
        probe=0.0,
    )
    clear = bool(
        scene.reference_slide_touch() <= 1e-4
        and not scene.contact_snapshot(run.target).hand_groups
    )
    if not clear:
        run.fail("fingertip_not_clear_after_retreat")
    run.enter("post_retreat_check")
    settled = run.step(20)
    return bool(moved and clear and settled)


def slide(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    preload: float = V1_DEFAULTS.slide_preload_N,
    distance: float = V1_DEFAULTS.slide_one_way_distance_m,
    duration: float = V1_DEFAULTS.slide_one_way_duration_s,
    force_limit: float = V1_DEFAULTS.slide_force_limit_N,
    recovery_steps: int = 30,
    *,
    mode: str | None = None,
    protocol_id: str = PROBE_PROTOCOL_ID,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(
        backend,
        "slide",
        int(target),
        mode=canonical_probe_mode("slide", mode),
        protocol_id=validate_protocol_id(protocol_id),
    )
    if backend.name == "allegro":
        run.sensor_profile_id = "sim.allegro_ff_tip_touch+wrist_ft.v1"
        effector_name = "ff_tip"
        sensor_name = "ff_tip_touch"
        contact_geom = _ALLEGRO_SURFACE_FINGERTIP_GEOM
    else:
        run.sensor_profile_id = "sim.reference_left_slide_touch+wrist_ft.v1"
        effector_name = "left_fingertip_pad"
        sensor_name = "ref_left_slide_touch"
        contact_geom = _REFERENCE_SLIDE_PAD_GEOM
    # The material surface intentionally uses a compliant contact solver. Keep
    # the fingertip below the former 1 mm central-probe envelope while allowing
    # the 0.6 N preload to settle without a solver-only false positive.
    run.target_penetration_limit_m = 0.0008
    scene = run.scene
    preload = _finite("preload", preload)
    distance = _finite("distance", distance)
    duration = _finite("duration", duration)
    force_limit = _finite("force_limit", force_limit)
    if preload <= 0.0:
        raise ValueError("preload must be positive")
    if abs(distance) <= 1e-6:
        raise ValueError("distance must be non-zero")
    if abs(distance) > 0.25:
        raise ValueError("distance exceeds the 250 mm admission cap")
    if not 0.05 <= duration <= 5.0:
        raise ValueError("duration must be in [0.05, 5.0] s")
    if force_limit < preload:
        raise ValueError("force_limit must be at least preload")
    if not isinstance(recovery_steps, (int, np.integer)) or recovery_steps < 0:
        raise ValueError("recovery_steps must be a non-negative integer")
    if recovery_steps > 5_000:
        raise ValueError("recovery_steps exceeds the admission cap")
    n = max(4, int(duration / scene.dt))
    if n > 20_000:
        raise ValueError("derived slide step count exceeds the admission cap")
    x0 = scene.candidate_x(run.target)
    start_x = x0 - 0.5 * distance

    approached, force_baseline = _prepare_fingertip_slide(
        run, start_x=start_x
    )

    run.enter("guarded_contact")
    z_command = _ctrl(scene, "act_wz")
    contacted = False
    max_guard_steps = max(1, int(np.ceil(0.050 / 0.00004)))
    for _ in range(max_guard_steps if approached else 0):
        z_command -= 0.00004
        scene.command(z=z_command, probe=0.0)
        if not run.step(3):
            break
        fn = _slide_fingertip_touch(run)
        contact = scene.contact_snapshot(run.target)
        if fn > force_limit:
            run.fail("force_limit")
            break
        # Detect first physical touch early.  The following quality phase ramps
        # to preload in feedback; descending open-loop to the final force causes
        # the stiff carriage servo to overshoot while its position catches up.
        if fn >= min(0.15, 0.25 * preload) and _legal_fingertip_slide_contact(
            run, contact
        ):
            contacted = True
            break
    if not contacted and not run.violations:
        run.fail("no_contact")

    run.enter("contact_quality_gate")
    integral = 0.0
    lost_count = 0
    overload_count = 0
    trace: List[Tuple[float, np.ndarray, float, bool, str]] = []
    max_lost_count = 0
    actual_path_fraction = 0.0
    return_path_fraction = 0.0
    endpoint_settle_steps = 0
    return_endpoint_settle_steps = 0
    return_error_m = abs(float(distance))
    start_tip_x = float("nan")
    max_object_displacement_m = 0.0
    object_start = scene.object_center_pos(run.target).copy()
    if contacted and not run.violations:
        stable_contact_steps = 0
        stable_forces = []
        for _ in range(240):
            fn = _slide_fingertip_touch(run)
            error = preload - fn
            integral = float(
                np.clip(integral + error * scene.dt, -0.5, 0.5)
            )
            dz = float(
                np.clip(
                    2.5e-5 * error + 4.0e-6 * integral,
                    -1.2e-5,
                    2.5e-5,
                )
            )
            z_command = float(
                np.clip(
                    z_command - dz,
                    *scene.model.actuator_ctrlrange[scene.act["act_wz"]],
                )
            )
            scene.command(z=z_command, probe=0.0)
            if not run.step(1):
                break
            contact = scene.contact_snapshot(run.target)
            fn = _slide_fingertip_touch(run)
            stable_forces.append(fn)
            if (
                _legal_fingertip_slide_contact(run, contact)
                and 0.80 * preload <= fn <= 1.20 * preload
            ):
                stable_contact_steps += 1
            else:
                stable_contact_steps = 0
            if fn > force_limit:
                run.fail("force_limit")
                break
            if stable_contact_steps >= 40:
                break
        run.quality["preload_stable_fraction"] = min(
            stable_contact_steps / 40.0, 1.0
        )
        if stable_contact_steps < 40:
            run.fail("unstable_preload")
        if stable_forces:
            run.quality["preload_force_std_N"] = float(np.std(stable_forces))

    if contacted and not run.violations:
        start_tip_x = float(_slide_fingertip_pos(run)[0])
        start_wrist_x = _ctrl(scene, "act_wx")
        z_ctrl_range = np.asarray(
            scene.model.actuator_ctrlrange[scene.act["act_wz"]], dtype=float
        )

        def command_contact(*, x: float) -> None:
            nonlocal integral, z_command
            fn_now = _slide_fingertip_touch(run)
            error = preload - fn_now
            integral = float(
                np.clip(integral + error * scene.dt, -0.5, 0.5)
            )
            dz = float(
                np.clip(
                    2.5e-5 * error + 4.0e-6 * integral,
                    -1.2e-5,
                    2.5e-5,
                )
            )
            z_command = float(np.clip(z_command - dz, *z_ctrl_range))
            scene.command(x=float(x), z=z_command, probe=0.0)

        def record_sample(
            *, leg: str, commanded_fraction: float, origin_x: float
        ) -> float:
            nonlocal lost_count, overload_count, max_lost_count
            nonlocal max_object_displacement_m
            fn_now = _slide_fingertip_touch(run)
            fvec = scene.wrist_force_vec() - force_baseline
            ft_now = float(np.linalg.norm(fvec[:2]))
            snapshot = scene.contact_snapshot(run.target)
            legal = _legal_fingertip_slide_contact(run, snapshot)
            if fn_now < 0.2 * preload or not legal:
                lost_count += 1
            else:
                lost_count = 0
            max_lost_count = max(max_lost_count, lost_count)
            fraction = float(
                np.clip(
                    abs(float(_slide_fingertip_pos(run)[0]) - origin_x)
                    / max(abs(distance), 1e-9),
                    0.0,
                    1.0,
                )
            )
            max_object_displacement_m = max(
                max_object_displacement_m,
                float(
                    np.linalg.norm(
                        scene.object_center_pos(run.target)[:2]
                        - object_start[:2]
                    )
                ),
            )
            trace.append((fn_now, fvec.copy(), fraction, legal, leg))
            run.sample(
                effector=effector_name,
                normal_force_N=fn_now,
                tangential_force_N=ft_now,
                wrist_force_delta_N=fvec,
                commanded_path_fraction=float(commanded_fraction),
                path_fraction=fraction,
                path_leg=leg,
                legal_contact=legal,
            )
            # A single-step complementarity impulse is not a sustained unsafe
            # load. Require 10 ms above the limit, while still preserving the
            # unfiltered peak in the collision audit.
            overload_count = overload_count + 1 if fn_now > force_limit else 0
            if overload_count >= 5:
                run.fail("force_limit")
            if lost_count > recovery_steps:
                run.fail("lost_contact")
            return fraction

        run.enter("primitive_execution")
        for alpha in np.linspace(0.0, 1.0, n):
            command_contact(x=start_wrist_x + float(alpha) * distance)
            if not run.step(1):
                break
            actual_path_fraction = record_sample(
                leg="outbound",
                commanded_fraction=float(alpha),
                origin_x=start_tip_x,
            )
            if run.violations:
                break

        # Measured fingertip pose, rather than actuator command, is the endpoint
        # criterion. A small bounded lead compensates position-servo lag.
        servo_lead = 0.004 * (1.0 if distance >= 0.0 else -1.0)
        for settle_index in range(200):
            if actual_path_fraction >= 0.95 or run.violations:
                break
            lead_alpha = min((settle_index + 1) / 80.0, 1.0)
            command_contact(
                x=start_wrist_x + distance + lead_alpha * servo_lead
            )
            if not run.step(1):
                break
            endpoint_settle_steps += 1
            actual_path_fraction = record_sample(
                leg="outbound",
                commanded_fraction=1.0,
                origin_x=start_tip_x,
            )

        forward_tip_x = float(_slide_fingertip_pos(run)[0])
        if actual_path_fraction >= 0.95 and not run.violations:
            lost_count = 0
            for alpha in np.linspace(0.0, 1.0, n):
                command_contact(
                    x=start_wrist_x + (1.0 - float(alpha)) * distance
                )
                if not run.step(1):
                    break
                return_path_fraction = record_sample(
                    leg="return",
                    commanded_fraction=float(alpha),
                    origin_x=forward_tip_x,
                )
                if run.violations:
                    break

            for settle_index in range(200):
                if return_path_fraction >= 0.95 or run.violations:
                    break
                lead_alpha = min((settle_index + 1) / 80.0, 1.0)
                command_contact(x=start_wrist_x - lead_alpha * servo_lead)
                if not run.step(1):
                    break
                return_endpoint_settle_steps += 1
                return_path_fraction = record_sample(
                    leg="return",
                    commanded_fraction=1.0,
                    origin_x=forward_tip_x,
                )
            return_error_m = abs(
                float(_slide_fingertip_pos(run)[0]) - start_tip_x
            )

    run.enter("post_check")
    completion = min(actual_path_fraction, return_path_fraction)
    run.quality["path_completion_ratio"] = completion
    run.quality["outbound_path_completion_ratio"] = actual_path_fraction
    run.quality["return_path_completion_ratio"] = return_path_fraction
    run.quality["round_trip_return_error_m"] = return_error_m
    run.quality["max_lost_contact_steps"] = float(max_lost_count)
    run.quality["endpoint_settle_steps"] = float(endpoint_settle_steps)
    run.quality["return_endpoint_settle_steps"] = float(
        return_endpoint_settle_steps
    )
    run.quality["max_object_xy_displacement_m"] = max_object_displacement_m
    if completion < 0.95:
        run.fail("path_incomplete")
    if return_error_m > 0.10 * abs(distance):
        run.fail("round_trip_return_error")
    # A surface probe that pushes the candidate around has not measured the
    # intended local material property, even if the carriage completed its path.
    if max_object_displacement_m > 0.003:
        run.fail("object_displacement_limit")

    if trace:
        fn_arr = np.asarray([item[0] for item in trace])
        force_arr = np.asarray([item[1] for item in trace])
        path_arr = np.asarray([item[2] for item in trace])
        ft_arr = np.linalg.norm(force_arr[:, :2], axis=1)
        contact_mask = np.asarray(
            [bool(item[3]) for item in trace], dtype=bool
        ) & (fn_arr > 1e-4)
        # Exclude reversal/endpoint transients from the friction estimate while
        # retaining every sample for contact-completion diagnostics.
        steady_mask = contact_mask & (path_arr >= 0.10) & (path_arr <= 0.90)
        contact_fraction = float(contact_mask.mean())
        if steady_mask.any():
            ratio = ft_arr[steady_mask] / (fn_arr[steady_mask] + 1e-6)
            mu = float(np.median(ratio))
            ft_median = float(np.median(ft_arr[steady_mask]))
            fn_median = float(np.median(fn_arr[steady_mask]))
            vibration = float(np.std(fn_arr[steady_mask]))
            outbound_mask = steady_mask & np.asarray(
                [item[4] == "outbound" for item in trace], dtype=bool
            )
            return_mask = steady_mask & np.asarray(
                [item[4] == "return" for item in trace], dtype=bool
            )
            mu_outbound = (
                float(
                    np.median(
                        ft_arr[outbound_mask]
                        / (fn_arr[outbound_mask] + 1e-6)
                    )
                )
                if outbound_mask.any()
                else 0.0
            )
            mu_return = (
                float(
                    np.median(
                        ft_arr[return_mask]
                        / (fn_arr[return_mask] + 1e-6)
                    )
                )
                if return_mask.any()
                else 0.0
            )
        else:
            mu = ft_median = fn_median = vibration = 0.0
            mu_outbound = mu_return = 0.0
    else:
        mu = ft_median = fn_median = vibration = contact_fraction = 0.0
        mu_outbound = mu_return = 0.0
    run.quality["contact_fraction"] = contact_fraction
    if contact_fraction < 0.80:
        run.fail("insufficient_contact_fraction")
    valid = not run.violations

    retreat_ok = _safe_fingertip_slide_retreat(run)
    valid = valid and retreat_ok and not run.violations
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "mu_est": max(mu, 0.0) if valid else 0.0,
            "friction_ratio": max(mu, 0.0) if valid else 0.0,
            "Ft_median_N": ft_median,
            "Fn_median_N": fn_median,
            "slide_vibration": vibration,
            "friction_ratio_outbound": mu_outbound,
            "friction_ratio_return": mu_return,
            "friction_direction_asymmetry": abs(mu_outbound - mu_return),
        },
        contact_seconds=len(trace) * scene.dt,
        params={
            "preload": preload,
            "force_limit": force_limit,
            "distance": distance,
            "duration": duration,
            "round_trip": True,
            "recovery_steps": recovery_steps,
            "force_limit_dwell_steps": 5,
            "effector": effector_name,
            "sensor": sensor_name,
            "tangential_force_source": "wrist_force",
        },
        raw_summary={
            "n_samples": len(trace),
            "force_baseline": force_baseline,
            "contact_geom_whitelist": [contact_geom],
            "central_probe_collision_enabled": bool(
                scene.model.geom_contype[scene.geom["probe_tip_geom"]] != 0
            ),
        },
        valid=valid,
    )


_DISPATCH: Dict[str, Callable[..., ProbeResult]] = {
    "poke": poke,
    "heft": heft,
    "shake": shake,
    "slide": slide,
}


def run_probe(
    executor: ProbeBackend | AllegroProbeScene,
    primitive: str,
    target: int,
    protocol_id: str = PROBE_PROTOCOL_ID,
    **params: Any,
) -> ProbeResult:
    backend = as_backend(executor)
    primitive = str(primitive)
    if primitive not in _DISPATCH:
        raise ValueError(
            f"unknown probe primitive {primitive!r}; expected {sorted(_DISPATCH)}"
        )
    expected = primitive_for_family(backend.scene.task.family)
    if primitive != expected:
        raise ValueError(
            f"scene family {backend.scene.task.family!r} requires {expected!r}, "
            f"got {primitive!r}; collision roles are fixed at model compilation"
        )
    target = int(target)
    if not 0 <= target < backend.scene.n:
        raise IndexError(f"target {target} outside [0, {backend.scene.n})")
    return _DISPATCH[primitive](
        backend,
        target,
        protocol_id=validate_protocol_id(protocol_id),
        **params,
    )
