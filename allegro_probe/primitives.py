"""Probe-aware state machines and feature extraction."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Tuple

import numpy as np

from allegro_probe.backends import ProbeBackend, as_backend
from allegro_probe.models import ProbeResult, canonical_family
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
        "lift",
        "primitive_execution",
        "measurement",
        "post_check",
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
        if self.primitive in {"poke", "slide"}:
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
            backend=self.backend.name,
            status=controller_status,
            valid=bool(valid),
            controller_status=controller_status,
            phase_reached=self.phase,
            violations=list(self.violations),
            quality=dict(self.quality),
            features=features,
            contact_seconds=float(contact_seconds),
            params=params,
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
) -> Tuple[bool, ContactSnapshot, float]:
    scene = run.scene
    rel = []
    valid_steps = 0
    last = ContactSnapshot()
    max_penetration = 0.0
    for _ in range(int(steps)):
        if not run.step(1):
            break
        last = scene.contact_snapshot(run.target)
        max_penetration = max(max_penetration, last.max_penetration_m)
        opposing = _legal_grasp(run, last)
        support_ok = not require_support_free or (
            not last.support_contact and not last.table_contact
        )
        penetration_ok = last.max_penetration_m <= penetration_limit
        if opposing and support_ok and penetration_ok:
            valid_steps += 1
        rel.append(scene.relative_object_position(run.target).copy())
    rel_arr = np.asarray(rel, dtype=float)
    drift = (
        float(np.max(np.linalg.norm(rel_arr - rel_arr[0], axis=1)))
        if len(rel_arr)
        else float("inf")
    )
    run.quality[f"{quality_prefix}_relative_drift_m"] = drift
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
    penetration_limit: float,
) -> Tuple[bool, ContactSnapshot]:
    scene = run.scene
    if not grasp.established:
        run.enter("post_check")
        return False, scene.contact_snapshot(run.target)
    run.enter("lift")
    z0 = _ctrl(scene, "act_wz")
    steps = 700 if grasp.top_entry else 220
    lost_steps = 0
    progress = grasp.close_alpha
    for index in range(steps):
        u = (index + 1) / steps
        alpha = u * u * (3.0 - 2.0 * u)
        scene.command(z=z0 + alpha * lift_height)
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
    grasp.close_alpha = progress

    run.enter("post_check")
    stable, snapshot, drift = _contact_stable(
        run,
        steps=80,
        require_support_free=True,
        penetration_limit=penetration_limit,
        quality_prefix="postlift",
    )
    lifted_distance = float(
        scene.object_pos(run.target)[2] - grasp.initial_object_pos[2]
    )
    run.quality["lift_distance_m"] = lifted_distance
    run.quality["support_contact_after_lift"] = float(snapshot.support_contact)
    run.quality["table_contact_after_lift"] = float(snapshot.table_contact)
    run.quality["postlift_group_count"] = float(len(snapshot.hand_groups))

    valid = grasp.established and stable and not run.violations
    if lifted_distance < min(0.010, 0.5 * lift_height):
        valid = False
        run.fail("not_lifted")
    if snapshot.support_contact or snapshot.table_contact:
        valid = False
        run.fail("support_contact_after_lift")
    if not _legal_grasp(run, snapshot):
        valid = False
        run.fail("lost_grasp")
    if drift > 0.008:
        valid = False
        run.fail("postlift_slip")
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


def poke(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    depth: float = 0.006,
    target_force: float = 3.0,
    force_limit: float = 10.0,
    contact_threshold: float = 0.05,
    lateral_ratio_limit: float = 0.35,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(backend, "poke", int(target))
    run.target_penetration_limit_m = 0.001
    scene = run.scene
    x = scene.candidate_x(run.target)

    run.enter("approach")
    _probe_above(run, x=x, clearance=0.030)
    force_baseline = _mean_vec(run, scene.probe_force_vec, 40)

    run.enter("guarded_contact")
    contact_z = None
    object_z = None
    extension = 0.0
    for extension in np.linspace(0.0, 0.17, 120):
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
    hold_time: float = 0.45,
    osc_amp: float = 0.001,
    osc_freq: float = 1.5,
    penetration_limit: float = 0.0055,
    min_grasp_force: float | None = None,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(backend, "heft", int(target))
    run.target_penetration_limit_m = float(penetration_limit)
    scene = run.scene
    commanded_lift = float(
        lift_height
        if lift_height is not None
        else (0.130 if backend.name == "allegro" else 0.025)
    )
    required_force = (
        float(min_grasp_force)
        if min_grasp_force is not None
        else (7.0 if backend.name == "allegro" else 3.0)
    )
    grasp = _prepare_grasp(
        run,
        penetration_limit=penetration_limit,
        min_grasp_force=required_force,
    )
    valid, snapshot = _lift_and_gate(
        run,
        grasp,
        lift_height=commanded_lift,
        penetration_limit=penetration_limit,
    )

    force_trace = []
    if valid:
        run.enter("measurement")
        z0 = _ctrl(scene, "act_wz")
        n = max(1, int(hold_time / scene.dt))
        invalid_contact_steps = 0
        for k in range(n):
            t = k * scene.dt
            scene.command(z=z0 + osc_amp * np.sin(2 * np.pi * osc_freq * t))
            if not run.step(1):
                valid = False
                break
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
            run.sample(wrist_force_delta_N=sample)
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
    if force_trace:
        force_arr = np.asarray(force_trace)
        # Sensor sign depends on the welded child convention, so use the stable
        # gravity-axis magnitude after baseline subtraction.
        weight_signal = float(abs(np.median(force_arr[:, 2]))) if valid else 0.0
        force_std = float(np.std(force_arr[:, 2]))
    else:
        weight_signal = force_std = 0.0
    m_est = weight_signal / 9.81

    object_lifted = run.quality.get("lift_distance_m", 0.0) >= 0.010
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
            "hold_time": hold_time,
            "osc_amp": osc_amp,
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
    tilt_amp: float = 0.07,
    yaw_amp: float = 0.06,
    freq: float = 1.2,
    duration: float = 0.8,
    penetration_limit: float = 0.0055,
    min_grasp_force: float | None = None,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(backend, "shake", int(target))
    run.target_penetration_limit_m = float(penetration_limit)
    scene = run.scene
    commanded_lift = float(
        lift_height
        if lift_height is not None
        else (0.130 if backend.name == "allegro" else 0.025)
    )
    required_force = (
        float(min_grasp_force)
        if min_grasp_force is not None
        else (7.0 if backend.name == "allegro" else 3.0)
    )
    grasp = _prepare_grasp(
        run,
        penetration_limit=penetration_limit,
        min_grasp_force=required_force,
    )
    heft_valid, snapshot = _lift_and_gate(
        run,
        grasp,
        lift_height=commanded_lift,
        penetration_limit=penetration_limit,
    )
    if not heft_valid:
        run.fail("heft_invalid")

    force_trace = []
    torque_trace = []
    valid = heft_valid
    if valid:
        run.enter("measurement")
        n = max(1, int(duration / scene.dt))
        invalid_contact_steps = 0
        for k in range(n):
            t = k * scene.dt
            scene.command(
                tilt=tilt_amp * np.sin(2 * np.pi * freq * t),
                yaw=yaw_amp * np.sin(2 * np.pi * freq * t + np.pi / 2),
            )
            if not run.step(1):
                valid = False
                break
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
            force = scene.wrist_force_vec() - grasp.baseline_force
            torque = scene.wrist_torque_vec() - grasp.baseline_torque
            force_trace.append(force.copy())
            torque_trace.append(torque.copy())
            run.sample(wrist_force_delta_N=force, wrist_torque_delta_Nm=torque)
            if current.support_contact or current.table_contact:
                valid = False
                run.fail("support_contact_during_shake")
                break
            if not _legal_grasp(run, current):
                invalid_contact_steps += 1
            else:
                invalid_contact_steps = 0
            if invalid_contact_steps > 20:
                valid = False
                run.fail("lost_grasp_during_shake")
                break

    run.enter("post_check")
    if force_trace:
        force_arr = np.asarray(force_trace)
        torque_arr = np.asarray(torque_trace)
        weight_proxy = float(abs(np.median(force_arr[:, 2]))) if valid else 0.0
        torque_rms = np.sqrt(np.mean(torque_arr * torque_arr, axis=0))
        torque_norm = np.linalg.norm(torque_arr, axis=1)
        slosh_proxy = float(np.std(torque_arr[:, 0]) + np.std(torque_arr[:, 1]))
        torque_peak = float(np.max(torque_norm))
        fill_proxy = torque_peak + 0.02 * weight_proxy
    else:
        weight_proxy = slosh_proxy = torque_peak = fill_proxy = 0.0
        torque_rms = np.zeros(3)

    object_lifted = run.quality.get("lift_distance_m", 0.0) >= 0.010
    cleanup_ok = _place_and_retreat(run, grasp, object_lifted=object_lifted)
    valid = valid and cleanup_ok and not run.violations
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "weight_proxy_N": weight_proxy if valid else 0.0,
            "fill_proxy": fill_proxy if valid else 0.0,
            "slosh_proxy": slosh_proxy if valid else 0.0,
            "torque_peak_Nm": torque_peak,
            "torque_rms_x_Nm": float(torque_rms[0]),
            "torque_rms_y_Nm": float(torque_rms[1]),
            "torque_rms_z_Nm": float(torque_rms[2]),
            "lifted": float(heft_valid),
            "hand_contact_group_count": float(len(snapshot.hand_groups)),
        },
        contact_seconds=len(force_trace) * scene.dt,
        params={
            "lift_height": commanded_lift,
            "tilt_amp": tilt_amp,
            "yaw_amp": yaw_amp,
            "freq": freq,
            "penetration_limit": penetration_limit,
        },
        raw_summary={
            "baseline_wrist_force": grasp.baseline_force,
            "baseline_wrist_torque": grasp.baseline_torque,
            "close_alpha": grasp.close_alpha,
            "heft_valid": heft_valid,
        },
        valid=valid,
    )


def slide(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    preload: float = 2.0,
    distance: float = 0.040,
    duration: float = 0.8,
    force_limit: float = 12.0,
    recovery_steps: int = 30,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(backend, "slide", int(target))
    run.target_penetration_limit_m = 0.001
    scene = run.scene
    x0 = scene.candidate_x(run.target)
    start_x = x0 - 0.5 * distance

    run.enter("approach")
    _probe_above(run, x=start_x, clearance=0.030)
    force_baseline = _mean_vec(run, scene.probe_force_vec, 40)

    run.enter("guarded_contact")
    extension = 0.0
    contacted = False
    for extension in np.linspace(0.0, 0.17, 120):
        scene.command(probe=float(extension))
        if not run.step(4):
            break
        fn = scene.probe_touch()
        contact = scene.probe_contact_snapshot(run.target)
        if fn > force_limit:
            run.fail("force_limit")
            break
        if fn >= 0.8 * preload and contact.target_contact:
            contacted = True
            break
    if not contacted and not run.violations:
        run.fail("no_contact")

    run.enter("contact_quality_gate")
    integral = 0.0
    lost_count = 0
    trace = []
    completed_steps = 0
    max_lost_count = 0
    actual_path_fraction = 0.0
    endpoint_settle_steps = 0
    n = max(4, int(duration / scene.dt))
    if contacted and not run.violations:
        stable_contact_steps = 0
        stable_forces = []
        for _ in range(30):
            if not run.step(1):
                break
            contact = scene.probe_contact_snapshot(run.target)
            fn = scene.probe_touch()
            stable_forces.append(fn)
            if contact.target_contact and fn >= 0.4 * preload:
                stable_contact_steps += 1
            else:
                stable_contact_steps = 0
        run.quality["preload_stable_fraction"] = stable_contact_steps / 30.0
        if stable_contact_steps < 24:
            run.fail("unstable_preload")
        if stable_forces:
            run.quality["preload_force_std_N"] = float(np.std(stable_forces))
        start_tip_x = float(scene.probe_tip_pos()[0])
        run.enter("primitive_execution")
        for k, alpha in enumerate(np.linspace(0.0, 1.0, n)):
            fn = scene.probe_touch()
            error = preload - fn
            integral = float(np.clip(integral + error * scene.dt, -2.0, 2.0))
            extension = float(
                np.clip(extension + 0.00025 * error + 0.00008 * integral, 0.0, 0.17)
            )
            scene.command(x=start_x + float(alpha) * distance, probe=extension)
            if not run.step(1):
                break
            fn = scene.probe_touch()
            fvec = scene.probe_force_vec() - force_baseline
            ft = float(np.linalg.norm(fvec[:2]))
            if fn < 0.2 * preload:
                lost_count += 1
            else:
                lost_count = 0
            max_lost_count = max(max_lost_count, lost_count)
            actual_path_fraction = float(
                np.clip(
                    abs(float(scene.probe_tip_pos()[0]) - start_tip_x)
                    / max(abs(distance), 1e-9),
                    0.0,
                    1.0,
                )
            )
            target_contact = scene.probe_contact_snapshot(run.target).target_contact
            trace.append((fn, fvec.copy(), actual_path_fraction, target_contact))
            run.sample(
                normal_force_N=fn,
                tangential_force_N=ft,
                commanded_path_fraction=float(alpha),
                path_fraction=actual_path_fraction,
            )
            if fn > force_limit:
                run.fail("force_limit")
                break
            if lost_count > recovery_steps:
                run.fail("lost_contact")
                break
            completed_steps = k + 1

        # Position servos lag the command endpoint.  Keep the final x target and
        # preload controller active for a bounded settling window, and judge the
        # path from the measured tip pose rather than command interpolation.
        for _ in range(200):
            if actual_path_fraction >= 0.95 or run.violations:
                break
            fn = scene.probe_touch()
            error = preload - fn
            integral = float(np.clip(integral + error * scene.dt, -2.0, 2.0))
            extension = float(
                np.clip(
                    extension + 0.00025 * error + 0.00008 * integral,
                    0.0,
                    0.17,
                )
            )
            servo_lead = 0.008 * (1.0 if distance >= 0.0 else -1.0)
            scene.command(x=start_x + distance + servo_lead, probe=extension)
            if not run.step(1):
                break
            endpoint_settle_steps += 1
            fn = scene.probe_touch()
            fvec = scene.probe_force_vec() - force_baseline
            ft = float(np.linalg.norm(fvec[:2]))
            if fn < 0.2 * preload:
                lost_count += 1
            else:
                lost_count = 0
            max_lost_count = max(max_lost_count, lost_count)
            actual_path_fraction = float(
                np.clip(
                    abs(float(scene.probe_tip_pos()[0]) - start_tip_x)
                    / max(abs(distance), 1e-9),
                    0.0,
                    1.0,
                )
            )
            if fn > force_limit:
                run.fail("force_limit")
                break
            if lost_count > recovery_steps:
                run.fail("lost_contact")
                break

    run.enter("post_check")
    completion = actual_path_fraction
    run.quality["path_completion_ratio"] = completion
    run.quality["max_lost_contact_steps"] = float(max_lost_count)
    run.quality["endpoint_settle_steps"] = float(endpoint_settle_steps)
    if completion < 0.95:
        run.fail("path_incomplete")

    if trace:
        fn_arr = np.asarray([item[0] for item in trace])
        force_arr = np.asarray([item[1] for item in trace])
        ft_arr = np.linalg.norm(force_arr[:, :2], axis=1)
        mask = np.asarray([bool(item[3]) for item in trace], dtype=bool) & (
            fn_arr > 1e-4
        )
        if mask.any():
            mu = float(np.median(ft_arr[mask] / (fn_arr[mask] + 1e-6)))
            ft_median = float(np.median(ft_arr[mask]))
            fn_median = float(np.median(fn_arr[mask]))
            vibration = float(np.std(fn_arr[mask]))
            contact_fraction = float(mask.mean())
        else:
            mu = ft_median = fn_median = vibration = contact_fraction = 0.0
    else:
        mu = ft_median = fn_median = vibration = contact_fraction = 0.0
    run.quality["contact_fraction"] = contact_fraction
    if contact_fraction < 0.80:
        run.fail("insufficient_contact_fraction")
    valid = not run.violations

    retreat_ok = _safe_probe_retreat(run)
    valid = valid and retreat_ok and not run.violations
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "mu_est": max(mu, 0.0) if valid else 0.0,
            "friction_ratio": max(mu, 0.0) if valid else 0.0,
            "Ft_median_N": ft_median,
            "Fn_median_N": fn_median,
            "slide_vibration": vibration,
        },
        contact_seconds=len(trace) * scene.dt,
        params={
            "preload": preload,
            "distance": distance,
            "duration": duration,
            "recovery_steps": recovery_steps,
        },
        raw_summary={
            "n_samples": len(trace),
            "force_baseline": force_baseline,
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
    return _DISPATCH[primitive](backend, target, **params)
