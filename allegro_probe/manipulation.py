"""A single audited manipulation path: Allegro short-can pick and place.

This module intentionally does not introduce arm planning or general grasp synthesis.
It consumes one trusted heft result, resets to the documented canonical simulation
checkpoint, and executes an object-specific 16-DoF hand template with contact gates.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from allegro_probe.geometry import (
    RigidTransform,
    quaternion_wxyz_to_matrix,
    rotation_to_xyz_rpy,
)
from allegro_probe.models import ProbeResult
from allegro_probe.scene import AllegroProbeScene, ContactSnapshot


MANIPULATION_SCHEMA_VERSION = "allegro_manip.v1"


def _convert(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, dict):
        return {str(key): _convert(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_convert(item) for item in value]
    return value


@dataclass(frozen=True)
class ShortCanPickPlaceRequest:
    """Task intent supplied by the layer that already selected the target."""

    target: int
    place_offset_xy_m: Tuple[float, float] = (0.0, 0.12)
    reset_before_execute: bool = True


@dataclass(frozen=True)
class ManipulationContext:
    schema_version: str
    scene_id: str
    backend: str
    family: str
    target: int
    object_id: str
    shape: str
    collision_size_m: Tuple[float, float, float]
    probe_primitive: str
    mass_estimate_kg: float
    weight_signal_N: float
    target_total_normal_force_N: float
    max_wrist_speed_mps: float
    probe_quality: Dict[str, float]
    handoff_policy: str
    pose_source: str = "canonical_scene"
    source_object_pose_world: Optional[RigidTransform] = None
    fixed_place_object_pose_world: Optional[RigidTransform] = None
    fixed_goal_id: Optional[str] = None
    selected_grasp_candidate_id: Optional[str] = None


@dataclass(frozen=True)
class AllegroHandTemplate:
    """Object-specific position template; arrays follow actuator order in scene.py."""

    name: str
    q_open: np.ndarray
    q_preshape: np.ndarray
    q_contact: np.ndarray
    q_squeeze_limit: np.ndarray
    active_fingers: Tuple[str, ...]
    required_contact_groups: Tuple[str, ...]
    required_object_geom_suffix: str
    wrist_y_offset_m: float
    wrist_to_object_center_z_m: float
    use_contact_waypoint: bool = False

    def pose(self, progress: float) -> np.ndarray:
        alpha = float(np.clip(progress, 0.0, 1.0))
        if not self.use_contact_waypoint:
            return (1.0 - alpha) * self.q_preshape + alpha * self.q_squeeze_limit
        contact_progress = 0.80
        if alpha <= contact_progress:
            local = alpha / contact_progress
            return (1.0 - local) * self.q_preshape + local * self.q_contact
        local = (alpha - contact_progress) / (1.0 - contact_progress)
        return (1.0 - local) * self.q_contact + local * self.q_squeeze_limit


@dataclass(frozen=True)
class ManipulationPlan:
    schema_version: str
    skill: str
    target: int
    object_id: str
    template: AllegroHandTemplate
    place_offset_xy_m: Tuple[float, float]
    reset_before_execute: bool
    lift_height_m: float
    min_carry_distance_m: float
    hold_time_s: float
    target_total_normal_force_N: float
    max_total_normal_force_N: float
    max_place_normal_force_N: float
    hand_table_release_guard_N: float
    max_hand_table_force_N: float
    max_wrist_speed_mps: float
    max_penetration_m: float
    max_place_penetration_m: float
    max_release_height_m: float
    place_surface_z_m: float
    post_descent_xy_correction: bool
    use_gravity_settle: bool
    max_place_error_m: float
    max_final_tilt_rad: float
    max_final_drift_m: float
    phases: Tuple[str, ...]
    handoff_policy: str = "reset_to_canonical_checkpoint"
    pose_source: str = "canonical_scene"
    source_object_pose_world: Optional[RigidTransform] = None
    fixed_place_object_pose_world: Optional[RigidTransform] = None
    fixed_goal_id: Optional[str] = None
    staging_wrist_pose_world: Optional[RigidTransform] = None
    pregrasp_wrist_pose_world: Optional[RigidTransform] = None
    grasp_wrist_pose_world: Optional[RigidTransform] = None
    lift_wrist_pose_world: Optional[RigidTransform] = None
    carry_wrist_pose_world: Optional[RigidTransform] = None
    selected_grasp_candidate_id: Optional[str] = None
    selected_symmetry_yaw_rad: float = 0.0
    source_position_tolerance_m: float = 0.008
    source_axis_tolerance_rad: float = 0.12
    require_precontact_clearance: bool = False
    forbid_palm_contact: bool = False
    forbid_other_object_contact: bool = False
    min_contact_force_per_group_N: float = 0.0
    forbid_inactive_finger_contact: bool = False
    allowed_hand_contact_tokens: Tuple[str, ...] = ()

    def to_dict(self) -> Dict[str, Any]:
        return _convert(asdict(self))


@dataclass(frozen=True)
class ManipulationPlanDecision:
    executable: bool
    reason: str
    context: Optional[ManipulationContext] = None
    plan: Optional[ManipulationPlan] = None

    def to_dict(self) -> Dict[str, Any]:
        return _convert(asdict(self))


@dataclass
class ManipulationExecutionResult:
    object_id: str
    target: int
    skill: str
    status: str
    success: bool
    backend: str
    phase_reached: str
    violations: List[str] = field(default_factory=list)
    quality: Dict[str, float] = field(default_factory=dict)
    params: Dict[str, Any] = field(default_factory=dict)
    trace: Dict[str, List[Any]] = field(default_factory=dict)

    def to_dict(self, include_trace: bool = False) -> Dict[str, Any]:
        payload = asdict(self)
        if not include_trace:
            payload.pop("trace", None)
        return _convert(payload)


def short_can_hand_template(scene: AllegroProbeScene) -> AllegroHandTemplate:
    """Return the first explicit 16-DoF hand template in the project."""

    q_open = scene.allegro_grip_pose(0.0)
    q_preshape = scene.allegro_grip_pose(0.10)
    q_contact = scene.allegro_grip_pose(0.78)
    q_squeeze = scene.allegro_grip_pose(0.94)
    return AllegroHandTemplate(
        name="short_can_side_wrap_v1",
        q_open=q_open,
        q_preshape=q_preshape,
        q_contact=q_contact,
        q_squeeze_limit=q_squeeze,
        active_fingers=("mf", "th"),
        required_contact_groups=("mf", "th"),
        required_object_geom_suffix="_geom",
        wrist_y_offset_m=0.020,
        # Lower than the heft pose so the active links load the cylindrical waist
        # instead of obtaining a purely lip-assisted grasp.
        wrist_to_object_center_z_m=0.414,
    )


def build_short_can_pick_place_plan(
    scene: AllegroProbeScene,
    probe_result: ProbeResult,
    request: ShortCanPickPlaceRequest,
    *,
    _require_legacy_scene: bool = True,
) -> ManipulationPlanDecision:
    """Validate the handoff and deterministically build one Allegro plan."""

    def reject(reason: str) -> ManipulationPlanDecision:
        return ManipulationPlanDecision(executable=False, reason=reason)

    if scene.config.backend != "allegro":
        return reject("allegro_backend_required")
    if _require_legacy_scene:
        if scene.config.full_hand_collisions or scene.full_hand_collisions_compiled():
            return reject("legacy_distal_only_collision_required")
        if any(f"obj{index}_pedestal" not in scene.geom for index in range(scene.n)):
            return reject("legacy_elevated_fixture_required")
    if not request.reset_before_execute:
        return reject("canonical_reset_required")
    expected_place_offset = (0.0, scene.config.short_can_place_y)
    if not np.allclose(request.place_offset_xy_m, expected_place_offset, atol=1e-9):
        return reject("fixed_place_offset_required")
    if scene.task.family != "mass":
        return reject("mass_scene_required")
    if request.target < 0 or request.target >= scene.n:
        return reject("target_out_of_range")
    obj = scene.task.objects[request.target]
    if obj.shape != "short_can":
        return reject("short_can_required")
    if probe_result.target != request.target or probe_result.object_id != obj.object_id:
        return reject("probe_target_mismatch")
    if probe_result.backend != "allegro":
        return reject("allegro_probe_required")
    if probe_result.primitive != "heft":
        return reject("heft_probe_required")
    if not probe_result.valid or probe_result.violations:
        return reject("probe_invalid")

    mass_estimate = float(probe_result.features.get("m_est_kg", 0.0))
    weight_signal = float(probe_result.features.get("weight_signal_N", 0.0))
    if not np.isfinite(mass_estimate) or mass_estimate <= 0.0:
        return reject("mass_estimate_missing")
    if not np.isfinite(weight_signal) or weight_signal <= 0.0:
        return reject("weight_signal_missing")

    # Force semantics are explicit: this is the sum of normal-force magnitudes over
    # all legal hand/object contacts, matching ContactSnapshot.hand_normal_force_N.
    # The full-collision top-pinch heft path reports an approximately physical
    # gravity load.  v1's former 1.6 N cutoff was calibrated to the biased
    # under-wrap probe and misclassified a 0.24 kg can as the heavy release path.
    lightweight_release = mass_estimate < 0.27
    target_force = (
        (6.8 if mass_estimate < 0.18 else 7.6)
        if lightweight_release
        else float(np.clip(8.0 + 0.45 * weight_signal, 8.0, 11.0))
    )
    max_speed = (
        0.070
        if lightweight_release
        else float(
            np.clip(0.075 / (1.0 + 0.65 * mass_estimate), 0.040, 0.070)
        )
    )
    context = ManipulationContext(
        schema_version=MANIPULATION_SCHEMA_VERSION,
        scene_id=scene.task.scene_id,
        backend="allegro",
        family="mass",
        target=request.target,
        object_id=obj.object_id,
        shape=obj.shape,
        collision_size_m=tuple(float(v) for v in obj.size),
        probe_primitive="heft",
        mass_estimate_kg=mass_estimate,
        weight_signal_N=weight_signal,
        target_total_normal_force_N=target_force,
        max_wrist_speed_mps=max_speed,
        probe_quality=dict(probe_result.quality),
        handoff_policy=(
            "reset_to_canonical_checkpoint"
            if request.reset_before_execute
            else "continue_from_live_state"
        ),
    )
    plan = ManipulationPlan(
        schema_version=MANIPULATION_SCHEMA_VERSION,
        skill="short_can_pick_place",
        target=request.target,
        object_id=obj.object_id,
        template=short_can_hand_template(scene),
        place_offset_xy_m=tuple(float(v) for v in request.place_offset_xy_m),
        reset_before_execute=bool(request.reset_before_execute),
        lift_height_m=0.035,
        min_carry_distance_m=0.080,
        hold_time_s=0.50,
        target_total_normal_force_N=target_force,
        max_total_normal_force_N=20.0,
        max_place_normal_force_N=30.0,
        hand_table_release_guard_N=20.0 if lightweight_release else 30.0,
        max_hand_table_force_N=40.0,
        max_wrist_speed_mps=max_speed,
        max_penetration_m=0.0052,
        max_place_penetration_m=0.0055,
        max_release_height_m=0.025 if lightweight_release else 0.010,
        place_surface_z_m=0.0,
        post_descent_xy_correction=not lightweight_release,
        use_gravity_settle=lightweight_release,
        # A 35 mm circular region is slightly larger than the can radius and
        # measures task-level placement, not millimetre-accurate arm positioning.
        max_place_error_m=0.035,
        max_final_tilt_rad=0.20,
        max_final_drift_m=0.005,
        phases=(
            "handoff",
            "preshape",
            "approach",
            "contact_acquire",
            "grip_regulate",
            "lift",
            "carry",
            "place_descent",
            "settle_to_surface",
            "release",
            "retreat",
            "final_verify",
        ),
    )
    return ManipulationPlanDecision(
        executable=True,
        reason="ok",
        context=context,
        plan=plan,
    )


@dataclass
class _Execution:
    scene: AllegroProbeScene
    plan: ManipulationPlan
    phase: str = "init"
    violations: List[str] = field(default_factory=list)
    quality: Dict[str, float] = field(default_factory=dict)
    trace: Dict[str, List[Any]] = field(default_factory=dict)
    closure_progress: float = 0.0
    lost_contact_steps: int = 0
    sample_index: int = 0
    max_penetration_m: float = 0.0
    max_grasp_carry_penetration_m: float = 0.0
    max_place_release_penetration_m: float = 0.0
    peak_hand_force_N: float = 0.0
    peak_grasp_carry_force_N: float = 0.0
    peak_place_release_force_N: float = 0.0
    peak_hand_table_force_N: float = 0.0
    peak_palm_object_force_N: float = 0.0
    peak_hand_other_object_force_N: float = 0.0

    def enter(self, phase: str) -> None:
        self.phase = phase
        self.trace.setdefault("phase", []).append(phase)

    def fail(self, violation: str) -> None:
        if violation not in self.violations:
            self.violations.append(violation)

    def observe(self) -> ContactSnapshot:
        snapshot = self.scene.contact_snapshot(self.plan.target)
        self.max_penetration_m = max(
            self.max_penetration_m, snapshot.hand_max_penetration_m
        )
        if self.phase in {"place_descent", "release"}:
            self.max_place_release_penetration_m = max(
                self.max_place_release_penetration_m,
                snapshot.hand_max_penetration_m,
            )
        else:
            self.max_grasp_carry_penetration_m = max(
                self.max_grasp_carry_penetration_m,
                snapshot.hand_max_penetration_m,
            )
        self.peak_hand_force_N = max(
            self.peak_hand_force_N, snapshot.hand_normal_force_N
        )
        if self.phase in {"place_descent", "release"}:
            self.peak_place_release_force_N = max(
                self.peak_place_release_force_N,
                snapshot.hand_normal_force_N,
            )
        else:
            self.peak_grasp_carry_force_N = max(
                self.peak_grasp_carry_force_N,
                snapshot.hand_normal_force_N,
            )
        self.peak_hand_table_force_N = max(
            self.peak_hand_table_force_N,
            snapshot.hand_table_normal_force_N,
        )
        self.peak_palm_object_force_N = max(
            self.peak_palm_object_force_N,
            snapshot.palm_object_normal_force_N,
        )
        self.peak_hand_other_object_force_N = max(
            self.peak_hand_other_object_force_N,
            snapshot.hand_other_object_normal_force_N,
        )
        if self.sample_index % 10 == 0:
            self.trace.setdefault("sample_phase", []).append(self.phase)
            object_position = (
                self.scene.object_center_pos(self.plan.target)
                if self.plan.source_object_pose_world is not None
                else self.scene.object_pos(self.plan.target)
            )
            self.trace.setdefault("object_pos_m", []).append(
                object_position.copy()
            )
            self.trace.setdefault("wrist_pos_m", []).append(
                self.scene.wrist_pos().copy()
            )
            self.trace.setdefault("hand_groups", []).append(
                list(snapshot.hand_groups)
            )
            self.trace.setdefault("hand_object_geoms", []).append(
                list(snapshot.hand_object_geoms)
            )
            self.trace.setdefault("hand_force_N", []).append(
                snapshot.hand_normal_force_N
            )
            self.trace.setdefault("penetration_m", []).append(
                snapshot.hand_max_penetration_m
            )
            self.trace.setdefault("source_support_contact", []).append(
                snapshot.support_contact
            )
            self.trace.setdefault("hand_table_contact", []).append(
                snapshot.hand_table_contact
            )
            self.trace.setdefault("hand_table_force_N", []).append(
                snapshot.hand_table_normal_force_N
            )
            self.trace.setdefault("hand_support_contact", []).append(
                snapshot.hand_support_contact
            )
        self.sample_index += 1
        return snapshot


def _wrist_controls(scene: AllegroProbeScene) -> Dict[str, float]:
    names = {
        "x": "act_wx",
        "y": "act_wy",
        "z": "act_wz",
        "roll": "act_wr",
        "tilt": "act_wt",
        "yaw": "act_wyaw",
    }
    return {key: float(scene.data.ctrl[scene.act[name]]) for key, name in names.items()}


def _smoothstep(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def _wrist_goal_for_pose(
    scene: AllegroProbeScene, pose_world: RigidTransform
) -> Dict[str, float]:
    """Convert a physical world<-wrist pose to carriage position controls."""

    if pose_world.parent_frame != "world" or pose_world.child_frame != "wrist":
        raise ValueError(
            "wrist pose must use frames world<-wrist, got "
            f"{pose_world.parent_frame}<-{pose_world.child_frame}"
        )
    roll, tilt, yaw = rotation_to_xyz_rpy(pose_world.rotation)
    x, y, z = pose_world.translation_m
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z - scene.config.palm_height),
        "roll": float(roll),
        "tilt": float(tilt),
        "yaw": float(yaw),
    }


def _source_pose_error(
    scene: AllegroProbeScene, plan: ManipulationPlan
) -> Tuple[float, float]:
    if plan.source_object_pose_world is None:
        return 0.0, 0.0
    expected = plan.source_object_pose_world
    position_error = float(
        np.linalg.norm(
            scene.object_center_pos(plan.target)
            - np.asarray(expected.translation_m, dtype=float)
        )
    )
    measured_axis = quaternion_wxyz_to_matrix(scene.object_quat(plan.target))[:, 2]
    expected_axis = expected.axis_z_parent
    axis_error = float(
        np.arccos(np.clip(float(np.dot(measured_axis, expected_axis)), -1.0, 1.0))
    )
    return position_error, axis_error


def _move_wrist(
    run: _Execution,
    *,
    goal: Dict[str, float],
    min_steps: int = 80,
    guard_grasp: bool = False,
    allow_table_contact: bool = False,
    max_force_N: Optional[float] = None,
    max_penetration_m: Optional[float] = None,
    guard_environment: bool = False,
    forbid_object_contact: bool = False,
) -> bool:
    scene = run.scene
    start = _wrist_controls(scene)
    distance = float(
        np.linalg.norm(
            np.asarray([goal.get(k, start[k]) - start[k] for k in ("x", "y", "z")])
        )
    )
    duration = distance / max(run.plan.max_wrist_speed_mps, 1e-4)
    steps = max(int(np.ceil(duration / scene.dt)), int(min_steps))
    for index in range(steps):
        alpha = _smoothstep((index + 1) / steps)
        command = {
            key: start[key] + alpha * (float(target) - start[key])
            for key, target in goal.items()
        }
        scene.command(**command)
        scene.step(1)
        snapshot = run.observe()
        penetration_limit = (
            run.plan.max_penetration_m
            if max_penetration_m is None
            else float(max_penetration_m)
        )
        if snapshot.hand_max_penetration_m > penetration_limit:
            run.fail("penetration_limit")
            return False
        if run.plan.forbid_palm_contact and snapshot.palm_object_contact:
            run.fail("palm_object_collision")
            return False
        if forbid_object_contact and (
            snapshot.hand_contact_geoms or snapshot.palm_object_contact
        ):
            run.fail("unexpected_contact_during_approach")
            return False
        if guard_environment and (
            snapshot.hand_table_contact
            or snapshot.hand_support_contact
            or snapshot.hand_other_object_contact
            or snapshot.object_other_object_contact
        ):
            run.fail("hand_environment_collision_during_approach")
            return False
        if guard_grasp and not _regulate_grasp(
            run,
            snapshot,
            allow_table_contact=allow_table_contact,
            max_force_N=max_force_N,
            max_penetration_m=max_penetration_m,
        ):
            return False
    return True


def _move_hand(
    run: _Execution,
    goal: np.ndarray,
    steps: int,
    *,
    guard_clearance: bool = False,
) -> bool:
    scene = run.scene
    start = scene.allegro_joint_targets()
    for index in range(max(int(steps), 1)):
        alpha = _smoothstep((index + 1) / max(int(steps), 1))
        scene.command_allegro_joints((1.0 - alpha) * start + alpha * goal)
        scene.step(1)
        snapshot = run.observe()
        if guard_clearance and (
            snapshot.hand_contact_geoms
            or snapshot.palm_object_contact
            or snapshot.hand_table_contact
            or snapshot.hand_support_contact
            or snapshot.hand_other_object_contact
            or snapshot.object_other_object_contact
        ):
            run.fail("unsafe_contact_during_preshape")
            return False
    return True


def _release_until_clear(run: _Execution) -> bool:
    """Use a low-stiffness symmetric opening until hand contact clears."""

    current = run.scene.allegro_joint_targets()
    release_groups = (tuple(range(16)),)
    run.scene.set_allegro_position_kp(0.1)
    try:
        for indices in release_groups:
            goal = current.copy()
            goal[list(indices)] = run.plan.template.q_open[list(indices)]
            clear_steps = 0
            for index in range(600):
                alpha = _smoothstep((index + 1) / 600)
                run.scene.command_allegro_joints(
                    (1.0 - alpha) * current + alpha * goal
                )
                run.scene.step(1)
                snapshot = run.observe()
                if (
                    snapshot.hand_max_penetration_m
                    > run.plan.max_place_penetration_m
                ):
                    run.fail("penetration_limit")
                    return False
                if snapshot.hand_normal_force_N > run.plan.max_place_normal_force_N:
                    run.fail("hand_force_limit")
                    return False
                if run.plan.skill == "pose_conditioned_short_can_pick_place":
                    if snapshot.palm_object_contact:
                        run.fail("palm_object_collision")
                        return False
                    if snapshot.hand_table_contact:
                        run.fail("hand_table_collision")
                        return False
                    if (
                        snapshot.hand_other_object_contact
                        or snapshot.object_other_object_contact
                    ):
                        run.fail("other_object_collision")
                        return False
                if run.plan.skill == "pose_conditioned_short_can_pick_place":
                    # A top-entry hand retreats vertically past the object.  Do not
                    # accept the v1 "no opposing pair" shortcut: even one residual
                    # fingertip contact can drag a surface-supported can away from
                    # the absolute goal during retreat.
                    released = (
                        not snapshot.hand_groups
                        and not snapshot.palm_object_contact
                    )
                else:
                    released = not snapshot.hand_groups or (
                        snapshot.table_contact
                        and not _opposing_contacts(run, snapshot)
                        and snapshot.hand_normal_force_N <= 1.5
                    )
                if released:
                    clear_steps += 1
                else:
                    clear_steps = 0
                if clear_steps >= 12:
                    run.closure_progress = -1.0
                    return True
            current = goal
    finally:
        run.scene.set_allegro_position_kp(8.0)
    run.fail("release_incomplete")
    return False


def _settle_object_to_table(run: _Execution, steps: int = 800) -> bool:
    """Let the can slide down inside a low-stiffness finger cage."""

    run.enter("settle_to_surface")
    scene = run.scene
    start = scene.allegro_joint_targets()
    scene.set_allegro_position_kp(0.35)
    table_steps = 0
    unsupported_clear_steps = 0
    for index in range(max(int(steps), 1)):
        alpha = _smoothstep((index + 1) / max(int(steps), 1))
        scene.command_allegro_joints(
            (1.0 - alpha) * start + alpha * run.plan.template.q_open
        )
        scene.step(1)
        snapshot = run.observe()
        if snapshot.hand_max_penetration_m > run.plan.max_place_penetration_m:
            run.fail("penetration_limit")
            return False
        if snapshot.hand_table_normal_force_N > run.plan.max_hand_table_force_N:
            run.fail("hand_table_force_limit")
            return False
        if run.plan.skill == "pose_conditioned_short_can_pick_place" and (
            snapshot.hand_table_contact
            or snapshot.hand_other_object_contact
            or snapshot.object_other_object_contact
            or snapshot.palm_object_contact
        ):
            run.fail("environment_collision_during_settle")
            return False
        if snapshot.table_contact:
            table_steps += 1
        else:
            table_steps = 0
        if not snapshot.hand_groups and not snapshot.table_contact:
            unsupported_clear_steps += 1
        else:
            unsupported_clear_steps = 0
        if unsupported_clear_steps > 100:
            run.fail("dropped_before_surface")
            return False
        if table_steps >= 12:
            scene.command(z=float(scene.sensor("wz_pos")[0]) + 0.002)
            scene.step(20)
            run.observe()
            return True
    run.fail("surface_not_reached_during_settle")
    return False


def _required_contacts(run: _Execution, snapshot: ContactSnapshot) -> bool:
    groups_ok = _opposing_contacts(run, snapshot)
    if run.plan.forbid_inactive_finger_contact:
        groups_ok = groups_ok and set(snapshot.hand_groups).issubset(
            set(run.plan.template.active_fingers)
        )
    waist_name = f"obj{run.plan.target}{run.plan.template.required_object_geom_suffix}"
    waist_ok = waist_name in set(snapshot.hand_object_geoms)
    links_ok = True
    if run.plan.allowed_hand_contact_tokens:
        links_ok = all(
            any(token in geom for token in run.plan.allowed_hand_contact_tokens)
            for geom in snapshot.hand_contact_geoms
        )
    palm_ok = not run.plan.forbid_palm_contact or not snapshot.palm_object_contact
    return groups_ok and waist_ok and links_ok and palm_ok


def _opposing_contacts(run: _Execution, snapshot: ContactSnapshot) -> bool:
    required = set(run.plan.template.required_contact_groups)
    forces = dict(snapshot.hand_force_by_group_N)
    return required.issubset(set(snapshot.hand_groups)) and all(
        float(forces.get(group, 0.0))
        >= run.plan.min_contact_force_per_group_N
        for group in required
    )


def _regulate_grasp(
    run: _Execution,
    snapshot: ContactSnapshot,
    *,
    allow_table_contact: bool,
    max_force_N: Optional[float] = None,
    max_penetration_m: Optional[float] = None,
) -> bool:
    if run.plan.forbid_palm_contact and snapshot.palm_object_contact:
        run.fail("palm_object_collision")
        return False
    if run.plan.forbid_other_object_contact and (
        snapshot.hand_other_object_contact
        or snapshot.object_other_object_contact
    ):
        run.fail("other_object_collision")
        return False
    if run.plan.forbid_inactive_finger_contact and (
        set(snapshot.hand_groups) - set(run.plan.template.active_fingers)
    ):
        run.fail("inactive_finger_contact")
        return False
    # Once the object is supported by the destination surface, a light incidental
    # distal-link contact may appear while the can settles/tilts into place.  It is
    # no longer part of grasp admission; palm, penetration, force, and hand-table
    # guards remain active.  Before support, the selected grasp-link whitelist is
    # enforced strictly throughout lift and carry.
    enforce_link_whitelist = not (allow_table_contact and snapshot.table_contact)
    if enforce_link_whitelist and run.plan.allowed_hand_contact_tokens and any(
        not any(token in geom for token in run.plan.allowed_hand_contact_tokens)
        for geom in snapshot.hand_contact_geoms
    ):
        run.fail("forbidden_hand_link_contact")
        return False
    penetration_limit = (
        run.plan.max_penetration_m
        if max_penetration_m is None
        else float(max_penetration_m)
    )
    if snapshot.hand_max_penetration_m > penetration_limit:
        run.fail("penetration_limit")
        return False
    force_limit = (
        run.plan.max_total_normal_force_N
        if max_force_N is None
        else float(max_force_N)
    )
    if snapshot.hand_normal_force_N > force_limit:
        run.closure_progress = max(run.closure_progress - 0.003, 0.0)
        run.scene.command_allegro_joints(run.plan.template.pose(run.closure_progress))
        run.fail("hand_force_limit")
        return False
    if not _opposing_contacts(run, snapshot):
        run.lost_contact_steps += 1
    else:
        run.lost_contact_steps = 0
    if run.lost_contact_steps > 80:
        run.fail("lost_legal_grasp")
        return False
    light_top_pinch = (
        run.plan.skill == "pose_conditioned_short_can_pick_place"
        and run.plan.target_total_normal_force_N <= 8.2
    )
    if light_top_pinch:
        # A very light can reacts quickly to closure and otherwise oscillates
        # between microslip and a force spike.  Use a narrow, gentle schedule for
        # this calibrated force band; heavier cans need the legacy tracking rate.
        if snapshot.hand_normal_force_N < 0.85 * run.plan.target_total_normal_force_N:
            run.closure_progress = min(run.closure_progress + 0.0004, 1.0)
            run.scene.command_allegro_joints(
                run.plan.template.pose(run.closure_progress)
            )
        elif snapshot.hand_normal_force_N > 1.10 * run.plan.target_total_normal_force_N:
            run.closure_progress = max(run.closure_progress - 0.0015, 0.0)
            run.scene.command_allegro_joints(
                run.plan.template.pose(run.closure_progress)
            )
    elif snapshot.hand_normal_force_N < 0.70 * run.plan.target_total_normal_force_N:
        run.closure_progress = min(run.closure_progress + 0.0015, 1.0)
        run.scene.command_allegro_joints(run.plan.template.pose(run.closure_progress))
    elif snapshot.hand_normal_force_N > 1.35 * run.plan.target_total_normal_force_N:
        run.closure_progress = max(run.closure_progress - 0.0010, 0.0)
        run.scene.command_allegro_joints(run.plan.template.pose(run.closure_progress))
    if snapshot.support_contact:
        run.fail("pedestal_contact_after_lift")
        return False
    if snapshot.hand_support_contact:
        run.fail("hand_support_collision")
        return False
    if snapshot.hand_table_normal_force_N > run.plan.max_hand_table_force_N:
        run.fail("hand_table_force_limit")
        return False
    if (
        run.plan.skill == "pose_conditioned_short_can_pick_place"
        and snapshot.hand_table_contact
    ):
        run.fail("hand_table_collision")
        return False
    if snapshot.hand_table_contact and not allow_table_contact:
        run.fail("hand_table_collision")
        return False
    if snapshot.table_contact and not allow_table_contact:
        run.fail("table_contact_during_carry")
        return False
    return True


def _object_tilt_rad(quaternion_wxyz: np.ndarray) -> float:
    q = np.asarray(quaternion_wxyz, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        return float("inf")
    w, x, y, z = q / norm
    del w, z
    local_z_world_z = float(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0))
    return float(np.arccos(local_z_world_z))


def _object_bottom_z(scene: AllegroProbeScene, target: int) -> float:
    """Lowest point of the short cylinder in world z, including object tilt."""

    obj = scene.task.objects[target]
    axis = quaternion_wxyz_to_matrix(scene.object_quat(target))[:, 2]
    radial_z = float(np.linalg.norm(axis[:2]))
    return float(
        scene.object_center_pos(target)[2]
        - obj.size[2] * abs(float(axis[2]))
        - max(obj.size[0], obj.size[1]) * radial_z
    )


def _result(run: _Execution, *, success: bool) -> ManipulationExecutionResult:
    # All guarded gain schedules are phase-local, including failure exits.
    run.scene.set_allegro_position_kp(8.0)
    run.quality["max_penetration_m"] = run.max_penetration_m
    run.quality["max_grasp_carry_penetration_m"] = (
        run.max_grasp_carry_penetration_m
    )
    run.quality["max_place_release_penetration_m"] = (
        run.max_place_release_penetration_m
    )
    run.quality["peak_hand_normal_force_N"] = run.peak_hand_force_N
    run.quality["peak_grasp_carry_force_N"] = run.peak_grasp_carry_force_N
    run.quality["peak_place_release_force_N"] = run.peak_place_release_force_N
    run.quality["peak_hand_table_force_N"] = run.peak_hand_table_force_N
    run.quality["peak_palm_object_force_N"] = run.peak_palm_object_force_N
    run.quality["peak_hand_other_object_force_N"] = (
        run.peak_hand_other_object_force_N
    )
    status = "ok" if success else (run.violations[0] if run.violations else "failed")
    return ManipulationExecutionResult(
        object_id=run.plan.object_id,
        target=run.plan.target,
        skill=run.plan.skill,
        status=status,
        success=bool(success),
        backend="allegro",
        phase_reached=run.phase,
        violations=list(run.violations),
        quality=dict(run.quality),
        params={
            "schema_version": run.plan.schema_version,
            "template": run.plan.template.name,
            "target_total_normal_force_N": run.plan.target_total_normal_force_N,
            "max_place_normal_force_N": run.plan.max_place_normal_force_N,
            "hand_table_release_guard_N": run.plan.hand_table_release_guard_N,
            "max_hand_table_force_N": run.plan.max_hand_table_force_N,
            "max_wrist_speed_mps": run.plan.max_wrist_speed_mps,
            "release_policy": (
                "low_kp_until_all_hand_contacts_clear"
                if run.plan.skill == "pose_conditioned_short_can_pick_place"
                else "low_kp_symmetric_until_clear"
            ),
            "collision_mode": (
                "full_hand"
                if run.scene.config.full_hand_collisions
                else "legacy_distal_only"
            ),
            "post_descent_xy_correction": run.plan.post_descent_xy_correction,
            "use_gravity_settle": run.plan.use_gravity_settle,
            "handoff_policy": run.plan.handoff_policy,
            "pose_source": run.plan.pose_source,
            "selected_grasp_candidate_id": run.plan.selected_grasp_candidate_id,
            "fixed_goal_id": run.plan.fixed_goal_id,
            "selected_symmetry_yaw_rad": run.plan.selected_symmetry_yaw_rad,
            "source_object_pose_world": (
                run.plan.source_object_pose_world.to_dict()
                if run.plan.source_object_pose_world is not None
                else None
            ),
            "fixed_place_object_pose_world": (
                run.plan.fixed_place_object_pose_world.to_dict()
                if run.plan.fixed_place_object_pose_world is not None
                else None
            ),
        },
        trace=run.trace,
    )


def _pose_plan_execution_error(
    scene: AllegroProbeScene, plan: ManipulationPlan
) -> Optional[str]:
    """Revalidate all safety-critical v2 facts against the compiled scene."""

    if plan.schema_version != "allegro_manip.pose.v2":
        return "pose_schema_required"
    if plan.skill != "pose_conditioned_short_can_pick_place":
        return "pose_skill_required"
    if scene.config.backend != "allegro":
        return "allegro_backend_required"
    if not scene.config.full_hand_collisions:
        return "full_hand_collisions_required"
    if not scene.full_hand_collisions_compiled():
        return "compiled_collision_model_insufficient"
    if any(f"obj{index}_pedestal" in scene.geom for index in range(scene.n)):
        return "support_free_table_scene_required"
    if plan.target < 0 or plan.target >= scene.n:
        return "target_out_of_range"
    obj = scene.task.objects[plan.target]
    if plan.object_id != obj.object_id or obj.shape != "short_can":
        return "plan_object_mismatch"
    if plan.source_object_pose_world is None:
        return "source_object_pose_required"
    if (
        plan.source_object_pose_world.parent_frame != "world"
        or plan.source_object_pose_world.child_frame != plan.object_id
    ):
        return "source_object_frame_mismatch"
    if plan.fixed_place_object_pose_world is None or not plan.fixed_goal_id:
        return "fixed_goal_required"
    if (
        plan.fixed_place_object_pose_world.parent_frame != "world"
        or plan.fixed_place_object_pose_world.child_frame != plan.object_id
    ):
        return "fixed_goal_frame_mismatch"
    table_gid = scene.geom.get("table")
    if table_gid is None:
        return "table_required"
    table_center = np.asarray(scene.data.geom_xpos[table_gid], dtype=float)
    table_half_size = np.asarray(scene.model.geom_size[table_gid], dtype=float)
    table_top = float(table_center[2] + table_half_size[2])
    source_position = np.asarray(
        plan.source_object_pose_world.translation_m, dtype=float
    )
    source_axis = plan.source_object_pose_world.axis_z_parent
    source_bottom = float(
        source_position[2]
        - obj.size[2] * abs(float(source_axis[2]))
        - max(obj.size[0], obj.size[1])
        * float(np.linalg.norm(source_axis[:2]))
    )
    goal_position = np.asarray(
        plan.fixed_place_object_pose_world.translation_m, dtype=float
    )
    goal_axis = plan.fixed_place_object_pose_world.axis_z_parent
    goal_axis_error = float(
        np.arccos(
            np.clip(float(np.dot(goal_axis, (0.0, 0.0, 1.0))), -1.0, 1.0)
        )
    )
    planar_clearance = max(obj.size[0], obj.size[1]) + 0.005
    if (
        abs(plan.place_surface_z_m - table_top) > 0.003
        or abs(source_bottom - table_top) > 0.010
        or abs(goal_position[2] - (table_top + obj.size[2])) > 0.010
        or goal_axis_error > 0.02
        or np.any(
            np.abs(source_position[:2] - table_center[:2]) + planar_clearance
            > table_half_size[:2]
        )
        or np.any(
            np.abs(goal_position[:2] - table_center[:2]) + planar_clearance
            > table_half_size[:2]
        )
    ):
        return "pose_workspace_policy_mismatch"
    for index, other in enumerate(scene.task.objects):
        if index == plan.target:
            continue
        other_position = scene.object_center_pos(index)
        radii = (
            max(obj.size[0], obj.size[1])
            + max(other.size[0], other.size[1])
        )
        if float(np.linalg.norm(source_position[:2] - other_position[:2])) < (
            radii + 0.080
        ):
            return "source_obstacle_clearance_required"
        if float(np.linalg.norm(goal_position[:2] - other_position[:2])) < (
            radii + 0.045
        ):
            return "fixed_goal_obstacle_clearance_required"
    if not (
        plan.require_precontact_clearance
        and plan.forbid_palm_contact
        and plan.forbid_other_object_contact
        and plan.forbid_inactive_finger_contact
        and plan.min_contact_force_per_group_N >= 0.20
    ):
        return "pose_contact_guards_required"
    expected_tokens = {
        "fingertip_collision",
        "thumbtip_collision",
        "distal_collision",
    }
    if (
        plan.template.name != "short_can_top_pinch_v1"
        or plan.template.active_fingers != ("mf", "th")
        or plan.template.required_contact_groups != ("mf", "th")
        or plan.template.required_object_geom_suffix != "_top_lip"
        or not plan.template.use_contact_waypoint
        or set(plan.allowed_hand_contact_tokens) != expected_tokens
    ):
        return "uncalibrated_grasp_template"
    expected_hand_poses = (
        scene.allegro_grip_pose(0.0),
        scene.allegro_grip_pose(0.10),
        scene.allegro_grip_pose(0.80),
        scene.allegro_grip_pose(0.98),
    )
    actual_hand_poses = (
        plan.template.q_open,
        plan.template.q_preshape,
        plan.template.q_contact,
        plan.template.q_squeeze_limit,
    )
    if any(
        not np.allclose(actual, expected, atol=1e-9)
        for actual, expected in zip(actual_hand_poses, expected_hand_poses)
    ):
        return "uncalibrated_hand_pose"
    if not (
        7.5 <= plan.target_total_normal_force_N <= 12.0
        and 0.0 < plan.max_total_normal_force_N <= 20.0
        and 0.0 < plan.max_place_normal_force_N <= 30.0
        and 0.0 < plan.max_penetration_m <= 0.0068
        and 0.0 < plan.max_place_penetration_m <= 0.0068
        and 0.0 < plan.max_wrist_speed_mps <= 0.110
        and 0.0 < plan.source_position_tolerance_m <= 0.008
        and 0.0 < plan.source_axis_tolerance_rad <= 0.12
        and 0.0 < plan.max_place_error_m <= 0.035
        and 0.0 < plan.max_final_tilt_rad <= 0.20
        and 0.0 < plan.max_final_drift_m <= 0.005
        and np.isclose(plan.lift_height_m, 0.130, atol=1e-9)
        and np.isclose(plan.max_release_height_m, 0.008, atol=1e-9)
        and np.isclose(plan.max_hand_table_force_N, 0.0, atol=1e-12)
        and np.isclose(plan.hand_table_release_guard_N, 0.0, atol=1e-12)
        and not plan.use_gravity_settle
        and plan.post_descent_xy_correction
    ):
        return "uncalibrated_control_limits"
    wrist_poses = (
        plan.staging_wrist_pose_world,
        plan.pregrasp_wrist_pose_world,
        plan.grasp_wrist_pose_world,
        plan.lift_wrist_pose_world,
        plan.carry_wrist_pose_world,
    )
    if any(pose is None for pose in wrist_poses):
        return "complete_wrist_trajectory_required"
    assert plan.grasp_wrist_pose_world is not None
    assert plan.lift_wrist_pose_world is not None
    assert plan.carry_wrist_pose_world is not None
    if not np.isclose(
        plan.lift_wrist_pose_world.translation_m[2]
        - plan.grasp_wrist_pose_world.translation_m[2],
        plan.lift_height_m,
        atol=1e-9,
    ) or not np.isclose(
        plan.carry_wrist_pose_world.translation_m[2],
        plan.lift_wrist_pose_world.translation_m[2],
        atol=1e-9,
    ):
        return "uncalibrated_lift_trajectory"
    actuator_for_key = {
        "x": "act_wx",
        "y": "act_wy",
        "z": "act_wz",
        "roll": "act_wr",
        "tilt": "act_wt",
        "yaw": "act_wyaw",
    }
    for pose in wrist_poses:
        assert pose is not None
        try:
            goal = _wrist_goal_for_pose(scene, pose)
        except ValueError:
            return "wrist_pose_frame_mismatch"
        for key, value in goal.items():
            aid = scene.act[actuator_for_key[key]]
            if scene.model.actuator_ctrllimited[aid]:
                low, high = scene.model.actuator_ctrlrange[aid]
                if value < low - 1e-8 or value > high + 1e-8:
                    return "wrist_pose_out_of_range"
    return None


def execute_short_can_pick_place(
    scene: AllegroProbeScene,
    plan: ManipulationPlan,
) -> ManipulationExecutionResult:
    """Execute either the v1 canonical or v2 pose-conditioned short-can plan."""

    if scene.config.backend != "allegro":
        raise ValueError("short-can pick/place requires the Allegro backend")
    if plan.skill not in {
        "short_can_pick_place",
        "pose_conditioned_short_can_pick_place",
    }:
        raise ValueError(f"unsupported manipulation skill: {plan.skill!r}")
    if plan.skill == "short_can_pick_place":
        if scene.config.full_hand_collisions or scene.full_hand_collisions_compiled():
            raise ValueError(
                "short_can_pick_place v1 requires the explicit distal-only "
                "legacy collision scene"
            )
        if any(f"obj{index}_pedestal" not in scene.geom for index in range(scene.n)):
            raise ValueError(
                "short_can_pick_place v1 requires the explicit elevated fixture"
            )
    if plan.skill == "pose_conditioned_short_can_pick_place":
        error = _pose_plan_execution_error(scene, plan)
        if error is not None:
            raise ValueError(f"invalid pose-conditioned plan for scene: {error}")

    run = _Execution(scene=scene, plan=plan)
    run.enter("handoff")
    if plan.handoff_policy == "reset_to_requested_pose":
        if plan.source_object_pose_world is None:
            raise ValueError("reset_to_requested_pose requires source_object_pose_world")
        scene.reset()
        scene.set_object_pose(
            plan.target,
            center_position_m=plan.source_object_pose_world.translation_m,
            quaternion_wxyz=plan.source_object_pose_world.quaternion_wxyz,
            record_initial=True,
        )
    elif plan.handoff_policy == "verify_live_pose":
        if plan.source_object_pose_world is None:
            raise ValueError("verify_live_pose requires source_object_pose_world")
    elif plan.handoff_policy == "reset_to_canonical_checkpoint" and plan.reset_before_execute:
        scene.reset()
    elif plan.handoff_policy not in {
        "reset_to_canonical_checkpoint",
        "continue_from_live_state",
    }:
        raise ValueError(f"unsupported handoff policy: {plan.handoff_policy!r}")
    scene.step(50)
    if plan.source_object_pose_world is not None:
        position_error, axis_error = _source_pose_error(scene, plan)
        run.quality["source_position_error_m"] = position_error
        run.quality["source_axis_error_rad"] = axis_error
        if position_error > plan.source_position_tolerance_m:
            run.fail("source_pose_position_mismatch")
            return _result(run, success=False)
        if axis_error > plan.source_axis_tolerance_rad:
            run.fail("source_pose_axis_mismatch")
            return _result(run, success=False)

    initial_object_pos = (
        scene.object_center_pos(plan.target).copy()
        if plan.source_object_pose_world is not None
        else scene.object_pos(plan.target).copy()
    )
    if plan.fixed_place_object_pose_world is not None:
        place_xy = np.asarray(
            plan.fixed_place_object_pose_world.translation_m[:2], dtype=float
        )
    else:
        place_xy = initial_object_pos[:2] + np.asarray(plan.place_offset_xy_m, dtype=float)
    run.trace["planned_place_xy_m"] = [place_xy.copy()]

    run.enter("preshape")
    if not _move_hand(
        run,
        plan.template.q_preshape,
        120,
        guard_clearance=plan.require_precontact_clearance,
    ):
        return _result(run, success=False)

    run.enter("approach")
    if plan.pregrasp_wrist_pose_world is not None:
        if plan.grasp_wrist_pose_world is None:
            raise ValueError("pose-conditioned plan requires grasp_wrist_pose_world")
        if plan.staging_wrist_pose_world is not None:
            # Translation and the 180-degree top-entry reorientation are separate
            # guarded moves.  Interpolating both together sweeps the long fingers
            # through objects/supports near the carriage's reset location even
            # though the staging endpoint itself is clear.
            staging_goal = _wrist_goal_for_pose(
                scene, plan.staging_wrist_pose_world
            )
            if not _move_wrist(
                run,
                goal={key: staging_goal[key] for key in ("x", "y", "z")},
                guard_environment=plan.require_precontact_clearance,
                forbid_object_contact=plan.require_precontact_clearance,
            ):
                return _result(run, success=False)
            if not _move_wrist(
                run,
                goal={
                    key: staging_goal[key]
                    for key in ("roll", "tilt", "yaw")
                },
                min_steps=160,
                guard_environment=plan.require_precontact_clearance,
                forbid_object_contact=plan.require_precontact_clearance,
            ):
                return _result(run, success=False)
        if not _move_wrist(
            run,
            goal=_wrist_goal_for_pose(scene, plan.pregrasp_wrist_pose_world),
            guard_environment=plan.require_precontact_clearance,
            forbid_object_contact=plan.require_precontact_clearance,
        ):
            return _result(run, success=False)
        if not _move_wrist(
            run,
            goal=_wrist_goal_for_pose(scene, plan.grasp_wrist_pose_world),
            min_steps=160,
            guard_environment=plan.require_precontact_clearance,
            forbid_object_contact=plan.require_precontact_clearance,
        ):
            return _result(run, success=False)
    else:
        object_center_z = float(
            scene.object_pos(plan.target)[2]
            - scene.task.objects[plan.target].size[2]
        )
        grasp_z = object_center_z - plan.template.wrist_to_object_center_z_m
        grasp_x = float(scene.object_pos(plan.target)[0])
        grasp_y = float(scene.object_pos(plan.target)[1] + plan.template.wrist_y_offset_m)
        if not _move_wrist(
            run,
            goal={
                "x": grasp_x,
                "y": grasp_y,
                "z": grasp_z + 0.075,
                "roll": 0.0,
                "tilt": 0.0,
                "yaw": 0.0,
            },
        ):
            return _result(run, success=False)
        if not _move_wrist(run, goal={"z": grasp_z}, min_steps=160):
            return _result(run, success=False)

    run.enter("contact_acquire")
    acquired = False
    first_contact_progress: Optional[float] = None
    for progress in np.linspace(0.0, 1.0, 181):
        run.closure_progress = float(progress)
        scene.command_allegro_joints(plan.template.pose(run.closure_progress))
        scene.step(5)
        snapshot = run.observe()
        if plan.forbid_palm_contact and snapshot.palm_object_contact:
            run.fail("palm_object_collision")
            break
        if plan.forbid_other_object_contact and (
            snapshot.hand_other_object_contact
            or snapshot.object_other_object_contact
        ):
            run.fail("other_object_collision")
            break
        if plan.allowed_hand_contact_tokens and any(
            not any(token in geom for token in plan.allowed_hand_contact_tokens)
            for geom in snapshot.hand_contact_geoms
        ):
            run.fail("forbidden_hand_link_contact")
            break
        if plan.forbid_inactive_finger_contact and (
            set(snapshot.hand_groups) - set(plan.template.active_fingers)
        ):
            run.fail("inactive_finger_contact")
            break
        if snapshot.hand_max_penetration_m > plan.max_place_penetration_m:
            run.fail("penetration_limit")
            break
        if snapshot.hand_support_contact:
            run.fail("hand_support_collision")
            break
        if plan.require_precontact_clearance and snapshot.hand_table_contact:
            run.fail("hand_table_collision")
            break
        if snapshot.hand_table_normal_force_N > plan.max_hand_table_force_N:
            run.fail("hand_table_force_limit")
            break
        if _required_contacts(run, snapshot) and first_contact_progress is None:
            first_contact_progress = run.closure_progress
        if (
            _required_contacts(run, snapshot)
            and snapshot.hand_normal_force_N >= plan.target_total_normal_force_N
        ):
            acquired = True
            break
    run.quality["first_legal_contact_progress"] = float(
        first_contact_progress if first_contact_progress is not None else -1.0
    )
    run.quality["closure_progress"] = run.closure_progress
    if not acquired:
        run.fail("legal_grasp_not_acquired")
        return _result(run, success=False)

    run.enter("grip_regulate")
    stable_steps = 0
    for _ in range(100):
        scene.step(1)
        snapshot = run.observe()
        if (
            plan.skill == "pose_conditioned_short_can_pick_place"
            and not _regulate_grasp(
                run,
                snapshot,
                allow_table_contact=True,
                max_force_N=plan.max_total_normal_force_N,
                max_penetration_m=plan.max_penetration_m,
            )
        ):
            return _result(run, success=False)
        if not _required_contacts(run, snapshot):
            stable_steps = 0
        elif snapshot.hand_normal_force_N >= 0.80 * plan.target_total_normal_force_N:
            stable_steps += 1
    if stable_steps < 80:
        run.fail("unstable_pregrasp")
        return _result(run, success=False)

    prelift_top_z = float(
        scene.object_center_pos(plan.target)[2]
        if plan.source_object_pose_world is not None
        else scene.object_pos(plan.target)[2]
    )
    run.enter("lift")
    lift_goal = (
        _wrist_goal_for_pose(scene, plan.lift_wrist_pose_world)
        if plan.lift_wrist_pose_world is not None
        else {"z": _wrist_controls(scene)["z"] + plan.lift_height_m}
    )
    if not _move_wrist(
        run,
        goal=lift_goal,
        min_steps=180,
        guard_grasp=plan.skill == "pose_conditioned_short_can_pick_place",
        allow_table_contact=plan.skill == "pose_conditioned_short_can_pick_place",
    ):
        return _result(run, success=False)
    snapshot = run.observe()
    if snapshot.support_contact or snapshot.table_contact:
        run.fail("support_contact_after_lift")
        return _result(run, success=False)
    if snapshot.hand_support_contact or snapshot.hand_table_contact:
        run.fail("hand_environment_collision_after_lift")
        return _result(run, success=False)
    if plan.forbid_other_object_contact and (
        snapshot.hand_other_object_contact
        or snapshot.object_other_object_contact
    ):
        run.fail("other_object_collision")
        return _result(run, success=False)
    if not _opposing_contacts(run, snapshot):
        run.fail("lost_legal_grasp")
        return _result(run, success=False)
    lift_distance = float(
        (
            scene.object_center_pos(plan.target)[2]
            if plan.source_object_pose_world is not None
            else scene.object_pos(plan.target)[2]
        )
        - prelift_top_z
    )
    run.quality["lift_distance_m"] = lift_distance
    if lift_distance < 0.020:
        run.fail("not_lifted")
        return _result(run, success=False)

    run.enter("carry")
    carry_goal = (
        _wrist_goal_for_pose(scene, plan.carry_wrist_pose_world)
        if plan.carry_wrist_pose_world is not None
        else {"x": float(place_xy[0]), "y": float(place_xy[1])}
    )
    if not _move_wrist(
        run,
        goal=carry_goal,
        guard_grasp=True,
    ):
        return _result(run, success=False)
    # Close the residual object-space error caused by the compliant grasp.  This
    # remains fixed-pose feedback, not arm planning.
    for _ in range(2):
        object_xy = (
            scene.object_center_pos(plan.target)[:2]
            if plan.source_object_pose_world is not None
            else scene.object_pos(plan.target)[:2]
        )
        correction = place_xy - object_xy
        if float(np.linalg.norm(correction)) <= 0.004:
            break
        controls = _wrist_controls(scene)
        if not _move_wrist(
            run,
            goal={
                "x": controls["x"] + float(correction[0]),
                "y": controls["y"] + float(correction[1]),
            },
            min_steps=80,
            guard_grasp=True,
        ):
            return _result(run, success=False)
    carry_distance = float(
        np.linalg.norm(
            (
                scene.object_center_pos(plan.target)[:2]
                if plan.source_object_pose_world is not None
                else scene.object_pos(plan.target)[:2]
            )
            - initial_object_pos[:2]
        )
    )
    run.quality["carry_distance_m"] = carry_distance
    if carry_distance < plan.min_carry_distance_m:
        run.fail("carry_distance_not_reached")
        return _result(run, success=False)

    run.enter("place_descent")
    scene.set_allegro_position_kp(2.0 if plan.use_gravity_settle else 8.0)
    descent_start = _wrist_controls(scene)["z"]
    placed = False
    descent_values = (
        np.linspace(0.0, 0.30, 601)
        if plan.skill == "pose_conditioned_short_can_pick_place"
        else np.linspace(0.0, 0.26, 1301)
    )
    for descent in descent_values:
        scene.command(z=descent_start - float(descent))
        # The carriage is position actuated and carries the hand/object load; give
        # it time to track each guarded descent target instead of outrunning it.
        scene.step(3)
        snapshot = run.observe()
        if snapshot.hand_max_penetration_m > plan.max_penetration_m:
            run.fail("penetration_limit")
            break
        if snapshot.hand_table_normal_force_N > plan.max_hand_table_force_N:
            run.fail("hand_table_force_limit")
            break
        if plan.require_precontact_clearance and snapshot.hand_table_contact:
            run.fail("hand_table_collision")
            break
        if plan.forbid_other_object_contact and (
            snapshot.hand_other_object_contact
            or snapshot.object_other_object_contact
        ):
            run.fail("other_object_collision")
            break
        object_bottom_z = (
            _object_bottom_z(scene, plan.target)
            if plan.source_object_pose_world is not None
            else float(
                scene.object_pos(plan.target)[2]
                - 2.0 * scene.task.objects[plan.target].size[2]
            )
        )
        clearance_guard = (
            object_bottom_z - plan.place_surface_z_m
            <= plan.max_release_height_m
        )
        hand_table_guard = (
            snapshot.hand_table_contact
            and snapshot.hand_table_normal_force_N
            >= plan.hand_table_release_guard_N
        )
        if snapshot.table_contact or clearance_guard or hand_table_guard:
            # Remove the accumulated downward position error before opening the
            # hand.  Otherwise the ideal carriage keeps pressing the can into the
            # table while the fingers release.
            scene.command(z=float(scene.sensor("wz_pos")[0]) + 0.002)
            scene.step(10)
            run.observe()
            placed = True
            break
        if not _regulate_grasp(
            run,
            snapshot,
            allow_table_contact=True,
            max_force_N=plan.max_place_normal_force_N,
            max_penetration_m=plan.max_place_penetration_m,
        ):
            break
    if not placed:
        run.fail("release_height_not_reached")
        return _result(run, success=False)

    if plan.post_descent_xy_correction:
        for _ in range(2):
            correction = place_xy - (
                scene.object_center_pos(plan.target)[:2]
                if plan.source_object_pose_world is not None
                else scene.object_pos(plan.target)[:2]
            )
            if float(np.linalg.norm(correction)) <= 0.002:
                break
            controls = _wrist_controls(scene)
            if not _move_wrist(
                run,
                goal={
                    "x": controls["x"] + float(correction[0]),
                    "y": controls["y"] + float(correction[1]),
                },
                min_steps=80,
                guard_grasp=True,
                allow_table_contact=True,
                max_force_N=plan.max_place_normal_force_N,
                max_penetration_m=plan.max_place_penetration_m,
            ):
                return _result(run, success=False)
        scene.command(z=float(scene.sensor("wz_pos")[0]) + 0.002)
        scene.step(10)
        run.observe()

    if plan.use_gravity_settle and not _settle_object_to_table(run):
        return _result(run, success=False)

    run.enter("release")
    pre_release_pos = (
        scene.object_center_pos(plan.target).copy()
        if plan.source_object_pose_world is not None
        else scene.object_pos(plan.target).copy()
    )
    if not _release_until_clear(run):
        return _result(run, success=False)
    clear_pos = (
        scene.object_center_pos(plan.target).copy()
        if plan.source_object_pose_world is not None
        else scene.object_pos(plan.target).copy()
    )
    scene.step(80)
    post_release_pos = (
        scene.object_center_pos(plan.target).copy()
        if plan.source_object_pose_world is not None
        else scene.object_pos(plan.target).copy()
    )
    run.quality["release_to_clear_displacement_m"] = float(
        np.linalg.norm(clear_pos - pre_release_pos)
    )
    run.quality["release_displacement_m"] = float(
        np.linalg.norm(post_release_pos - pre_release_pos)
    )

    run.enter("retreat")
    retreat_z = min(_wrist_controls(scene)["z"] + 0.10, 0.10)
    if not _move_wrist(
        run,
        goal={"z": retreat_z},
        min_steps=180,
        guard_environment=plan.require_precontact_clearance,
        forbid_object_contact=plan.require_precontact_clearance,
    ):
        return _result(run, success=False)
    if not _move_hand(
        run,
        plan.template.q_open,
        80,
        guard_clearance=plan.require_precontact_clearance,
    ):
        return _result(run, success=False)

    run.enter("final_verify")
    final_positions = []
    hold_steps = max(int(plan.hold_time_s / scene.dt), 1)
    for _ in range(hold_steps):
        scene.step(1)
        run.observe()
        final_positions.append(
            scene.object_center_pos(plan.target).copy()
            if plan.source_object_pose_world is not None
            else scene.object_pos(plan.target).copy()
        )
    final_pos = (
        scene.object_center_pos(plan.target).copy()
        if plan.source_object_pose_world is not None
        else scene.object_pos(plan.target).copy()
    )
    final_snapshot = scene.contact_snapshot(plan.target)
    if plan.fixed_place_object_pose_world is not None:
        goal_position = np.asarray(
            plan.fixed_place_object_pose_world.translation_m, dtype=float
        )
        place_error = float(np.linalg.norm(final_pos - goal_position))
        final_axis = quaternion_wxyz_to_matrix(scene.object_quat(plan.target))[:, 2]
        goal_axis = plan.fixed_place_object_pose_world.axis_z_parent
        final_tilt = float(
            np.arccos(
                np.clip(float(np.dot(final_axis, goal_axis)), -1.0, 1.0)
            )
        )
    else:
        place_error = float(np.linalg.norm(final_pos[:2] - place_xy))
        final_tilt = _object_tilt_rad(scene.object_quat(plan.target))
    positions = np.asarray(final_positions, dtype=float)
    final_drift = float(
        np.max(np.linalg.norm(positions - positions[0], axis=1))
        if len(positions)
        else 0.0
    )
    run.quality.update(
        {
            "place_error_m": place_error,
            "final_tilt_rad": final_tilt,
            "final_drift_m": final_drift,
            "final_table_contact": float(final_snapshot.table_contact),
            "final_hand_contact_group_count": float(len(final_snapshot.hand_groups)),
            "final_hand_table_contact": float(final_snapshot.hand_table_contact),
        }
    )
    if place_error > plan.max_place_error_m:
        run.fail("place_error")
    if final_tilt > plan.max_final_tilt_rad:
        run.fail("object_tilted")
    if final_drift > plan.max_final_drift_m:
        run.fail("object_not_settled")
    if not final_snapshot.table_contact:
        run.fail("object_not_supported_after_place")
    if final_snapshot.hand_groups:
        run.fail("release_incomplete")
    if final_snapshot.hand_table_contact:
        run.fail("hand_not_retreated")
    if plan.forbid_palm_contact and final_snapshot.palm_object_contact:
        run.fail("palm_object_collision")
    if plan.forbid_other_object_contact and (
        final_snapshot.hand_other_object_contact
        or final_snapshot.object_other_object_contact
    ):
        run.fail("other_object_collision")
    if run.max_grasp_carry_penetration_m > plan.max_penetration_m:
        run.fail("penetration_limit")
    if run.max_place_release_penetration_m > plan.max_place_penetration_m:
        run.fail("place_penetration_limit")
    if run.peak_grasp_carry_force_N > plan.max_total_normal_force_N:
        run.fail("hand_force_limit")
    if run.peak_place_release_force_N > plan.max_place_normal_force_N:
        run.fail("place_hand_force_limit")
    if run.peak_hand_table_force_N > plan.max_hand_table_force_N:
        run.fail("hand_table_force_limit")
    return _result(run, success=not run.violations)
