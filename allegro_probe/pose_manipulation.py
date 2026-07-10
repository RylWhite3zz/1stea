"""Training-free, pose-conditioned short-can manipulation interface.

This module is deliberately separate from the canonical v1 compatibility path.  A
caller supplies an observed world pose for one known short can; a controller owns one
absolute, fixed placement goal.  Geometry and cylinder symmetry generate wrist poses,
while the existing contact-feedback executor closes and regulates the Allegro hand.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from typing import Any, Dict, List, Literal, Optional, Tuple

import numpy as np

from allegro_probe.geometry import (
    RigidTransform,
    matrix_to_quaternion_wxyz,
    rotation_about_z,
    rotation_to_xyz_rpy,
)
from allegro_probe.manipulation import (
    AllegroHandTemplate,
    ManipulationExecutionResult,
    ManipulationPlan,
    ManipulationPlanDecision,
    ShortCanPickPlaceRequest,
    build_short_can_pick_place_plan,
    execute_short_can_pick_place,
    _pose_plan_execution_error,
)
from allegro_probe.models import ProbeResult
from allegro_probe.scene import AllegroProbeScene


POSE_MANIPULATION_SCHEMA_VERSION = "allegro_manip.pose.v2"
TOP_PINCH_LIFT_HEIGHT_M = 0.130


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
class ObjectPoseObservation:
    """One pose-provider output; the object frame origin is its geometry center."""

    target: int
    object_id: str
    T_world_object: RigidTransform
    timestamp_s: Optional[float] = None
    confidence: float = 1.0
    symmetry: str = "continuous_about_local_z"

    def __post_init__(self) -> None:
        if self.T_world_object.parent_frame != "world":
            raise ValueError("T_world_object parent frame must be 'world'")
        if self.T_world_object.child_frame != self.object_id:
            raise ValueError(
                "T_world_object child frame must equal object_id, got "
                f"{self.T_world_object.child_frame!r} and {self.object_id!r}"
            )
        confidence = float(self.confidence)
        if not np.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be finite and in [0, 1]")
        if self.timestamp_s is not None and not np.isfinite(float(self.timestamp_s)):
            raise ValueError("timestamp_s must be finite when provided")
        if self.symmetry != "continuous_about_local_z":
            raise ValueError("short_can requires continuous_about_local_z symmetry")

    def to_dict(self) -> Dict[str, Any]:
        return _convert(asdict(self))


@dataclass(frozen=True)
class FixedPlaceSpec:
    """Controller-owned absolute goal, not an offset from the observed source."""

    goal_id: str
    T_world_object_goal: RigidTransform
    surface_z_m: float = 0.0
    max_position_error_m: float = 0.035
    max_tilt_rad: float = 0.20
    yaw_free_about_object_axis: bool = True

    def __post_init__(self) -> None:
        if not self.goal_id:
            raise ValueError("goal_id must be non-empty")
        if self.T_world_object_goal.parent_frame != "world":
            raise ValueError("fixed goal parent frame must be 'world'")
        for name in ("surface_z_m", "max_position_error_m", "max_tilt_rad"):
            value = float(getattr(self, name))
            if not np.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.max_position_error_m <= 0.0 or self.max_tilt_rad <= 0.0:
            raise ValueError("fixed-goal tolerances must be positive")
        if self.max_position_error_m > 0.035:
            raise ValueError("max_position_error_m must be <= 0.035 for v2")
        if self.max_tilt_rad > 0.20:
            raise ValueError("max_tilt_rad must be <= 0.20 for v2")

    def to_dict(self) -> Dict[str, Any]:
        return _convert(asdict(self))


@dataclass(frozen=True)
class PoseConditionedPickPlaceRequest:
    object_pose: ObjectPoseObservation
    fixed_goal_id: str = "short_can_drop_zone_v1"
    handoff_policy: Literal[
        "reset_to_requested_pose", "verify_live_pose"
    ] = "verify_live_pose"

    def __post_init__(self) -> None:
        if self.handoff_policy not in {
            "reset_to_requested_pose",
            "verify_live_pose",
        }:
            raise ValueError(f"unsupported handoff_policy {self.handoff_policy!r}")

    def to_dict(self) -> Dict[str, Any]:
        return _convert(asdict(self))


@dataclass(frozen=True)
class ObjectRelativeGraspTemplate:
    name: str
    T_object_wrist: RigidTransform
    pregrasp_offset_object_m: Tuple[float, float, float]
    q_open: np.ndarray
    q_preshape: np.ndarray
    q_contact: np.ndarray
    q_squeeze_limit: np.ndarray
    symmetry_yaw_samples_rad: Tuple[float, ...]
    required_contact_groups: Tuple[str, ...]
    required_object_geom_suffix: str
    allowed_hand_contact_tokens: Tuple[str, ...]

    def __post_init__(self) -> None:
        for name in ("q_open", "q_preshape", "q_contact", "q_squeeze_limit"):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (16,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be a finite 16-vector")
            object.__setattr__(self, name, value.copy())
        offset = np.asarray(self.pregrasp_offset_object_m, dtype=float)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("pregrasp_offset_object_m must be a finite 3-vector")
        object.__setattr__(
            self,
            "pregrasp_offset_object_m",
            tuple(float(value) for value in offset),
        )


@dataclass(frozen=True)
class GraspCandidate:
    candidate_id: str
    symmetry_yaw_rad: float
    T_world_wrist_staging: RigidTransform
    T_world_wrist_pregrasp: RigidTransform
    T_world_wrist_grasp: RigidTransform
    T_world_wrist_lift: RigidTransform
    T_world_wrist_carry: RigidTransform
    score: float
    carriage_margin: float

    def to_dict(self) -> Dict[str, Any]:
        return _convert(asdict(self))


def short_can_top_pinch_template(
    scene: AllegroProbeScene, object_id: str
) -> ObjectRelativeGraspTemplate:
    """Return a top-entry template that does not pass the palm through the can.

    The wrist is rolled by pi so the existing cylinder synergy curls downward.  The
    can is acquired between middle fingertip and thumb tip at its top lip.  Unlike the
    v1 under-wrap template, this works for a can resting directly on the table when
    the scene is compiled with full hand collision proxies.
    """

    return ObjectRelativeGraspTemplate(
        name="short_can_top_pinch_v1",
        T_object_wrist=RigidTransform(
            parent_frame=object_id,
            child_frame="wrist",
            translation_m=(0.0, -0.020, 0.130),
            quaternion_wxyz=(0.0, 1.0, 0.0, 0.0),
        ),
        pregrasp_offset_object_m=(0.0, 0.0, 0.075),
        q_open=scene.allegro_grip_pose(0.0),
        q_preshape=scene.allegro_grip_pose(0.10),
        q_contact=scene.allegro_grip_pose(0.80),
        # The wider limit gives the force regulator headroom as a heavy can
        # microslips; execution normally stops well before this endpoint.
        q_squeeze_limit=scene.allegro_grip_pose(0.98),
        symmetry_yaw_samples_rad=tuple(
            float(value) for value in np.linspace(-np.pi, np.pi, 13)[:-1]
        ),
        required_contact_groups=("mf", "th"),
        required_object_geom_suffix="_top_lip",
        # Distal links may join the two tip contacts as the compliant can tilts
        # during lift/place.  Palm, base, proximal, and all environment contacts
        # remain forbidden by the executor's independent gates.
        allowed_hand_contact_tokens=(
            "fingertip_collision",
            "thumbtip_collision",
            "distal_collision",
        ),
    )


def _canonical_cylinder_pose(
    observation: ObjectPoseObservation,
) -> Tuple[RigidTransform, float]:
    """Remove unobservable cylinder yaw while retaining its measured local-z axis."""

    axis = observation.T_world_object.axis_z_parent
    axis /= np.linalg.norm(axis)
    tilt = float(np.arccos(np.clip(float(axis[2]), -1.0, 1.0)))
    reference = np.asarray([1.0, 0.0, 0.0], dtype=float)
    x_axis = reference - float(np.dot(reference, axis)) * axis
    if float(np.linalg.norm(x_axis)) < 1e-8:
        reference = np.asarray([0.0, 1.0, 0.0], dtype=float)
        x_axis = reference - float(np.dot(reference, axis)) * axis
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    rotation = np.column_stack((x_axis, y_axis, axis))
    return (
        RigidTransform(
            parent_frame="world",
            child_frame=observation.object_id,
            translation_m=observation.T_world_object.translation_m,
            quaternion_wxyz=matrix_to_quaternion_wxyz(rotation),
        ),
        tilt,
    )


def _transform_from_matrix(
    matrix: np.ndarray, *, child_frame: str
) -> RigidTransform:
    return RigidTransform.from_matrix(
        parent_frame="world", child_frame=child_frame, matrix=matrix
    )


def _carriage_controls(
    scene: AllegroProbeScene, pose: RigidTransform
) -> np.ndarray:
    roll, tilt, yaw = rotation_to_xyz_rpy(pose.rotation)
    x, y, z = pose.translation_m
    return np.asarray(
        [x, y, z - scene.config.palm_height, roll, tilt, yaw], dtype=float
    )


def _carriage_margin(scene: AllegroProbeScene, poses: Tuple[RigidTransform, ...]) -> float:
    names = ("act_wx", "act_wy", "act_wz", "act_wr", "act_wt", "act_wyaw")
    margin = float("inf")
    for pose in poses:
        controls = _carriage_controls(scene, pose)
        for value, name in zip(controls, names):
            aid = scene.act[name]
            if scene.model.actuator_ctrllimited[aid]:
                low, high = scene.model.actuator_ctrlrange[aid]
                if value < low - 1e-8 or value > high + 1e-8:
                    return -1.0
                span = max(float(high - low), 1e-9)
                margin = min(margin, float(min(value - low, high - value) / span))
    return float(margin)


def _table_geometry(scene: AllegroProbeScene) -> Tuple[np.ndarray, np.ndarray]:
    gid = scene.geom.get("table")
    if gid is None:
        raise ValueError("pose-conditioned manipulation requires a table geom")
    return (
        np.asarray(scene.data.geom_xpos[gid], dtype=float).copy(),
        np.asarray(scene.model.geom_size[gid], dtype=float).copy(),
    )


def _compiled_full_hand_collisions(scene: AllegroProbeScene) -> bool:
    return scene.full_hand_collisions_compiled()


def _inside_table_xy(
    scene: AllegroProbeScene, position_m: np.ndarray, clearance_m: float
) -> bool:
    center, half_size = _table_geometry(scene)
    offset = np.abs(np.asarray(position_m[:2], dtype=float) - center[:2])
    return bool(np.all(offset + float(clearance_m) <= half_size[:2]))


def _clear_of_other_objects(
    scene: AllegroProbeScene,
    target: int,
    position_m: np.ndarray,
    *,
    extra_clearance_m: float,
) -> bool:
    target_radius = max(
        scene.task.objects[target].size[0],
        scene.task.objects[target].size[1],
    )
    for index, other in enumerate(scene.task.objects):
        if index == target:
            continue
        other_radius = max(other.size[0], other.size[1])
        required = target_radius + other_radius + float(extra_clearance_m)
        distance = float(
            np.linalg.norm(
                np.asarray(position_m[:2], dtype=float)
                - scene.object_center_pos(index)[:2]
            )
        )
        if distance < required:
            return False
    return True


def generate_short_can_grasp_candidates(
    scene: AllegroProbeScene,
    observation: ObjectPoseObservation,
    fixed_goal: FixedPlaceSpec,
    template: ObjectRelativeGraspTemplate,
) -> List[GraspCandidate]:
    """Generate deterministic symmetry-equivalent candidates without learning."""

    if fixed_goal.T_world_object_goal.child_frame != observation.object_id:
        raise ValueError("fixed goal child frame must match the observed object")
    if (
        template.T_object_wrist.parent_frame != observation.object_id
        or template.T_object_wrist.child_frame != "wrist"
    ):
        raise ValueError("grasp template must use frames object<-wrist")

    source, tilt = _canonical_cylinder_pose(observation)
    if tilt > 0.12:
        return []
    goal_axis = fixed_goal.T_world_object_goal.axis_z_parent
    goal_tilt = float(
        np.arccos(np.clip(float(np.dot(goal_axis, (0.0, 0.0, 1.0))), -1.0, 1.0))
    )
    if goal_tilt > 0.02:
        return []

    current_position = scene.wrist_pos().copy()
    candidates: List[GraspCandidate] = []
    for index, symmetry_yaw in enumerate(template.symmetry_yaw_samples_rad):
        symmetry = np.eye(4, dtype=float)
        symmetry[:3, :3] = rotation_about_z(symmetry_yaw)
        grasp_matrix = source.matrix @ symmetry @ template.T_object_wrist.matrix
        grasp = _transform_from_matrix(grasp_matrix, child_frame="wrist")

        pregrasp_matrix = grasp_matrix.copy()
        pregrasp_matrix[:3, 3] += (
            source.rotation
            @ rotation_about_z(symmetry_yaw)
            @ np.asarray(template.pregrasp_offset_object_m, dtype=float)
        )
        pregrasp = _transform_from_matrix(pregrasp_matrix, child_frame="wrist")

        staging_matrix = pregrasp_matrix.copy()
        staging_matrix[:3, 3] = np.asarray(
            [
                pregrasp.translation_m[0],
                pregrasp.translation_m[1],
                max(0.46, pregrasp.translation_m[2] + 0.12),
            ],
            dtype=float,
        )
        staging = _transform_from_matrix(staging_matrix, child_frame="wrist")

        lift_matrix = grasp_matrix.copy()
        # The compliant two-finger pinch loses height under load and may tilt the
        # can.  A high carry clears the remaining candidates before descent.
        lift_matrix[2, 3] += TOP_PINCH_LIFT_HEIGHT_M
        lift = _transform_from_matrix(lift_matrix, child_frame="wrist")

        goal_matrix = fixed_goal.T_world_object_goal.matrix @ symmetry @ template.T_object_wrist.matrix
        carry_matrix = goal_matrix.copy()
        carry_matrix[2, 3] = lift_matrix[2, 3]
        carry = _transform_from_matrix(carry_matrix, child_frame="wrist")

        margin = _carriage_margin(scene, (staging, pregrasp, grasp, lift, carry))
        if margin < 0.0:
            continue
        translation_cost = float(
            np.linalg.norm(np.asarray(pregrasp.translation_m) - current_position)
        )
        rotation_cost = abs(float(symmetry_yaw))
        score = translation_cost + 0.015 * rotation_cost - 0.05 * margin
        candidates.append(
            GraspCandidate(
                candidate_id=f"top_pinch_yaw_{index:02d}",
                symmetry_yaw_rad=float(symmetry_yaw),
                T_world_wrist_staging=staging,
                T_world_wrist_pregrasp=pregrasp,
                T_world_wrist_grasp=grasp,
                T_world_wrist_lift=lift,
                T_world_wrist_carry=carry,
                score=score,
                carriage_margin=margin,
            )
        )
    return sorted(candidates, key=lambda item: (item.score, item.candidate_id))


def build_pose_conditioned_short_can_plan(
    scene: AllegroProbeScene,
    probe_result: ProbeResult,
    request: PoseConditionedPickPlaceRequest,
    fixed_goal: FixedPlaceSpec,
) -> ManipulationPlanDecision:
    """Build one absolute-goal plan from a supplied object pose and geometry."""

    def reject(reason: str) -> ManipulationPlanDecision:
        return ManipulationPlanDecision(executable=False, reason=reason)

    observation = request.object_pose
    if request.fixed_goal_id != fixed_goal.goal_id:
        return reject("fixed_goal_id_mismatch")
    if fixed_goal.T_world_object_goal.child_frame != observation.object_id:
        return reject("fixed_goal_object_frame_mismatch")
    if not fixed_goal.yaw_free_about_object_axis:
        return reject("yaw_constrained_goal_unsupported")
    if not scene.config.full_hand_collisions:
        return reject("full_hand_collisions_required")
    if not _compiled_full_hand_collisions(scene):
        return reject("compiled_collision_model_insufficient")
    if any(f"obj{index}_pedestal" in scene.geom for index in range(scene.n)):
        return reject("support_free_table_scene_required")
    roll_range = scene.model.actuator_ctrlrange[scene.act["act_wr"]]
    if float(roll_range[0]) > -np.pi + 1e-3 or float(roll_range[1]) < np.pi - 1e-3:
        return reject("top_grasp_wrist_rotation_unreachable")
    if observation.confidence < 0.5:
        return reject("object_pose_low_confidence")
    if observation.target < 0 or observation.target >= scene.n:
        return reject("target_out_of_range")
    obj = scene.task.objects[observation.target]
    if observation.object_id != obj.object_id:
        return reject("object_pose_target_mismatch")
    if not probe_result.scene_id:
        return reject("probe_scene_id_required")
    if probe_result.scene_id != scene.task.scene_id:
        return reject("probe_scene_mismatch")

    base = build_short_can_pick_place_plan(
        scene,
        probe_result,
        ShortCanPickPlaceRequest(
            target=observation.target,
            place_offset_xy_m=(0.0, scene.config.short_can_place_y),
        ),
        _require_legacy_scene=False,
    )
    if not base.executable or base.plan is None or base.context is None:
        return base
    if not 0.025 <= base.context.mass_estimate_kg <= 0.70:
        return reject("short_can_mass_out_of_calibrated_range")
    if not 0.25 <= base.context.weight_signal_N <= 6.60:
        return reject("short_can_weight_out_of_calibrated_range")

    _, source_tilt = _canonical_cylinder_pose(observation)
    if source_tilt > 0.12:
        return reject("upright_short_can_required")
    source_center = np.asarray(observation.T_world_object.translation_m, dtype=float)
    source_axis = observation.T_world_object.axis_z_parent
    source_bottom = float(
        source_center[2]
        - obj.size[2] * abs(float(source_axis[2]))
        - max(obj.size[0], obj.size[1])
        * float(np.linalg.norm(source_axis[:2]))
    )
    table_center, table_half_size = _table_geometry(scene)
    table_top = float(table_center[2] + table_half_size[2])
    if abs(float(fixed_goal.surface_z_m) - table_top) > 0.003:
        return reject("fixed_surface_not_table")
    if source_bottom < fixed_goal.surface_z_m - 0.003:
        return reject("source_below_support_surface")
    if abs(source_bottom - fixed_goal.surface_z_m) > 0.010:
        return reject("source_not_on_support_surface")
    goal_center = np.asarray(fixed_goal.T_world_object_goal.translation_m, dtype=float)
    goal_axis = fixed_goal.T_world_object_goal.axis_z_parent
    goal_tilt = float(
        np.arccos(np.clip(float(np.dot(goal_axis, (0.0, 0.0, 1.0))), -1.0, 1.0))
    )
    if goal_tilt > 0.02:
        return reject("upright_fixed_goal_required")
    clearance = max(obj.size[0], obj.size[1]) + 0.005
    if not _inside_table_xy(scene, source_center, clearance):
        return reject("source_outside_table_workspace")
    if not _inside_table_xy(scene, goal_center, clearance):
        return reject("fixed_goal_outside_table_workspace")
    if not _clear_of_other_objects(
        scene,
        observation.target,
        source_center,
        extra_clearance_m=0.080,
    ):
        return reject("source_obstacle_clearance_required")
    if not _clear_of_other_objects(
        scene,
        observation.target,
        goal_center,
        extra_clearance_m=0.045,
    ):
        return reject("fixed_goal_obstacle_clearance_required")
    expected_goal_z = float(fixed_goal.surface_z_m + obj.size[2])
    if abs(float(goal_center[2]) - expected_goal_z) > 0.010:
        return reject("fixed_goal_height_mismatch")

    template = short_can_top_pinch_template(scene, observation.object_id)
    candidates = generate_short_can_grasp_candidates(
        scene, observation, fixed_goal, template
    )
    if not candidates:
        return reject("no_reachable_grasp_candidate")
    selected = candidates[0]

    hand_template = AllegroHandTemplate(
        name=template.name,
        q_open=template.q_open,
        q_preshape=template.q_preshape,
        q_contact=template.q_contact,
        q_squeeze_limit=template.q_squeeze_limit,
        active_fingers=("mf", "th"),
        required_contact_groups=template.required_contact_groups,
        required_object_geom_suffix=template.required_object_geom_suffix,
        wrist_y_offset_m=0.0,
        wrist_to_object_center_z_m=0.0,
        use_contact_waypoint=True,
    )
    source = _canonical_cylinder_pose(observation)[0]
    source_to_goal_xy = float(np.linalg.norm(goal_center[:2] - source_center[:2]))
    minimum_carry = max(0.0, source_to_goal_xy - fixed_goal.max_position_error_m)
    # Top-pinch force calibration is different from the legacy under-wrap grasp.
    # The heft weight signal remains the only conditioning input; values are
    # bounded by the validated full-collision short-can envelope.
    top_pinch_force = float(
        np.clip(7.0 + 0.82 * base.context.weight_signal_N, 8.1, 10.7)
    )
    # This gripper-like carriage must complete lateral transfer before a heavy can
    # can microslip out of the top pinch.  The value remains a bounded task-space
    # speed, not a learned policy output.
    top_pinch_speed = float(
        np.clip(0.070 + 0.008 * base.context.weight_signal_N, 0.080, 0.106)
    )

    plan: ManipulationPlan = replace(
        base.plan,
        schema_version=POSE_MANIPULATION_SCHEMA_VERSION,
        skill="pose_conditioned_short_can_pick_place",
        template=hand_template,
        place_offset_xy_m=(0.0, 0.0),
        reset_before_execute=request.handoff_policy == "reset_to_requested_pose",
        lift_height_m=TOP_PINCH_LIFT_HEIGHT_M,
        min_carry_distance_m=minimum_carry,
        hold_time_s=0.50,
        target_total_normal_force_N=top_pinch_force,
        max_total_normal_force_N=20.0,
        max_place_normal_force_N=30.0,
        hand_table_release_guard_N=0.0,
        max_hand_table_force_N=0.0,
        max_wrist_speed_mps=top_pinch_speed,
        max_penetration_m=0.0068,
        max_place_penetration_m=0.0068,
        max_release_height_m=0.008,
        place_surface_z_m=float(fixed_goal.surface_z_m),
        post_descent_xy_correction=True,
        use_gravity_settle=False,
        max_place_error_m=float(fixed_goal.max_position_error_m),
        max_final_tilt_rad=float(fixed_goal.max_tilt_rad),
        handoff_policy=request.handoff_policy,
        pose_source="request_object_pose",
        source_object_pose_world=source,
        fixed_place_object_pose_world=fixed_goal.T_world_object_goal,
        fixed_goal_id=fixed_goal.goal_id,
        staging_wrist_pose_world=selected.T_world_wrist_staging,
        pregrasp_wrist_pose_world=selected.T_world_wrist_pregrasp,
        grasp_wrist_pose_world=selected.T_world_wrist_grasp,
        lift_wrist_pose_world=selected.T_world_wrist_lift,
        carry_wrist_pose_world=selected.T_world_wrist_carry,
        selected_grasp_candidate_id=selected.candidate_id,
        selected_symmetry_yaw_rad=selected.symmetry_yaw_rad,
        require_precontact_clearance=True,
        forbid_palm_contact=True,
        forbid_other_object_contact=True,
        min_contact_force_per_group_N=0.20,
        forbid_inactive_finger_contact=True,
        allowed_hand_contact_tokens=template.allowed_hand_contact_tokens,
    )
    context = replace(
        base.context,
        schema_version=POSE_MANIPULATION_SCHEMA_VERSION,
        handoff_policy=request.handoff_policy,
        pose_source="request_object_pose",
        source_object_pose_world=source,
        fixed_place_object_pose_world=fixed_goal.T_world_object_goal,
        fixed_goal_id=fixed_goal.goal_id,
        selected_grasp_candidate_id=selected.candidate_id,
        target_total_normal_force_N=top_pinch_force,
        max_wrist_speed_mps=top_pinch_speed,
    )
    return ManipulationPlanDecision(
        executable=True,
        reason="ok",
        context=context,
        plan=plan,
    )


def execute_pose_conditioned_short_can_plan(
    scene: AllegroProbeScene, plan: ManipulationPlan
) -> ManipulationExecutionResult:
    if plan.schema_version != POSE_MANIPULATION_SCHEMA_VERSION:
        raise ValueError(
            "pose-conditioned executor requires "
            f"{POSE_MANIPULATION_SCHEMA_VERSION!r}, got {plan.schema_version!r}"
        )
    error = _pose_plan_execution_error(scene, plan)
    if error is not None:
        raise ValueError(f"invalid pose-conditioned plan for scene: {error}")
    return execute_short_can_pick_place(scene, plan)


class PoseConditionedShortCanController:
    """Manipulation-stage interface with one controller-owned fixed destination."""

    def __init__(self, scene: AllegroProbeScene, fixed_goal: FixedPlaceSpec):
        self.scene = scene
        self.fixed_goal = fixed_goal

    def plan(
        self,
        probe_result: ProbeResult,
        request: PoseConditionedPickPlaceRequest,
    ) -> ManipulationPlanDecision:
        return build_pose_conditioned_short_can_plan(
            self.scene, probe_result, request, self.fixed_goal
        )

    def execute(self, plan: ManipulationPlan) -> ManipulationExecutionResult:
        error = _pose_plan_execution_error(self.scene, plan)
        if error is not None:
            raise ValueError(f"invalid pose-conditioned plan for scene: {error}")
        if plan.fixed_goal_id != self.fixed_goal.goal_id:
            raise ValueError("plan fixed_goal_id does not belong to this controller")
        if plan.fixed_place_object_pose_world is None or not np.allclose(
            plan.fixed_place_object_pose_world.matrix,
            self.fixed_goal.T_world_object_goal.matrix,
            atol=1e-9,
        ):
            raise ValueError("plan fixed pose does not belong to this controller")
        goal_policy_matches = (
            np.isclose(
                plan.place_surface_z_m,
                self.fixed_goal.surface_z_m,
                atol=1e-9,
            )
            and np.isclose(
                plan.max_place_error_m,
                self.fixed_goal.max_position_error_m,
                atol=1e-9,
            )
            and np.isclose(
                plan.max_final_tilt_rad,
                self.fixed_goal.max_tilt_rad,
                atol=1e-9,
            )
            and self.fixed_goal.yaw_free_about_object_axis
        )
        if not goal_policy_matches:
            raise ValueError("plan fixed-goal policy does not belong to this controller")
        return execute_pose_conditioned_short_can_plan(self.scene, plan)

    def run(
        self,
        probe_result: ProbeResult,
        request: PoseConditionedPickPlaceRequest,
    ) -> ManipulationExecutionResult:
        decision = self.plan(probe_result, request)
        if not decision.executable or decision.plan is None:
            raise ValueError(f"no executable pose-conditioned plan: {decision.reason}")
        return self.execute(decision.plan)


def run_pose_conditioned_short_can_pick_place(
    scene: AllegroProbeScene,
    probe_result: ProbeResult,
    request: PoseConditionedPickPlaceRequest,
    fixed_goal: FixedPlaceSpec,
) -> ManipulationExecutionResult:
    """One-call typed interface intended for the manipulation stage."""

    return PoseConditionedShortCanController(scene, fixed_goal).run(
        probe_result, request
    )
