from dataclasses import replace

import numpy as np
import pytest

from allegro_probe import (
    AllegroHandBackend,
    FixedPlaceSpec,
    ObjectPoseObservation,
    PoseConditionedPickPlaceRequest,
    PoseConditionedShortCanController,
    ProbeCommand,
    ProbeHarness,
    ProbeResult,
    RigidTransform,
    build_pose_conditioned_short_can_plan,
    execute_pose_conditioned_short_can_plan,
    make_demo_scene,
)


def _synthetic_heft(spec, target: int) -> ProbeResult:
    obj = spec.objects[target]
    return ProbeResult(
        object_id=obj.object_id,
        target=target,
        primitive="heft",
        scene_id=spec.scene_id,
        backend="allegro",
        valid=True,
        features={
            "m_est_kg": obj.mass_kg,
            "weight_signal_N": obj.mass_kg * 9.81,
        },
    )


def _scene(spec):
    return AllegroHandBackend.create(
        spec,
        allegro_grasp_lift=0.0,
        full_hand_collisions=True,
        wrist_roll_limit_rad=np.pi,
    ).scene


def _observation(spec, target: int, xy=(0.06, -0.12)):
    obj = spec.objects[target]
    return ObjectPoseObservation(
        target=target,
        object_id=obj.object_id,
        T_world_object=RigidTransform(
            "world",
            obj.object_id,
            (float(xy[0]), float(xy[1]), obj.size[2]),
        ),
    )


def _goal(spec, target: int, xy=(0.0, 0.12)):
    obj = spec.objects[target]
    return FixedPlaceSpec(
        "drop",
        RigidTransform(
            "world",
            obj.object_id,
            (float(xy[0]), float(xy[1]), obj.size[2]),
        ),
    )


def test_rigid_transform_compose_and_inverse_use_parent_child_convention() -> None:
    half = np.sqrt(0.5)
    world_object = RigidTransform(
        "world", "object", (1.0, 2.0, 3.0), (half, 0.0, 0.0, half)
    )
    object_wrist = RigidTransform("object", "wrist", (0.1, 0.0, 0.0))

    world_wrist = world_object.compose(object_wrist)
    assert world_wrist.parent_frame == "world"
    assert world_wrist.child_frame == "wrist"
    assert np.allclose(world_wrist.translation_m, (1.0, 2.1, 3.0))
    assert np.allclose(
        world_wrist.compose(world_wrist.inverse()).matrix,
        np.eye(4),
        atol=1e-7,
    )
    with pytest.raises(ValueError, match="frame mismatch"):
        object_wrist.compose(world_object)


def test_pose_plan_admission_requires_real_collision_and_support_free_scene() -> None:
    spec = make_demo_scene("mass", 3, 0)
    probe = _synthetic_heft(spec, 1)
    request = PoseConditionedPickPlaceRequest(_observation(spec, 1), "drop")
    goal = _goal(spec, 1)

    partial = AllegroHandBackend.create(
        spec,
        allegro_grasp_lift=0.0,
        full_hand_collisions=False,
    ).scene
    decision = build_pose_conditioned_short_can_plan(
        partial, probe, request, goal
    )
    assert not decision.executable
    assert decision.reason == "full_hand_collisions_required"

    pedestal = AllegroHandBackend.create(
        spec,
        allegro_grasp_lift=0.090,
        full_hand_collisions=True,
        wrist_roll_limit_rad=np.pi,
    ).scene
    decision = build_pose_conditioned_short_can_plan(
        pedestal, probe, request, goal
    )
    assert not decision.executable
    assert decision.reason == "support_free_table_scene_required"

    scene = _scene(spec)
    decision = build_pose_conditioned_short_can_plan(scene, probe, request, goal)
    assert decision.executable, decision.to_dict()

    palm_gid = scene.geom["palm_palm_collision"]
    original_contype = int(scene.model.geom_contype[palm_gid])
    scene.model.geom_contype[palm_gid] = 0
    decision = build_pose_conditioned_short_can_plan(scene, probe, request, goal)
    assert not decision.executable
    assert decision.reason == "compiled_collision_model_insufficient"
    scene.model.geom_contype[palm_gid] = original_contype

    wrong_frame_goal = replace(
        goal,
        T_world_object_goal=RigidTransform(
            "world", "another_object", goal.T_world_object_goal.translation_m
        ),
    )
    decision = build_pose_conditioned_short_can_plan(
        scene, probe, request, wrong_frame_goal
    )
    assert not decision.executable
    assert decision.reason == "fixed_goal_object_frame_mismatch"

    constrained_yaw = replace(goal, yaw_free_about_object_axis=False)
    decision = build_pose_conditioned_short_can_plan(
        scene, probe, request, constrained_yaw
    )
    assert not decision.executable
    assert decision.reason == "yaw_constrained_goal_unsupported"

    wrong_scene_probe = replace(probe, scene_id="another_scene")
    decision = build_pose_conditioned_short_can_plan(
        scene, wrong_scene_probe, request, goal
    )
    assert not decision.executable
    assert decision.reason == "probe_scene_mismatch"


def test_pose_plan_rejects_unreachable_or_out_of_envelope_requests() -> None:
    spec = make_demo_scene("mass", 3, 0)
    scene = _scene(spec)
    probe = _synthetic_heft(spec, 1)
    goal = _goal(spec, 1)

    outside = _observation(spec, 1, xy=(0.60, -0.12))
    decision = build_pose_conditioned_short_can_plan(
        scene,
        probe,
        PoseConditionedPickPlaceRequest(outside, "drop"),
        goal,
    )
    assert not decision.executable
    assert decision.reason == "source_outside_table_workspace"

    upside_down = replace(
        _observation(spec, 1),
        T_world_object=RigidTransform(
            "world", "obj1", (0.06, -0.12, 0.036), (0.0, 1.0, 0.0, 0.0)
        ),
    )
    decision = build_pose_conditioned_short_can_plan(
        scene,
        probe,
        PoseConditionedPickPlaceRequest(upside_down, "drop"),
        goal,
    )
    assert not decision.executable
    assert decision.reason == "upright_short_can_required"

    uncalibrated = replace(
        probe,
        features={"m_est_kg": 0.9, "weight_signal_N": 8.8},
    )
    decision = build_pose_conditioned_short_can_plan(
        scene,
        uncalibrated,
        PoseConditionedPickPlaceRequest(_observation(spec, 1), "drop"),
        goal,
    )
    assert not decision.executable
    assert decision.reason == "short_can_mass_out_of_calibrated_range"


def test_source_pose_changes_grasp_pose_but_not_absolute_fixed_goal() -> None:
    spec = make_demo_scene("mass", 3, 0)
    scene = _scene(spec)
    target = 1
    probe = _synthetic_heft(spec, target)
    goal = _goal(spec, target)
    source_a = _observation(spec, target, (-0.06, -0.13))
    source_b = _observation(spec, target, (0.06, -0.13))

    plan_a = build_pose_conditioned_short_can_plan(
        scene,
        probe,
        PoseConditionedPickPlaceRequest(source_a, "drop"),
        goal,
    ).plan
    plan_b = build_pose_conditioned_short_can_plan(
        scene,
        probe,
        PoseConditionedPickPlaceRequest(source_b, "drop"),
        goal,
    ).plan
    assert plan_a is not None and plan_b is not None
    delta = np.asarray(
        source_b.T_world_object.translation_m
    ) - np.asarray(source_a.T_world_object.translation_m)
    grasp_delta = np.asarray(
        plan_b.grasp_wrist_pose_world.translation_m
    ) - np.asarray(plan_a.grasp_wrist_pose_world.translation_m)
    assert np.allclose(grasp_delta, delta)
    assert plan_a.fixed_place_object_pose_world == goal.T_world_object_goal
    assert plan_b.fixed_place_object_pose_world == goal.T_world_object_goal
    assert np.allclose(
        plan_a.carry_wrist_pose_world.translation_m[:2],
        plan_b.carry_wrist_pose_world.translation_m[:2],
    )
    assert plan_a.require_precontact_clearance
    assert plan_a.forbid_palm_contact

    other_goal = replace(goal, goal_id="another_drop_zone")
    with pytest.raises(ValueError, match="fixed_goal_id"):
        PoseConditionedShortCanController(scene, other_goal).execute(plan_a)

    unsafe_plan = replace(plan_a, max_penetration_m=0.10)
    with pytest.raises(ValueError, match="uncalibrated_control_limits"):
        execute_pose_conditioned_short_can_plan(scene, unsafe_plan)


@pytest.mark.parametrize(
    "seed,target,source_xy",
    (
        (0, 0, (-0.08, -0.12)),  # 0.42 kg
        (0, 2, (0.11, -0.09)),   # 0.62 kg
        (2, 2, (0.11, -0.09)),   # 0.10 kg
    ),
)
def test_pose_conditioned_short_can_pick_place_closed_loop(
    seed: int, target: int, source_xy
) -> None:
    spec = make_demo_scene("mass", 3, seed)
    scene = _scene(spec)
    probe = ProbeHarness(AllegroHandBackend.create(spec)).execute(
        ProbeCommand("heft", target)
    )
    assert probe.valid, probe.to_dict(include_trace=True)
    request = PoseConditionedPickPlaceRequest(
        _observation(spec, target, source_xy),
        "drop",
        handoff_policy="reset_to_requested_pose",
    )
    controller = PoseConditionedShortCanController(scene, _goal(spec, target))

    decision = controller.plan(probe, request)
    assert decision.executable, decision.to_dict()
    assert decision.plan is not None
    result = controller.execute(decision.plan)

    assert result.success, result.to_dict(include_trace=True)
    assert result.phase_reached == "final_verify"
    assert result.quality["lift_distance_m"] >= 0.020
    assert result.quality["place_error_m"] <= decision.plan.max_place_error_m
    assert result.quality["final_tilt_rad"] <= decision.plan.max_final_tilt_rad
    assert result.quality["final_table_contact"] == 1.0
    assert result.quality["final_hand_contact_group_count"] == 0.0
    assert result.quality["peak_palm_object_force_N"] == 0.0
    assert result.quality["max_grasp_carry_penetration_m"] <= 0.0068
    assert result.params["collision_mode"] == "full_hand"
    assert result.params["selected_grasp_candidate_id"]


def test_verify_live_pose_rejects_a_scene_that_does_not_match_request() -> None:
    spec = make_demo_scene("mass", 3, 0)
    scene = _scene(spec)
    target = 1
    request = PoseConditionedPickPlaceRequest(
        _observation(spec, target, (0.10, -0.12)),
        "drop",
        handoff_policy="verify_live_pose",
    )
    controller = PoseConditionedShortCanController(scene, _goal(spec, target))
    decision = controller.plan(_synthetic_heft(spec, target), request)
    assert decision.executable and decision.plan is not None

    result = controller.execute(decision.plan)
    assert not result.success
    assert result.status == "source_pose_position_mismatch"
    assert result.phase_reached == "handoff"


def test_real_heft_result_flows_into_successful_live_pose_manipulation() -> None:
    spec = make_demo_scene("mass", 3, 2)
    target = 2  # the 0.10 kg can exercises the low end of real heft estimates
    obj = spec.objects[target]
    probe_backend = AllegroHandBackend.create(spec)
    probe_result = ProbeHarness(probe_backend).execute(
        ProbeCommand("heft", target)
    )
    assert probe_result.valid, probe_result.to_dict(include_trace=True)

    scene = _scene(spec)
    scene.set_object_pose(
        target,
        center_position_m=(0.11, -0.09, obj.size[2]),
        record_initial=True,
    )
    scene.step(50)
    live_observation = ObjectPoseObservation(
        target=target,
        object_id=obj.object_id,
        T_world_object=RigidTransform(
            "world",
            obj.object_id,
            tuple(scene.object_center_pos(target)),
            tuple(scene.object_quat(target)),
        ),
        timestamp_s=float(scene.data.time),
    )
    request = PoseConditionedPickPlaceRequest(
        live_observation,
        "drop",
        handoff_policy="verify_live_pose",
    )
    controller = PoseConditionedShortCanController(scene, _goal(spec, target))
    decision = controller.plan(probe_result, request)
    assert decision.executable, decision.to_dict()
    assert decision.plan is not None

    result = controller.execute(decision.plan)
    assert result.success, result.to_dict(include_trace=True)
    assert result.quality["source_position_error_m"] < 0.001
    assert result.quality["place_error_m"] <= decision.plan.max_place_error_m
    assert result.quality["peak_palm_object_force_N"] == 0.0
    assert result.quality["peak_hand_other_object_force_N"] == 0.0
