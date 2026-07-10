from dataclasses import replace

import numpy as np
import pytest

from allegro_probe import (
    AllegroHandBackend,
    ProbeCommand,
    ProbeHarness,
    ProbeResult,
    ReferenceProbeBackend,
    ShortCanPickPlaceRequest,
    build_short_can_pick_place_plan,
    execute_short_can_pick_place,
    make_demo_scene,
    short_can_hand_template,
)


def _synthetic_heft(target: int, *, mass_kg: float, weight_N: float) -> ProbeResult:
    return ProbeResult(
        object_id=f"obj{target}",
        target=target,
        primitive="heft",
        backend="allegro",
        valid=True,
        features={
            "m_est_kg": mass_kg,
            "weight_signal_N": weight_N,
        },
    )


def _legacy_v1_backend(spec):
    return AllegroHandBackend.create(
        spec,
        allegro_grasp_lift=0.090,
        full_hand_collisions=False,
        wrist_roll_limit_rad=0.9,
    )


def test_plan_admission_and_probe_conditioning() -> None:
    backend = _legacy_v1_backend(make_demo_scene("mass", 3, 0))
    scene = backend.scene
    light = _synthetic_heft(0, mass_kg=0.04, weight_N=0.4)
    heavy = _synthetic_heft(0, mass_kg=0.45, weight_N=4.4)

    light_decision = build_short_can_pick_place_plan(
        scene, light, ShortCanPickPlaceRequest(0)
    )
    heavy_decision = build_short_can_pick_place_plan(
        scene, heavy, ShortCanPickPlaceRequest(0)
    )
    assert light_decision.executable and light_decision.plan is not None
    assert heavy_decision.executable and heavy_decision.plan is not None
    assert (
        heavy_decision.plan.target_total_normal_force_N
        > light_decision.plan.target_total_normal_force_N
    )
    assert (
        heavy_decision.plan.max_wrist_speed_mps
        < light_decision.plan.max_wrist_speed_mps
    )
    assert not light_decision.plan.post_descent_xy_correction
    assert heavy_decision.plan.post_descent_xy_correction
    assert light_decision.plan.use_gravity_settle
    assert not heavy_decision.plan.use_gravity_settle

    invalid = replace(light, valid=False, violations=["probe_invalid"])
    rejected = build_short_can_pick_place_plan(
        scene, invalid, ShortCanPickPlaceRequest(0)
    )
    assert not rejected.executable
    assert rejected.reason == "probe_invalid"

    no_reset = build_short_can_pick_place_plan(
        scene,
        light,
        ShortCanPickPlaceRequest(0, reset_before_execute=False),
    )
    assert not no_reset.executable
    assert no_reset.reason == "canonical_reset_required"

    wrong_place = build_short_can_pick_place_plan(
        scene,
        light,
        ShortCanPickPlaceRequest(0, place_offset_xy_m=(0.0, 0.10)),
    )
    assert not wrong_place.executable
    assert wrong_place.reason == "fixed_place_offset_required"

    reference = ReferenceProbeBackend.create(make_demo_scene("mass", 3, 0))
    wrong_backend = build_short_can_pick_place_plan(
        reference.scene,
        replace(light, backend="reference"),
        ShortCanPickPlaceRequest(0),
    )
    assert not wrong_backend.executable
    assert wrong_backend.reason == "allegro_backend_required"

    safe_default = AllegroHandBackend.create(
        make_demo_scene("mass", 3, 0)
    ).scene
    legacy_rejected = build_short_can_pick_place_plan(
        safe_default,
        light,
        ShortCanPickPlaceRequest(0),
    )
    assert not legacy_rejected.executable
    assert legacy_rejected.reason == "legacy_distal_only_collision_required"
    with pytest.raises(ValueError, match="distal-only"):
        execute_short_can_pick_place(safe_default, light_decision.plan)


def test_allegro_template_exposes_explicit_16_dof_targets() -> None:
    scene = _legacy_v1_backend(make_demo_scene("mass", 3, 0)).scene
    template = short_can_hand_template(scene)
    assert template.q_open.shape == (16,)
    assert template.q_preshape.shape == (16,)
    assert template.q_contact.shape == (16,)
    assert template.q_squeeze_limit.shape == (16,)
    assert not np.allclose(template.q_preshape, template.q_squeeze_limit)

    scene.command_allegro_joints(template.q_contact)
    assert np.allclose(scene.allegro_joint_targets(), template.q_contact)
    with pytest.raises(ValueError):
        scene.command_allegro_joints(np.zeros(15))

    scene.set_allegro_position_kp(0.5)
    assert scene.model.actuator_gainprm[scene.act["ffa0"], 0] == pytest.approx(0.5)
    scene.set_allegro_position_kp(8.0)


@pytest.mark.parametrize("seed", (0, 1, 2))
@pytest.mark.parametrize("target", (0, 1, 2))
def test_short_can_pick_place_closed_loop(seed: int, target: int) -> None:
    spec = make_demo_scene("mass", 3, seed)
    probe_backend = AllegroHandBackend.create(spec)
    probe_result = ProbeHarness(probe_backend).execute(ProbeCommand("heft", target))
    assert probe_result.valid, probe_result.to_dict()
    backend = _legacy_v1_backend(spec)
    scene = backend.scene

    decision = build_short_can_pick_place_plan(
        scene,
        probe_result,
        ShortCanPickPlaceRequest(target),
    )
    assert decision.executable, decision.to_dict()
    assert decision.plan is not None
    plan = decision.plan
    result = execute_short_can_pick_place(scene, plan)

    assert result.success, result.to_dict(include_trace=True)
    assert result.status == "ok"
    assert result.phase_reached == "final_verify"
    assert not result.violations
    assert result.quality["lift_distance_m"] >= 0.020
    assert result.quality["carry_distance_m"] >= plan.min_carry_distance_m
    assert result.quality["place_error_m"] <= plan.max_place_error_m
    assert result.quality["final_tilt_rad"] <= plan.max_final_tilt_rad
    assert result.quality["final_drift_m"] <= plan.max_final_drift_m
    assert result.quality["final_table_contact"] == 1.0
    assert result.quality["final_hand_contact_group_count"] == 0.0
    assert result.quality["final_hand_table_contact"] == 0.0
    assert (
        result.quality["max_grasp_carry_penetration_m"]
        <= plan.max_penetration_m
    )
    assert (
        result.quality["max_place_release_penetration_m"]
        <= plan.max_place_penetration_m
    )
    assert (
        result.quality["peak_grasp_carry_force_N"]
        <= plan.max_total_normal_force_N
    )
    assert (
        result.quality["peak_place_release_force_N"]
        <= plan.max_place_normal_force_N
    )
    assert (
        result.quality["peak_hand_table_force_N"]
        <= plan.max_hand_table_force_N
    )
    assert scene.model.actuator_gainprm[scene.act["ffa0"], 0] == pytest.approx(8.0)
