import numpy as np
import pytest

from allegro_probe import (
    FEATURE_SCHEMA_VERSION,
    PROBE_PROTOCOL_ID,
    AllegroProbeScene,
    ProbeCommand,
    ProbeHarness,
    SceneConfig,
    make_demo_scene,
    primitive_for_family,
)


BACKENDS = ("reference", "allegro")
FEATURE = {
    "stiffness": "k_est_N_per_m",
    "mass": "m_est_kg",
    "fill": "dynamic_torque_gain_Nm_per_rad",
    "material": "mu_est",
}


def _execute(family: str, backend: str, target: int, seed: int = 0, **params):
    spec = make_demo_scene(family, n_candidates=3, seed=seed)
    scene = AllegroProbeScene(spec, SceneConfig(backend=backend))
    result = ProbeHarness(scene).execute(
        ProbeCommand(primitive_for_family(family), target, params)
    )
    return spec, scene, result


@pytest.mark.parametrize("backend", BACKENDS)
def test_full_pose_and_fixed_collision_roles(backend: str) -> None:
    stiff = AllegroProbeScene(
        make_demo_scene("stiffness", 2, 0), SceneConfig(backend=backend)
    )
    mass = AllegroProbeScene(
        make_demo_scene("mass", 2, 0), SceneConfig(backend=backend)
    )

    for joint in ("wx", "wy", "wz", "wr", "wt", "wyaw"):
        assert f"{joint}_pos" in stiff.sensor_names
        assert f"{joint}_vel" in stiff.sensor_names
    if backend == "reference":
        assert stiff.model.geom_contype[stiff.geom["probe_tip_geom"]] != 0
    else:
        assert stiff.model.geom_contype[stiff.geom["probe_tip_geom"]] == 0
        assert stiff.model.geom_rgba[stiff.geom["probe_tip_geom"], 3] == 0.0
    assert mass.model.geom_contype[mass.geom["probe_tip_geom"]] == 0
    if backend == "reference":
        assert "obj0_pedestal" in mass.geom
    else:
        assert "obj0_pedestal" not in mass.geom
        assert stiff.full_hand_collisions_compiled()
        assert mass.full_hand_collisions_compiled()
    assert not any("cradle" in name for name in mass.geom)

    probe_gid = mass.geom["probe_tip_geom"]
    assert mass.model.geom_rgba[probe_gid, 3] == pytest.approx(0.0)


@pytest.mark.parametrize("backend", BACKENDS)
def test_probe_tip_pose_is_the_physical_capsule_surface(backend: str) -> None:
    scene = AllegroProbeScene(
        make_demo_scene("stiffness", 2, 0), SceneConfig(backend=backend)
    )
    desired_z = 0.120
    extension = 0.037
    scene.data.qpos[scene.joint_qadr["wz"]] = scene.wz_for_tip_z(
        desired_z, extension
    )
    scene.data.qpos[scene.joint_qadr["wp"]] = extension
    scene.data.qpos[scene.joint_qadr["wr"]] = 0.0
    scene.data.qpos[scene.joint_qadr["wt"]] = 0.0
    scene.data.qpos[scene.joint_qadr["wyaw"]] = 0.0
    scene.mujoco.mj_forward(scene.model, scene.data)
    assert scene.probe_tip_pos()[2] == pytest.approx(desired_z, abs=1e-9)


def test_allegro_heft_rejects_partial_collision_model() -> None:
    spec = make_demo_scene("mass", 3, 0)
    scene = AllegroProbeScene(
        spec,
        SceneConfig(
            backend="allegro",
            allegro_grasp_lift=0.0,
            full_hand_collisions=False,
        ),
    )
    result = ProbeHarness(scene).execute(ProbeCommand("heft", 1))
    assert not result.valid
    assert "full_hand_collisions_required" in result.violations
    assert result.features["m_est_kg"] == 0.0


def test_allegro_heft_rejects_contact_with_neighbour_candidate() -> None:
    spec = make_demo_scene("mass", 3, 0)
    scene = AllegroProbeScene(
        spec,
        SceneConfig(backend="allegro", candidate_spacing=0.070),
    )
    result = ProbeHarness(scene).execute(ProbeCommand("heft", 1))
    assert not result.valid
    assert "other_object_collision" in result.violations
    assert result.quality["peak_forbidden_force_N"] > 0.0


def test_content_proxy_has_true_fixed_and_mobile_controls() -> None:
    spec = make_demo_scene("fill", 3, 0)
    scene = AllegroProbeScene(spec, SceneConfig(backend="reference"))
    for obj in spec.objects:
        x_name = f"obj{obj.index}_slosh_x"
        y_name = f"obj{obj.index}_slosh_y"
        if obj.content_mobility_class == "fixed":
            assert x_name not in scene.joint_qadr
            assert y_name not in scene.joint_qadr
        else:
            assert x_name in scene.joint_qadr
            assert y_name in scene.joint_qadr


@pytest.mark.parametrize("backend", BACKENDS)
def test_exhausted_micro_lift_still_runs_guarded_cleanup(backend: str) -> None:
    _, _, result = _execute(
        "mass",
        backend,
        target=1,
        lift_height=0.009,
    )
    assert not result.valid
    assert "object_lift_target_not_reached" in result.violations
    assert result.quality["lift_started"] == 1.0
    assert result.quality["cleanup_new_violation_count"] == 0.0
    assert result.features["m_est_kg"] == 0.0


def test_shake_rejects_unsealed_track_at_admission() -> None:
    spec = make_demo_scene("fill", 3, 0)
    spec.objects[0].container_sealed = False
    scene = AllegroProbeScene(spec, SceneConfig(backend="reference"))
    with pytest.raises(ValueError, match="sealed container"):
        ProbeHarness(scene).execute(ProbeCommand("shake", 0))


def test_shake_cannot_lower_the_protocol_clearance_floor() -> None:
    spec = make_demo_scene("fill", 3, 0)
    scene = AllegroProbeScene(spec, SceneConfig(backend="allegro"))
    initial_wrist = scene.wrist_pos().copy()
    with pytest.raises(ValueError, match=r"\[1.5, 10\] mm"):
        ProbeHarness(scene).execute(
            ProbeCommand("shake", 1, {"minimum_bottom_clearance": 0.001})
        )
    assert not scene.contact_snapshot(1).hand_groups
    assert scene.wrist_pos() == pytest.approx(initial_wrist)


def test_shake_clearance_uses_live_object_geoms_not_wrist_tilt() -> None:
    _, _, result = _execute(
        "fill",
        "allegro",
        target=1,
        minimum_bottom_clearance=0.008,
    )
    assert not result.valid
    assert "object_below_clearance_during_dynamic_baseline" in result.violations
    # The wrist has not begun its tilt input, yet the compliant top pinch has
    # already tilted the object enough for the exact exterior-geom gate to fire.
    assert result.quality["max_actual_tilt_rad"] == pytest.approx(0.0)
    assert result.quality["shake_max_object_axis_tilt_rad"] > 0.10
    assert (
        result.quality["shake_min_bottom_clearance_m"]
        < result.params["minimum_bottom_clearance"]
    )
    assert result.quality["cleanup_new_violation_count"] == 0.0


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize(
    "params, message",
    (
        ({"lift_height": 0.0}, "lift_height"),
        ({"penetration_limit": float("nan")}, "finite"),
        ({"osc_amp": 0.020}, "static protocol"),
    ),
)
def test_invalid_heft_parameters_are_rejected_before_contact(
    backend: str, params, message: str
) -> None:
    spec = make_demo_scene("mass", 3, 0)
    scene = AllegroProbeScene(spec, SceneConfig(backend=backend))
    initial_wrist = scene.wrist_pos().copy()
    with pytest.raises(ValueError, match=message):
        ProbeHarness(scene).execute(ProbeCommand("heft", 1, params))
    snapshot = scene.contact_snapshot(1)
    assert not snapshot.hand_groups
    assert scene.finger_touch_total() == pytest.approx(0.0)
    assert scene.wrist_pos() == pytest.approx(initial_wrist)


@pytest.mark.parametrize("backend", BACKENDS)
def test_static_heft_never_evaluates_deprecated_extreme_frequency(
    backend: str,
) -> None:
    _, _, result = _execute(
        "mass", backend, target=1, osc_freq=1e308
    )
    assert result.valid, result.to_dict()
    assert np.isfinite(result.features["weight_signal_N"])


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("seed", (0, 1, 2, 3, 4))
def test_four_primitives_are_valid_and_physically_ordered(
    backend: str, seed: int
) -> None:
    for family in ("stiffness", "mass", "fill", "material"):
        truth = []
        measured = []
        for target in range(3):
            spec, _, result = _execute(family, backend, target, seed=seed)
            assert result.valid, result.to_dict()
            assert result.controller_status == "ok"
            assert not result.violations
            assert result.phase_reached == "complete"
            assert result.protocol_id == PROBE_PROTOCOL_ID
            assert result.feature_schema == FEATURE_SCHEMA_VERSION
            assert result.mode == result.params["mode"]
            assert result.quality["max_forbidden_penetration_m"] <= 0.00025
            assert result.quality["peak_forbidden_force_N"] == pytest.approx(0.0)
            measured.append(result.features[FEATURE[family]])
            obj = spec.objects[target]
            truth.append(
                {
                    "stiffness": obj.stiffness_N_per_m,
                    "mass": obj.mass_kg,
                    "fill": obj.content_mobility_class,
                    "material": obj.friction_mu,
                }[family]
            )

            if family in {"mass", "fill"}:
                assert result.quality["max_probe_target_penetration_m"] == 0.0
                assert (
                    result.quality["max_hand_target_penetration_m"]
                    <= result.params["penetration_limit"]
                )
                assert result.quality["lift_distance_m"] == pytest.approx(
                    result.params["target_object_lift"],
                    abs=result.params["lift_tolerance"],
                )
                assert (
                    result.quality["support_free_dwell_s"]
                    >= result.params["support_loss_dwell"]
                )
                assert result.quality["support_contact_after_lift"] == 0.0
                assert result.quality["table_contact_after_lift"] == 0.0
                assert result.quality["postlift_group_count"] >= 2.0
                assert result.quality["cleanup_new_violation_count"] == 0.0
                if family == "mass":
                    assert (
                        result.quality["measurement_min_object_lift_m"]
                        >= result.params["target_object_lift"]
                        - result.params["lift_tolerance"]
                    )
                    assert (
                        result.quality["measurement_max_object_lift_m"]
                        <= result.params["max_object_lift"]
                    )
                else:
                    assert (
                        result.quality["shake_min_bottom_clearance_m"]
                        >= result.params["minimum_bottom_clearance"]
                    )
                    assert result.quality["shake_min_geometric_margin_m"] >= 0.0
                    assert (
                        result.quality["shake_max_object_lift_m"]
                        <= result.params["max_object_lift"]
                    )
                    assert (
                        result.quality["shake_max_wrist_lift_command_m"]
                        <= result.params["lift_height"] + 1e-9
                    )
                    assert result.quality["analysis_wrist_z_command_span_m"] == 0.0
            elif family == "stiffness" and backend == "allegro":
                assert result.params["effector"] == "ff_tip"
                assert result.params["sensor"] == "ff_tip_touch"
                assert result.quality["max_probe_target_penetration_m"] == 0.0
                assert result.quality["max_hand_target_penetration_m"] <= 0.0005
                assert result.quality["peak_hand_target_force_N"] <= 1.0
            else:
                assert result.quality["max_hand_target_penetration_m"] == 0.0
                assert result.quality["max_probe_target_penetration_m"] <= 0.001
            if family == "material":
                assert result.quality["path_completion_ratio"] >= 0.95
                assert result.quality["outbound_path_completion_ratio"] >= 0.95
                assert result.quality["return_path_completion_ratio"] >= 0.95
                assert result.quality["contact_fraction"] >= 0.80
            if family == "fill":
                assert result.features["angle_tracking_ratio"] >= 0.75
                assert result.quality["final_tilt_error_rad"] <= np.deg2rad(0.5)
                assert obj.container_sealed
                assert obj.mass_kg == pytest.approx(0.30)

        if family == "fill":
            # Direction is backend-specific because the raw response still
            # contains the gripper/container rigid-body transfer function.
            # Calibration may invert it, but the three equal-mass mobility
            # controls must remain individually separable.
            by_class = dict(zip(truth, measured))
            assert set(by_class) == {"fixed", "damped", "mobile"}
            assert min(
                abs(by_class[a] - by_class[b])
                for a, b in (("fixed", "damped"), ("damped", "mobile"))
            ) > 1e-4
        else:
            assert list(np.argsort(measured)) == list(np.argsort(truth))


@pytest.mark.parametrize("backend", BACKENDS)
def test_failed_grasp_never_produces_trusted_heft_features(backend: str) -> None:
    _, _, result = _execute(
        "mass",
        backend,
        target=0,
        min_grasp_force=1_000.0,
    )
    assert not result.valid
    assert result.controller_status != "ok"
    assert result.violations
    assert result.features["m_est_kg"] == 0.0
    assert result.features["lifted"] == 0.0


@pytest.mark.parametrize("backend", BACKENDS)
def test_shake_cannot_bypass_failed_heft_gate(backend: str) -> None:
    _, _, result = _execute(
        "fill",
        backend,
        target=0,
        min_grasp_force=1_000.0,
    )
    assert not result.valid
    assert "heft_invalid" in result.violations
    assert result.features["fill_proxy"] == 0.0
    assert result.features["slosh_proxy"] == 0.0


@pytest.mark.parametrize("backend", BACKENDS)
def test_incomplete_slide_is_not_relabelled_success(backend: str) -> None:
    _, _, result = _execute(
        "material",
        backend,
        target=1,
        distance=0.20,
        recovery_steps=5,
    )
    assert not result.valid
    assert result.controller_status != "ok"
    assert result.features["mu_est"] == 0.0
