import numpy as np
import pytest

from allegro_probe import (
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
    "fill": "fill_proxy",
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
    assert stiff.model.geom_contype[stiff.geom["probe_tip_geom"]] != 0
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
            assert result.quality["max_forbidden_penetration_m"] <= 0.00025
            assert result.quality["peak_forbidden_force_N"] == pytest.approx(0.0)
            measured.append(result.features[FEATURE[family]])
            obj = spec.objects[target]
            truth.append(
                {
                    "stiffness": obj.stiffness_N_per_m,
                    "mass": obj.mass_kg,
                    "fill": obj.fill_level,
                    "material": obj.friction_mu,
                }[family]
            )

            if family in {"mass", "fill"}:
                assert result.quality["max_probe_target_penetration_m"] == 0.0
                assert (
                    result.quality["max_hand_target_penetration_m"]
                    <= result.params["penetration_limit"]
                )
                assert result.quality["lift_distance_m"] >= 0.010
                assert result.quality["support_contact_after_lift"] == 0.0
                assert result.quality["table_contact_after_lift"] == 0.0
                assert result.quality["postlift_group_count"] >= 2.0
                assert result.quality["cleanup_new_violation_count"] == 0.0
            else:
                assert result.quality["max_hand_target_penetration_m"] == 0.0
                assert result.quality["max_probe_target_penetration_m"] <= 0.001
            if family == "material":
                assert result.quality["path_completion_ratio"] >= 0.95
                assert result.quality["contact_fraction"] >= 0.80

        if family == "fill":
            assert int(np.argmin(measured)) == int(np.argmin(truth))
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
