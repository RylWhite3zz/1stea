import hashlib
from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from allegro_probe.franka_scene import (
    CANONICAL_ALLEGRO_OPEN,
    CANONICAL_PANDA_HOME,
    CANONICAL_Q,
    FRANKA_ALLEGRO_BACKEND,
    HAND_ACTUATOR_NAMES,
    HAND_JOINT_NAMES,
    PANDA_ACTUATOR_NAMES,
    PANDA_JOINT_NAMES,
    FrankaAllegroScene,
)
from allegro_probe.models import FRANKA_ALLEGRO_MUJOCO_BACKEND


@pytest.fixture(scope="module")
def scene() -> FrankaAllegroScene:
    return FrankaAllegroScene()


def test_stage1_model_compiles_as_independent_7_plus_16_scene(
    scene: FrankaAllegroScene,
) -> None:
    assert FRANKA_ALLEGRO_BACKEND == FRANKA_ALLEGRO_MUJOCO_BACKEND
    assert scene.config.backend == FRANKA_ALLEGRO_MUJOCO_BACKEND
    assert scene.mount_profile.profile_id == "sim.synthetic_panda_allegro_mount.v1"
    assert (scene.model.nq, scene.model.nv, scene.model.nu) == (23, 23, 23)

    joint_names = tuple(
        scene.model.joint(index).name for index in range(scene.model.njnt)
    )
    actuator_names = tuple(
        scene.model.actuator(index).name for index in range(scene.model.nu)
    )
    assert joint_names == PANDA_JOINT_NAMES + HAND_JOINT_NAMES
    assert actuator_names == PANDA_ACTUATOR_NAMES + HAND_ACTUATOR_NAMES
    for geom_id in range(scene.model.ngeom):
        if (
            int(scene.model.geom_contype[geom_id])
            or int(scene.model.geom_conaffinity[geom_id])
        ):
            assert scene.model.geom(geom_id).name, (
                f"collision geom {geom_id} must have a stable audit name"
            )

    provenance = scene.model_provenance
    assert provenance.menagerie_root == scene.config.menagerie_root.resolve()
    assert provenance.panda_xml_sha256 == hashlib.sha256(
        provenance.panda_xml_path.read_bytes()
    ).hexdigest()
    assert provenance.allegro_xml_sha256 == hashlib.sha256(
        provenance.allegro_xml_path.read_bytes()
    ).hexdigest()
    assert len(provenance.panda_xml_sha256) == 64
    assert len(provenance.allegro_xml_sha256) == 64
    assert provenance.mujoco_version == scene.mujoco.__version__
    assert provenance.timestep_s == pytest.approx(scene.dt)
    assert provenance.cone == "elliptic"
    assert provenance.mount_profile_id == scene.mount_profile.profile_id
    assert "panda_menagerie_pd+allegro_position" in provenance.controller_profile_id
    assert provenance.hand_kp == pytest.approx(scene.config.hand_kp)
    with pytest.raises(FrozenInstanceError):
        provenance.hand_kp = 99.0  # type: ignore[misc]

    # Stage 1 is a new robot model, not the old carriage with different labels.
    all_names = {
        *(scene.model.body(index).name for index in range(scene.model.nbody)),
        *(scene.model.joint(index).name for index in range(scene.model.njnt)),
        *(scene.model.actuator(index).name for index in range(scene.model.nu)),
        *(scene.model.site(index).name for index in range(scene.model.nsite)),
        *(scene.model.geom(index).name for index in range(scene.model.ngeom)),
    }
    assert not {
        "wx", "wy", "wz", "wr", "wt", "wyaw", "wp",
        "act_wx", "act_wy", "act_wz", "act_wr", "act_wt", "act_wyaw",
        "probe_tip_geom", "probe_tip_site", "central_probe",
    } & all_names


def test_model_assets_are_independent_of_current_working_directory(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.chdir(tmp_path)
    rebuilt = FrankaAllegroScene()
    assert (rebuilt.model.nq, rebuilt.model.nv, rebuilt.model.nu) == (23, 23, 23)


def test_canonical_keyframe_is_explicit_legal_and_stable(
    scene: FrankaAllegroScene,
) -> None:
    scene.reset()
    np.testing.assert_allclose(scene.data.qpos, CANONICAL_Q, atol=0.0, rtol=0.0)
    np.testing.assert_allclose(scene.data.ctrl, CANONICAL_Q, atol=0.0, rtol=0.0)
    np.testing.assert_allclose(scene.arm_qpos, CANONICAL_PANDA_HOME)
    np.testing.assert_allclose(scene.hand_qpos, CANONICAL_ALLEGRO_OPEN)

    assert np.all(scene.model.jnt_limited)
    assert np.all(CANONICAL_Q >= scene.model.jnt_range[:, 0])
    assert np.all(CANONICAL_Q <= scene.model.jnt_range[:, 1])
    assert np.all(scene.model.actuator_ctrllimited)
    assert np.all(CANONICAL_Q >= scene.model.actuator_ctrlrange[:, 0])
    assert np.all(CANONICAL_Q <= scene.model.actuator_ctrlrange[:, 1])
    # The invalid raw Allegro zero pose must never leak into the thumb base.
    thumb_id = scene.model.joint("hand/thj0").id
    assert CANONICAL_ALLEGRO_OPEN[12] == pytest.approx(0.45)
    assert scene.hand_qpos[12] >= scene.model.jnt_range[thumb_id, 0]

    key_id = scene.model.key("home").id
    np.testing.assert_allclose(scene.model.key_qpos[key_id], CANONICAL_Q)
    np.testing.assert_allclose(scene.model.key_ctrl[key_id], CANONICAL_Q)

    # A half-second controlled settle is part of the canonical collision gate.
    scene.step(250)
    assert np.all(np.isfinite(scene.data.qpos))
    assert np.all(np.isfinite(scene.data.qvel))
    assert not scene.collision_snapshot().has_self_collision
    assert not scene.distance_audit().has_forbidden_penetration


def test_canonical_collision_gate_includes_welded_arm_hand_distance(
    scene: FrankaAllegroScene,
) -> None:
    scene.reset()
    snapshot = scene.collision_snapshot()
    audit = scene.distance_audit(max_distance_m=0.100)
    assert snapshot.contacts == ()
    assert not snapshot.has_self_collision
    assert not audit.has_forbidden_penetration

    link7 = scene.model.body("link7").id
    palm = scene.model.body("hand/palm").id
    # MuJoCo contact filtering welds this fixed chain.  The distance gate must
    # nevertheless inspect link7/palm, otherwise a bad direct mount could pass
    # solely because data.ncon is zero.
    assert scene.model.body_weldid[link7] == scene.model.body_weldid[palm]
    arm_palm_pairs = [
        pair
        for pair in audit.pairs
        if {pair.body1, pair.body2} == {"link7", "hand/palm"}
    ]
    assert len(arm_palm_pairs) == 1
    assert not arm_palm_pairs[0].policy_filtered
    assert arm_palm_pairs[0].signed_distance_m > 0.020

    mount_palm_pairs = [
        pair
        for pair in audit.pairs
        if {pair.body1, pair.body2} == {"synthetic_mount", "hand/palm"}
    ]
    assert len(mount_palm_pairs) == 1
    assert mount_palm_pairs[0].policy_filter_reason == "synthetic_mount_interface"
    assert mount_palm_pairs[0].signed_distance_m > 0.0

    # link7 -> attachment -> synthetic_mount is another two-hop welded pair.
    # It remains a real audit pair, not a global "near ancestor" exemption.
    arm_mount_pairs = [
        pair
        for pair in audit.pairs
        if {pair.body1, pair.body2} == {"link7", "synthetic_mount"}
    ]
    assert len(arm_mount_pairs) == 1
    assert not arm_mount_pairs[0].policy_filtered
    assert arm_mount_pairs[0].signed_distance_m > 0.003

    # MuJoCo 3.10 reports raw distance 0 for this separated box pair but fills
    # the closest-point endpoints.  The audit normalizes that API quirk to the
    # positive endpoint separation instead of falsely reporting contact.
    separated_box_pairs = [
        pair
        for pair in audit.pairs
        if {pair.body1, pair.body2}
        == {"hand/ff_proximal", "hand/rf_base"}
    ]
    assert len(separated_box_pairs) == 1
    assert not separated_box_pairs[0].policy_filtered
    assert separated_box_pairs[0].signed_distance_m == pytest.approx(
        0.06221, abs=1e-4
    )
    assert audit.minimum_forbidden_distance_m > 0.003


def test_fk_attachment_matches_source_panda_at_multiple_legal_q7(
    scene: FrankaAllegroScene,
) -> None:
    mj = scene.mujoco
    source_path = (
        scene.config.menagerie_root
        / "franka_emika_panda"
        / "panda_nohand.xml"
    )
    source_model = mj.MjModel.from_xml_path(str(source_path))
    source_data = mj.MjData(source_model)
    source_site = source_model.site("attachment_site").id
    profile = scene.mount_profile
    mount_rotation = np.empty(9, dtype=float)
    mj.mju_quat2Mat(
        mount_rotation,
        np.asarray(profile.attachment_to_palm_quaternion_wxyz, dtype=float),
    )
    mount_rotation = mount_rotation.reshape(3, 3)
    legal_arm_poses = (
        CANONICAL_PANDA_HOME,
        np.array([0.15, -0.30, 0.20, -1.70, 0.10, 1.30, -0.50]),
        np.array([-0.20, 0.25, -0.15, -1.20, -0.20, 2.00, 0.30]),
    )
    for q7 in legal_arm_poses:
        assert np.all(q7 >= source_model.jnt_range[:, 0])
        assert np.all(q7 <= source_model.jnt_range[:, 1])
        source_data.qpos[:] = q7
        mj.mj_forward(source_model, source_data)
        scene.reset()
        scene.data.qpos[scene.arm_qpos_indices] = q7
        scene.data.qpos[scene.hand_qpos_indices] = CANONICAL_ALLEGRO_OPEN
        mj.mj_forward(scene.model, scene.data)

        attachment = scene.frame_pose("attachment")
        np.testing.assert_allclose(
            attachment.position_m, source_data.site_xpos[source_site], atol=1e-12
        )
        np.testing.assert_allclose(
            attachment.rotation_matrix,
            source_data.site_xmat[source_site].reshape(3, 3),
            atol=1e-12,
        )
        expected_palm_position = (
            attachment.position_m
            + attachment.rotation_matrix
            @ np.asarray(profile.attachment_to_palm_position_m)
        )
        expected_palm_rotation = attachment.rotation_matrix @ mount_rotation

        for frame_name in ("mount_palm", "allegro_palm", "protocol_wrist"):
            pose = scene.frame_pose(frame_name)
            np.testing.assert_allclose(
                pose.position_m, expected_palm_position, atol=1e-12
            )
            np.testing.assert_allclose(
                pose.rotation_matrix, expected_palm_rotation, atol=1e-12
            )

    expected_tip_bodies = {
        "ff_tip": "hand/ff_tip",
        "mf_tip": "hand/mf_tip",
        "rf_tip": "hand/rf_tip",
        "th_tip": "hand/th_tip",
    }
    hand_source_path = (
        scene.config.menagerie_root / "wonik_allegro" / "right_hand.xml"
    )
    hand_source_model = mj.MjModel.from_xml_path(str(hand_source_path))
    hand_source_data = mj.MjData(hand_source_model)
    hand_source_data.qpos[:] = CANONICAL_ALLEGRO_OPEN
    mj.mj_forward(hand_source_model, hand_source_data)
    source_palm_id = hand_source_model.body("palm").id
    source_palm_position = hand_source_data.xpos[source_palm_id]
    source_palm_rotation = hand_source_data.xmat[source_palm_id].reshape(3, 3)
    merged_palm = scene.frame_pose("allegro_palm")
    for frame_name, body_name in expected_tip_bodies.items():
        pose = scene.frame_pose(frame_name)
        assert pose.position_m.shape == (3,)
        assert pose.rotation_matrix.shape == (3, 3)
        assert np.linalg.det(pose.rotation_matrix) == pytest.approx(1.0, abs=1e-12)
        site_id = scene.model.site(f"hand/{frame_name}_site").id
        assert scene.model.body(scene.model.site_bodyid[site_id]).name == body_name

        # Independent hand-chain oracle: recover palm<-tip-body from the
        # untouched Allegro MJCF, then append the merged site's explicit local
        # offset.  This catches prefix/attachment or fingertip-site mistakes.
        source_tip_id = hand_source_model.body(frame_name).id
        source_tip_position = hand_source_data.xpos[source_tip_id]
        source_tip_rotation = hand_source_data.xmat[source_tip_id].reshape(3, 3)
        palm_to_tip_position = source_palm_rotation.T @ (
            source_tip_position - source_palm_position
        )
        palm_to_tip_rotation = source_palm_rotation.T @ source_tip_rotation
        expected_tip_rotation = (
            merged_palm.rotation_matrix @ palm_to_tip_rotation
        )
        expected_site_position = (
            merged_palm.position_m
            + merged_palm.rotation_matrix @ palm_to_tip_position
            + expected_tip_rotation @ scene.model.site_pos[site_id]
        )
        np.testing.assert_allclose(
            pose.position_m, expected_site_position, atol=1e-12
        )
        np.testing.assert_allclose(
            pose.rotation_matrix, expected_tip_rotation, atol=1e-12
        )

    with pytest.raises(KeyError, match="unknown frame"):
        scene.frame_pose("not_a_robot_frame")


@pytest.mark.parametrize("direction", (-1.0, 1.0))
def test_all_7_plus_16_actuators_move_independently(
    scene: FrankaAllegroScene,
    direction: float,
) -> None:
    delta = 0.010 * direction
    groups = (
        (
            "arm",
            CANONICAL_PANDA_HOME,
            scene.arm_qpos_indices,
            scene.arm_actuator_ids,
            scene.hand_actuator_ids,
            scene.command_arm_joints,
        ),
        (
            "hand",
            CANONICAL_ALLEGRO_OPEN,
            scene.hand_qpos_indices,
            scene.hand_actuator_ids,
            scene.arm_actuator_ids,
            scene.command_hand_joints,
        ),
    )
    for label, canonical, qpos_indices, actuator_ids, other_ids, command in groups:
        for local_index in range(len(canonical)):
            scene.reset()
            qpos_before = scene.data.qpos.copy()
            ctrl_before = scene.data.ctrl.copy()
            target = np.asarray(canonical).copy()
            target[local_index] += delta

            returned_target = command(target)
            np.testing.assert_allclose(returned_target, target)
            np.testing.assert_allclose(scene.data.ctrl[actuator_ids], target)
            np.testing.assert_allclose(
                scene.data.ctrl[other_ids], ctrl_before[other_ids], atol=0.0, rtol=0.0
            )
            changed = np.flatnonzero(
                scene.data.ctrl[actuator_ids] != ctrl_before[actuator_ids]
            )
            assert changed.tolist() == [local_index]

            scene.step(40)
            movement = (
                scene.data.qpos[qpos_indices[local_index]]
                - qpos_before[qpos_indices[local_index]]
            )
            assert direction * movement > 1e-3, (
                f"{label} actuator {local_index} did not move in command direction"
            )
            assert np.all(np.isfinite(scene.data.qpos))
            assert np.all(np.isfinite(scene.data.qvel))
            assert not scene.collision_snapshot().has_self_collision
            assert not scene.distance_audit(
                max_distance_m=0.005
            ).has_forbidden_penetration


def test_joint_commands_reject_bad_shape_nonfinite_and_out_of_range(
    scene: FrankaAllegroScene,
) -> None:
    with pytest.raises(ValueError, match="shape"):
        scene.command_arm_joints(np.zeros(6))
    invalid_hand = CANONICAL_ALLEGRO_OPEN.copy()
    invalid_hand[3] = np.nan
    with pytest.raises(ValueError, match="finite"):
        scene.command_hand_joints(invalid_hand)
    invalid_arm = CANONICAL_PANDA_HOME.copy()
    invalid_arm[3] = 0.0
    with pytest.raises(ValueError, match="outside actuator range"):
        scene.command_arm_joints(invalid_arm)
    with pytest.raises(ValueError, match="positive integer"):
        scene.step(0)
