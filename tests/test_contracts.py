import pytest
import numpy as np
import inspect

from allegro_probe import (
    FEATURE_SCHEMA_VERSION,
    PROBE_PROTOCOL_ID,
    ProbeCommand,
    ObjectSpec,
    make_demo_scene,
    primitive_for_family,
)
from allegro_probe.protocols import canonical_probe_mode, validate_protocol_id
from allegro_probe.primitives import heft, shake


def test_family_to_primitive_contract() -> None:
    assert primitive_for_family("stiffness") == "poke"
    assert primitive_for_family("mass") == "heft"
    assert primitive_for_family("fill") == "shake"
    assert primitive_for_family("material") == "slide"


def test_demo_scene_has_no_answer_field() -> None:
    scene = make_demo_scene("mass", n_candidates=3, seed=0)
    payload = scene.to_dict(reveal_hidden=False)
    assert "target" not in payload
    assert len(payload["objects"]) == 3
    assert ProbeCommand("heft", target=1).target == 1


def test_versioned_modes_reject_semantic_fallback() -> None:
    assert canonical_probe_mode("heft") == "unsupported_micro_lift"
    assert canonical_probe_mode("shake") == "unsupported_micro_shake"
    assert canonical_probe_mode("slide") == "round_trip_force_control"
    with pytest.raises(ValueError, match="supported_nudge"):
        canonical_probe_mode("heft", "supported_nudge")
    with pytest.raises(ValueError, match="supported_tilt"):
        canonical_probe_mode("shake", "supported_tilt")
    assert validate_protocol_id(PROBE_PROTOCOL_ID) == PROBE_PROTOCOL_ID
    assert FEATURE_SCHEMA_VERSION.endswith(".v2")


def test_content_mobility_demo_is_mass_balanced() -> None:
    scene = make_demo_scene("fill", n_candidates=3, seed=0)
    assert scene.track == "content_mobility"
    assert {obj.content_mobility_class for obj in scene.objects} == {
        "fixed",
        "damped",
        "mobile",
    }
    assert len({obj.mass_kg for obj in scene.objects}) == 1
    assert len({obj.fill_level for obj in scene.objects}) == 1
    assert len({obj.liquid_mass_kg for obj in scene.objects}) == 1
    assert len({obj.slosh_range_m for obj in scene.objects}) == 1


def test_legacy_fill_object_keeps_the_old_mobile_proxy() -> None:
    obj = ObjectSpec(
        index=0,
        family="fill",
        shape="opaque_cup",
        size=(0.034, 0.034, 0.045),
        mass_kg=0.30,
        fill_level=0.5,
        liquid_mass_kg=0.16,
        slosh_range_m=0.01,
    )
    omega = 2.0 * np.pi * obj.slosh_natural_frequency_Hz
    assert obj.content_mobility_class == "mobile"
    assert obj.content_proxy_version == "legacy_xy_msd.v1"
    assert obj.liquid_mass_kg * omega * omega == pytest.approx(4.0)
    assert (
        2.0 * obj.slosh_damping_ratio * obj.liquid_mass_kg * omega
        == pytest.approx(0.8)
    )


def test_legacy_positional_primitive_parameters_keep_their_order() -> None:
    assert list(inspect.signature(heft).parameters)[:8] == [
        "executor",
        "target",
        "lift_height",
        "hold_time",
        "osc_amp",
        "osc_freq",
        "penetration_limit",
        "min_grasp_force",
    ]
    assert list(inspect.signature(shake).parameters)[:9] == [
        "executor",
        "target",
        "lift_height",
        "tilt_amp",
        "yaw_amp",
        "freq",
        "duration",
        "penetration_limit",
        "min_grasp_force",
    ]


def test_content_mobility_typos_and_incomplete_mobile_proxy_are_rejected() -> None:
    common = dict(
        index=0,
        family="fill",
        shape="opaque_cup",
        size=(0.034, 0.034, 0.045),
        mass_kg=0.30,
        liquid_mass_kg=0.16,
    )
    with pytest.raises(ValueError, match="content_mobility_class"):
        ObjectSpec(**common, content_mobility_class="fixd")
    with pytest.raises(ValueError, match="positive slosh"):
        ObjectSpec(**common, content_mobility_class="mobile")
