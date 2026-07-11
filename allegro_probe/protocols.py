"""Versioned v1 probe semantics and validated action modes.

This module freezes execution semantics only.  Dataset construction, scoring,
calibration policy, and leaderboard rules remain ProbeBench responsibilities.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping


PROBE_PROTOCOL_ID = "probebench.probe.v1"
FEATURE_SCHEMA_VERSION = "allegro_probe.features.v2"


DEFAULT_PROBE_MODES: Mapping[str, str] = MappingProxyType(
    {
        "poke": "normal_force_ramp",
        "heft": "unsupported_micro_lift",
        "shake": "unsupported_micro_shake",
        "slide": "round_trip_force_control",
    }
)

ALLOWED_PROBE_MODES: Mapping[str, frozenset[str]] = MappingProxyType(
    {
        "poke": frozenset({"normal_force_ramp"}),
        # supported_nudge is deliberately not accepted: it cannot produce the
        # unsupported gravity signal required by the v1 mass protocol.
        "heft": frozenset({"unsupported_micro_lift"}),
        # supported_tilt needs a separately modelled pivot/support baseline and
        # is therefore not silently substituted for the support-free protocol.
        "shake": frozenset({"unsupported_micro_shake"}),
        "slide": frozenset({"round_trip_force_control"}),
    }
)


@dataclass(frozen=True)
class ProbeProtocolDefaults:
    """Calibrated starting envelope for the local v1 execution layer."""

    poke_target_force_N: float = 3.0
    poke_force_limit_N: float = 10.0
    poke_max_depth_m: float = 0.006
    allegro_poke_target_force_N: float = 0.8
    allegro_poke_force_limit_N: float = 1.0
    allegro_poke_max_depth_m: float = 0.0018
    allegro_poke_hold_s: float = 0.2
    micro_lift_target_m: float = 0.008
    micro_lift_tolerance_m: float = 0.0015
    micro_lift_speed_m_per_s: float = 0.020
    micro_lift_max_wrist_travel_m: float = 0.035
    max_object_lift_m: float = 0.015
    support_loss_dwell_s: float = 0.120
    micro_lift_stable_dwell_s: float = 0.080
    shake_lift_stable_dwell_s: float = 0.200
    heft_hold_s: float = 0.200
    shake_tilt_amplitude_rad: float = 0.05235987755982989  # 3 degrees
    shake_yaw_amplitude_rad: float = 0.0
    shake_frequency_Hz: float = 3.0
    shake_duration_s: float = 0.5
    shake_geometric_margin_m: float = 0.00050
    # Authoritative live safety gate: exact exterior collision geometry must
    # remain this far above its source-support height throughout the waveform.
    shake_min_bottom_clearance_m: float = 0.00150
    # Calibrated allowance for compliant-grasp settling over the complete
    # ramp/analysis/return waveform. It raises the planned target; it does not
    # relax the live geometric gate.
    shake_dynamic_sag_reserve_m: float = 0.00150
    slide_preload_N: float = 2.0
    slide_one_way_distance_m: float = 0.040
    slide_one_way_duration_s: float = 0.8


V1_DEFAULTS = ProbeProtocolDefaults()


def canonical_probe_mode(primitive: str, mode: str | None = None) -> str:
    """Return the canonical mode or reject cross-protocol semantic fallback."""

    primitive = str(primitive)
    if primitive not in DEFAULT_PROBE_MODES:
        raise ValueError(
            f"unknown probe primitive {primitive!r}; "
            f"expected {sorted(DEFAULT_PROBE_MODES)}"
        )
    value = DEFAULT_PROBE_MODES[primitive] if mode is None else str(mode)
    if value not in ALLOWED_PROBE_MODES[primitive]:
        raise ValueError(
            f"unsupported mode {value!r} for {primitive!r}; "
            f"expected {sorted(ALLOWED_PROBE_MODES[primitive])}"
        )
    return value


def validate_protocol_id(protocol_id: str) -> str:
    value = str(protocol_id)
    if value != PROBE_PROTOCOL_ID:
        raise ValueError(
            f"unsupported protocol_id {value!r}; expected {PROBE_PROTOCOL_ID!r}"
        )
    return value
