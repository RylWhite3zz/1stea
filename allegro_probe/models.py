"""Data contracts shared by the simulator and probe primitives."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Tuple

import numpy as np

from allegro_probe.protocols import FEATURE_SCHEMA_VERSION, PROBE_PROTOCOL_ID


FAMILIES = ("stiffness", "mass", "fill", "material")
FAMILY_ALIASES = {"smoothness": "material"}

# ``AllegroProbeScene`` implements the original carriage-based simulator only.
# Keep that narrower set separate from the registered execution backends so a
# newly registered robot embodiment cannot accidentally fall through one of
# the carriage scene's ``reference`` branches.
CARRIAGE_BACKENDS = ("reference", "allegro")
FRANKA_ALLEGRO_MUJOCO_BACKEND = "franka_allegro_mujoco"
BACKENDS = (*CARRIAGE_BACKENDS, FRANKA_ALLEGRO_MUJOCO_BACKEND)


def canonical_family(family: str) -> str:
    value = FAMILY_ALIASES.get(str(family), str(family))
    if value not in FAMILIES:
        raise ValueError(
            f"family must be one of {FAMILIES} or smoothness, got {family!r}"
        )
    return value


@dataclass
class ObjectSpec:
    """One visually matched object with hidden simulation parameters."""

    index: int
    family: str
    shape: str
    size: Tuple[float, float, float]
    mass_kg: float
    stiffness_N_per_m: float = 500.0
    friction_mu: float = 0.8
    fill_level: float = 0.0
    liquid_mass_kg: float = 0.0
    slosh_range_m: float = 0.0
    rgba: Tuple[float, float, float, float] = (0.55, 0.55, 0.58, 1.0)
    # Appended fields preserve the positional ABI of the original ObjectSpec.
    container_sealed: bool = False
    content_mobility_class: str = "none"
    slosh_natural_frequency_Hz: float = 0.0
    slosh_damping_ratio: float = 1.0
    content_proxy_version: str = "none"

    def __post_init__(self) -> None:
        numeric = {
            "mass_kg": self.mass_kg,
            "stiffness_N_per_m": self.stiffness_N_per_m,
            "friction_mu": self.friction_mu,
            "fill_level": self.fill_level,
            "liquid_mass_kg": self.liquid_mass_kg,
            "slosh_range_m": self.slosh_range_m,
            "slosh_natural_frequency_Hz": self.slosh_natural_frequency_Hz,
            "slosh_damping_ratio": self.slosh_damping_ratio,
        }
        if any(not np.isfinite(float(value)) for value in numeric.values()):
            raise ValueError("ObjectSpec numeric fields must be finite")
        if len(self.size) != 3 or any(
            not np.isfinite(float(value)) or float(value) <= 0.0
            for value in self.size
        ):
            raise ValueError("size must contain three positive finite half-extents")
        if self.mass_kg <= 0.0:
            raise ValueError("mass_kg must be positive")
        if self.stiffness_N_per_m <= 0.0:
            raise ValueError("stiffness_N_per_m must be positive")
        if self.friction_mu <= 0.0:
            raise ValueError("friction_mu must be positive")
        if not 0.0 <= self.fill_level <= 1.0:
            raise ValueError("fill_level must be in [0, 1]")
        if self.liquid_mass_kg < 0.0 or self.liquid_mass_kg > self.mass_kg:
            raise ValueError("liquid_mass_kg must be in [0, mass_kg]")
        if self.slosh_range_m < 0.0:
            raise ValueError("slosh_range_m must be non-negative")
        if self.slosh_natural_frequency_Hz < 0.0:
            raise ValueError("slosh_natural_frequency_Hz must be non-negative")
        if self.slosh_damping_ratio < 0.0:
            raise ValueError("slosh_damping_ratio must be non-negative")
        if (
            self.family == "fill"
            and self.liquid_mass_kg > 1e-5
            and self.slosh_range_m > 0.0
            and self.content_mobility_class == "none"
        ):
            # Preserve the pre-v2 2-DoF proxy instead of silently welding an old
            # ObjectSpec.  k=4 N/m and c=0.8 N*s/m are represented in the new
            # frequency/damping-ratio parameterization exactly.
            omega = float(np.sqrt(4.0 / self.liquid_mass_kg))
            self.content_mobility_class = "mobile"
            self.slosh_natural_frequency_Hz = omega / (2.0 * np.pi)
            self.slosh_damping_ratio = float(
                0.8 / (2.0 * np.sqrt(4.0 * self.liquid_mass_kg))
            )
            if self.content_proxy_version == "none":
                self.content_proxy_version = "legacy_xy_msd.v1"
        allowed_mobility = {"none", "fixed", "damped", "mobile"}
        if self.content_mobility_class not in allowed_mobility:
            raise ValueError(
                "content_mobility_class must be one of "
                f"{sorted(allowed_mobility)}, got {self.content_mobility_class!r}"
            )
        if self.content_mobility_class in {"fixed", "damped", "mobile"}:
            if self.family != "fill" or self.liquid_mass_kg <= 1e-5:
                raise ValueError(
                    "content mobility controls require a fill object with internal mass"
                )
        if self.content_mobility_class in {"damped", "mobile"} and (
            self.slosh_range_m <= 0.0
            or self.slosh_natural_frequency_Hz <= 0.0
        ):
            raise ValueError(
                "damped/mobile content requires positive slosh range and frequency"
            )

    @property
    def object_id(self) -> str:
        return f"obj{self.index}"

    def visible_dict(self) -> Dict[str, Any]:
        return {
            "object_id": self.object_id,
            "index": self.index,
            "shape": self.shape,
            "size": list(self.size),
            "rgba": list(self.rgba),
        }

    def hidden_dict(self) -> Dict[str, Any]:
        return {
            "mass_kg": self.mass_kg,
            "stiffness_N_per_m": self.stiffness_N_per_m,
            "friction_mu": self.friction_mu,
            "fill_level": self.fill_level,
            "liquid_mass_kg": self.liquid_mass_kg,
            "slosh_range_m": self.slosh_range_m,
            "container_sealed": self.container_sealed,
            "content_mobility_class": self.content_mobility_class,
            "slosh_natural_frequency_Hz": self.slosh_natural_frequency_Hz,
            "slosh_damping_ratio": self.slosh_damping_ratio,
            "content_proxy_version": self.content_proxy_version,
        }


@dataclass
class ProbeSceneSpec:
    """A simulation scene; it deliberately contains no benchmark answer."""

    scene_id: str
    family: str
    instruction: str
    objects: List[ObjectSpec]
    seed: int = 0
    # Attribute track is separate from the collision family/primitive routing.
    track: str = ""

    @property
    def n_candidates(self) -> int:
        return len(self.objects)

    def to_dict(self, reveal_hidden: bool = False) -> Dict[str, Any]:
        value = {
            "scene_id": self.scene_id,
            "family": self.family,
            "instruction": self.instruction,
            "track": self.track or self.family,
            "n_candidates": self.n_candidates,
            "objects": [obj.visible_dict() for obj in self.objects],
        }
        if reveal_hidden:
            value["hidden"] = [obj.hidden_dict() for obj in self.objects]
        return value


@dataclass
class ProbeResult:
    object_id: str
    target: int
    primitive: str
    features: Dict[str, float]
    status: str = "ok"
    backend: str = "allegro"
    valid: bool = True
    controller_status: str = "ok"
    phase_reached: str = "complete"
    violations: List[str] = field(default_factory=list)
    quality: Dict[str, float] = field(default_factory=dict)
    contact_seconds: float = 0.0
    params: Dict[str, Any] = field(default_factory=dict)
    raw_summary: Dict[str, Any] = field(default_factory=dict)
    trace: Dict[str, Any] = field(default_factory=dict)
    # Appended to preserve the positional ABI of all pre-v2 ProbeResult fields.
    scene_id: str = ""
    protocol_id: str = PROBE_PROTOCOL_ID
    feature_schema: str = FEATURE_SCHEMA_VERSION
    mode: str = ""
    sensor_profile_id: str = ""

    def to_dict(self, include_trace: bool = False) -> Dict[str, Any]:
        def convert(value: Any) -> Any:
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, np.floating):
                return float(value)
            if isinstance(value, np.integer):
                return int(value)
            if isinstance(value, dict):
                return {str(key): convert(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [convert(item) for item in value]
            return value

        payload = asdict(self)
        if not include_trace:
            payload.pop("trace", None)
        return convert(payload)
