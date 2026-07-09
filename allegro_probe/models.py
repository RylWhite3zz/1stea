"""Data contracts shared by the simulator and probe primitives."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Tuple

import numpy as np


FAMILIES = ("stiffness", "mass", "fill", "material")
FAMILY_ALIASES = {"smoothness": "material"}
BACKENDS = ("reference", "allegro")


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
        }


@dataclass
class ProbeSceneSpec:
    """A simulation scene; it deliberately contains no benchmark answer."""

    scene_id: str
    family: str
    instruction: str
    objects: List[ObjectSpec]
    seed: int = 0

    @property
    def n_candidates(self) -> int:
        return len(self.objects)

    def to_dict(self, reveal_hidden: bool = False) -> Dict[str, Any]:
        value = {
            "scene_id": self.scene_id,
            "family": self.family,
            "instruction": self.instruction,
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
