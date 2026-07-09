"""High-level integration contracts that are intentionally not implemented yet."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Literal, Mapping, Protocol

from allegro_probe.backends import ProbeBackend, as_backend
from allegro_probe.models import ProbeResult
from allegro_probe.primitives import run_probe
from allegro_probe.scene import AllegroProbeScene


@dataclass(frozen=True)
class ProbeCommand:
    primitive: Literal["poke", "heft", "shake", "slide"]
    target: int
    params: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ManipulationCommand:
    """Placeholder only; action vocabulary and controller are not designed."""

    name: str
    target: int
    params: Dict[str, Any] = field(default_factory=dict)


class VLMPolicy(Protocol):
    """Future adapter from a multimodal model to a harness command."""

    def decide(
        self, observation: Mapping[str, Any]
    ) -> ProbeCommand | ManipulationCommand:
        ...


class ManipulationController(Protocol):
    """Future execution backend for final task actions."""

    def execute(
        self, scene: AllegroProbeScene, command: ManipulationCommand
    ) -> Mapping[str, Any]:
        ...


class ProbeHarness:
    """The currently implemented high-level boundary: execute one probe command."""

    def __init__(self, executor: ProbeBackend | AllegroProbeScene):
        self.backend = as_backend(executor)
        self.scene = self.backend.scene

    def execute(self, command: ProbeCommand) -> ProbeResult:
        return run_probe(
            self.backend,
            primitive=command.primitive,
            target=command.target,
            **command.params,
        )
