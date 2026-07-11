"""Generic high-level contracts; fixed manipulation lives in a typed side module."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Literal, Mapping, Protocol

from allegro_probe.backends import ProbeBackend, as_backend
from allegro_probe.models import ProbeResult
from allegro_probe.primitives import run_probe
from allegro_probe.protocols import PROBE_PROTOCOL_ID, validate_protocol_id
from allegro_probe.scene import AllegroProbeScene


@dataclass(frozen=True)
class ProbeCommand:
    primitive: Literal["poke", "heft", "shake", "slide"]
    target: int
    params: Dict[str, Any] = field(default_factory=dict)
    # Appended so ProbeCommand(primitive, target, params) remains source-compatible.
    mode: str | None = None
    protocol_id: str = PROBE_PROTOCOL_ID


@dataclass(frozen=True)
class ManipulationCommand:
    """Placeholder for a future generic vocabulary.

    The implemented fixed ``short_can_pick_place`` path has stricter typed contracts
    in :mod:`allegro_probe.manipulation` and does not consume this placeholder.
    """

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
    """Future generic execution backend; one fixed Allegro path exists separately."""

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
        protocol_id = validate_protocol_id(command.protocol_id)
        params = dict(command.params)
        if "protocol_id" in params:
            if str(params.pop("protocol_id")) != protocol_id:
                raise ValueError(
                    "ProbeCommand.protocol_id conflicts with params['protocol_id']"
                )
        if "mode" in params and command.mode is not None:
            if str(params["mode"]) != str(command.mode):
                raise ValueError("ProbeCommand.mode conflicts with params['mode']")
        if command.mode is not None:
            params["mode"] = command.mode
        return run_probe(
            self.backend,
            primitive=command.primitive,
            target=command.target,
            protocol_id=protocol_id,
            **params,
        )
