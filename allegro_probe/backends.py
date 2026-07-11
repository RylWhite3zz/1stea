"""Execution backends for the shared probe state machines."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, FrozenSet, Protocol, runtime_checkable

from allegro_probe.models import (
    BACKENDS,
    CARRIAGE_BACKENDS,
    FRANKA_ALLEGRO_MUJOCO_BACKEND,
    ProbeSceneSpec,
)
from allegro_probe.scene import AllegroProbeScene, SceneConfig


PROBE_PRIMITIVES: FrozenSet[str] = frozenset({"poke", "heft", "shake", "slide"})


@dataclass(frozen=True)
class BackendCapabilities:
    """Actions which an execution backend can truthfully execute today."""

    supported_primitives: FrozenSet[str]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "supported_primitives",
            frozenset(str(value) for value in self.supported_primitives),
        )
        unknown = self.supported_primitives - PROBE_PRIMITIVES
        if unknown:
            raise ValueError(f"unknown probe primitives in capabilities: {sorted(unknown)}")

    def supports_primitive(self, primitive: str) -> bool:
        return str(primitive) in self.supported_primitives


class UnsupportedPrimitiveError(RuntimeError):
    """Raised before dispatch when a backend has no implementation for an action."""


class BackendSceneConfig(Protocol):
    """Configuration identity needed by every registered scene."""

    backend: str


class BackendScene(Protocol):
    """Small common surface shared by carriage and robot-arm scenes."""

    config: BackendSceneConfig

    def reset(self) -> None:
        ...


@runtime_checkable
class ProbeBackend(Protocol):
    """Backend boundary consumed by the primitive controller."""

    name: str
    scene: BackendScene
    capabilities: BackendCapabilities

    @property
    def supported_primitives(self) -> FrozenSet[str]:
        ...

    def supports_primitive(self, primitive: str) -> bool:
        ...

    def reset(self) -> None:
        ...


class _SceneBackend:
    name = ""
    capabilities = BackendCapabilities(PROBE_PRIMITIVES)

    def __init__(self, scene: BackendScene):
        if scene.config.backend != self.name:
            raise ValueError(
                f"{type(self).__name__} requires a {self.name!r} scene, "
                f"got {scene.config.backend!r}"
            )
        self.scene = scene

    @property
    def supported_primitives(self) -> FrozenSet[str]:
        return self.capabilities.supported_primitives

    def supports_primitive(self, primitive: str) -> bool:
        return self.capabilities.supports_primitive(primitive)

    def __getattr__(self, name: str) -> Any:
        return getattr(self.scene, name)

    def reset(self) -> None:
        self.scene.reset()

    @classmethod
    def create(
        cls,
        task: ProbeSceneSpec,
        *,
        menagerie_root: Path | None = None,
        **scene_options: Any,
    ) -> "_SceneBackend":
        options = dict(scene_options)
        options["backend"] = cls.name
        if menagerie_root is not None:
            options["menagerie_root"] = Path(menagerie_root)
        return cls(AllegroProbeScene(task, SceneConfig(**options)))


class ReferenceProbeBackend(_SceneBackend):
    """Reference poke tool plus fingertip pad and deterministic two-jaw gripper."""

    name = "reference"


class AllegroHandBackend(_SceneBackend):
    """Wonik Allegro hand with fingertip poke/slide sensing and wrist F/T."""

    name = "allegro"


class FrankaAllegroMujocoBackend(_SceneBackend):
    """Stage-1 Panda+Allegro model backend.

    The independent model and actuator smoke tests exist before any probe
    primitive is ported.  Empty capabilities are intentional: advertising the
    carriage primitives here would route the robot through incompatible
    position-command branches.
    """

    name = FRANKA_ALLEGRO_MUJOCO_BACKEND
    capabilities = BackendCapabilities(frozenset())

    @classmethod
    def create(
        cls,
        task: ProbeSceneSpec | None = None,
        *,
        menagerie_root: Path | None = None,
        **scene_options: Any,
    ) -> "FrankaAllegroMujocoBackend":
        # Delayed import keeps the existing two backends independent of the
        # optional robot scene at import time and avoids a scene/backend cycle.
        from allegro_probe.franka_scene import FrankaAllegroScene, FrankaSceneConfig

        options = dict(scene_options)
        options["backend"] = cls.name
        if menagerie_root is not None:
            options["menagerie_root"] = Path(menagerie_root)
        return cls(FrankaAllegroScene(task, FrankaSceneConfig(**options)))


_BACKEND_TYPES = {
    "reference": ReferenceProbeBackend,
    "allegro": AllegroHandBackend,
    FRANKA_ALLEGRO_MUJOCO_BACKEND: FrankaAllegroMujocoBackend,
}


def create_backend(
    name: str,
    task: ProbeSceneSpec | None = None,
    *,
    menagerie_root: Path | None = None,
    **scene_options: Any,
) -> ProbeBackend:
    """Construct the scene implementation registered for ``name``."""

    backend_name = str(name)
    try:
        backend_type = _BACKEND_TYPES[backend_name]
    except KeyError as exc:
        raise ValueError(
            f"backend must be one of {BACKENDS}, got {backend_name!r}"
        ) from exc
    if backend_name in CARRIAGE_BACKENDS and task is None:
        raise ValueError(f"backend {backend_name!r} requires a ProbeSceneSpec")
    return backend_type.create(
        task, menagerie_root=menagerie_root, **scene_options
    )


def require_supported_primitive(backend: ProbeBackend, primitive: str) -> None:
    """Reject unsupported actions before embodiment-specific dispatch begins."""

    primitive_name = str(primitive)
    if not backend.supports_primitive(primitive_name):
        supported = sorted(backend.supported_primitives)
        raise UnsupportedPrimitiveError(
            f"backend {backend.name!r} does not support probe primitive "
            f"{primitive_name!r}; supported_primitives={supported}"
        )


def as_backend(value: ProbeBackend | BackendScene) -> ProbeBackend:
    if isinstance(value, AllegroProbeScene):
        if value.config.backend == "reference":
            return ReferenceProbeBackend(value)
        if value.config.backend == "allegro":
            return AllegroHandBackend(value)
        raise ValueError(
            "AllegroProbeScene only supports carriage backends "
            f"{CARRIAGE_BACKENDS}, got {value.config.backend!r}"
        )
    if isinstance(value, ProbeBackend):
        return value

    # Import only when a non-carriage scene needs classification.  This makes
    # importing legacy entry points independent of Franka model construction.
    try:
        from allegro_probe.franka_scene import FrankaAllegroScene
    except ModuleNotFoundError as exc:
        if exc.name != "allegro_probe.franka_scene":
            raise
        raise TypeError(
            f"expected ProbeBackend or backend scene, got {type(value)!r}"
        ) from exc

    if isinstance(value, FrankaAllegroScene):
        return FrankaAllegroMujocoBackend(value)
    raise TypeError(f"expected ProbeBackend or backend scene, got {type(value)!r}")
