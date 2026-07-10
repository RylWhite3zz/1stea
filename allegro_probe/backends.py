"""Execution backends for the shared probe state machines."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from allegro_probe.models import ProbeSceneSpec
from allegro_probe.scene import AllegroProbeScene, SceneConfig


@runtime_checkable
class ProbeBackend(Protocol):
    """Backend boundary consumed by the primitive controller."""

    name: str
    scene: AllegroProbeScene

    def reset(self) -> None:
        ...


class _SceneBackend:
    name = ""

    def __init__(self, scene: AllegroProbeScene):
        if scene.config.backend != self.name:
            raise ValueError(
                f"{type(self).__name__} requires a {self.name!r} scene, "
                f"got {scene.config.backend!r}"
            )
        self.scene = scene

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
    """Instrumented probe plus a deterministic two-jaw reference gripper."""

    name = "reference"


class AllegroHandBackend(_SceneBackend):
    """Wonik Allegro hand with an instrumented wrist probe."""

    name = "allegro"


def as_backend(value: ProbeBackend | AllegroProbeScene) -> ProbeBackend:
    if isinstance(value, AllegroProbeScene):
        if value.config.backend == "reference":
            return ReferenceProbeBackend(value)
        return AllegroHandBackend(value)
    if isinstance(value, ProbeBackend):
        return value
    raise TypeError(f"expected ProbeBackend or AllegroProbeScene, got {type(value)!r}")
