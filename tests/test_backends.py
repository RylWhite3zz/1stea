from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

import allegro_probe.backends as backend_module
from allegro_probe.backends import (
    AllegroHandBackend,
    BackendCapabilities,
    FrankaAllegroMujocoBackend,
    ReferenceProbeBackend,
    UnsupportedPrimitiveError,
    as_backend,
    create_backend,
)
from allegro_probe.demo_scenes import make_demo_scene
from allegro_probe.interfaces import ProbeCommand, ProbeHarness
from allegro_probe.models import BACKENDS, CARRIAGE_BACKENDS
from allegro_probe.primitives import poke, run_probe
from allegro_probe.scene import AllegroProbeScene, SceneConfig


def test_backend_registry_separates_carriage_and_franka() -> None:
    assert CARRIAGE_BACKENDS == ("reference", "allegro")
    assert BACKENDS == (
        "reference",
        "allegro",
        "franka_allegro_mujoco",
    )
    assert ReferenceProbeBackend.capabilities.supported_primitives == {
        "poke",
        "heft",
        "shake",
        "slide",
    }
    assert AllegroHandBackend.capabilities.supported_primitives == {
        "poke",
        "heft",
        "shake",
        "slide",
    }
    assert FrankaAllegroMujocoBackend.capabilities == BackendCapabilities(frozenset())


@pytest.mark.parametrize(
    ("backend_name", "backend_type"),
    [("reference", ReferenceProbeBackend), ("allegro", AllegroHandBackend)],
)
def test_factory_preserves_carriage_scene_types(backend_name, backend_type) -> None:
    class FakeCarriageScene:
        def __init__(self, task, config):
            self.task = task
            self.config = config

        def reset(self) -> None:
            pass

    original = backend_module.AllegroProbeScene
    backend_module.AllegroProbeScene = FakeCarriageScene
    try:
        backend = create_backend(
            backend_name,
            make_demo_scene("stiffness", n_candidates=2, seed=0),
        )
    finally:
        backend_module.AllegroProbeScene = original
    assert isinstance(backend, backend_type)
    assert isinstance(backend.scene, FakeCarriageScene)
    assert backend.scene.config.backend == backend_name


def test_factory_requires_a_task_for_carriage_scenes() -> None:
    with pytest.raises(ValueError, match="requires a ProbeSceneSpec"):
        create_backend("reference")
    with pytest.raises(ValueError, match="backend must be one of"):
        create_backend("not-a-backend")


def test_factory_uses_the_independent_franka_scene(monkeypatch) -> None:
    class FakeFrankaConfig:
        def __init__(self, **options):
            self.__dict__.update(options)

    class FakeFrankaScene:
        def __init__(self, task, config):
            self.task = task
            self.config = config

        def reset(self) -> None:
            pass

    monkeypatch.setitem(
        sys.modules,
        "allegro_probe.franka_scene",
        SimpleNamespace(
            FrankaAllegroScene=FakeFrankaScene,
            FrankaSceneConfig=FakeFrankaConfig,
        ),
    )
    task = make_demo_scene("mass", n_candidates=2, seed=0)
    backend = create_backend(
        "franka_allegro_mujoco",
        task,
        menagerie_root="/tmp/menagerie",
    )
    assert isinstance(backend, FrankaAllegroMujocoBackend)
    assert isinstance(backend.scene, FakeFrankaScene)
    assert backend.scene.task is task
    assert backend.scene.config.backend == "franka_allegro_mujoco"
    assert str(backend.scene.config.menagerie_root) == "/tmp/menagerie"


def test_as_backend_does_not_treat_unknown_carriage_scene_as_reference() -> None:
    scene = object.__new__(AllegroProbeScene)
    scene.config = SimpleNamespace(backend="franka_allegro_mujoco")
    with pytest.raises(ValueError, match="only supports carriage backends"):
        as_backend(scene)


def test_carriage_scene_config_rejects_the_franka_backend() -> None:
    with pytest.raises(ValueError, match="AllegroProbeScene backend"):
        SceneConfig(backend="franka_allegro_mujoco")


def test_stage1_franka_harness_rejects_before_dispatch() -> None:
    class Stage1Scene:
        config = SimpleNamespace(backend="franka_allegro_mujoco")

        def reset(self) -> None:
            raise AssertionError("unsupported action must not touch the scene")

    backend = FrankaAllegroMujocoBackend(Stage1Scene())
    harness = ProbeHarness(backend)
    with pytest.raises(
        UnsupportedPrimitiveError,
        match=r"franka_allegro_mujoco.*does not support.*poke",
    ):
        harness.execute(ProbeCommand("poke", target=0))

    # The lower-level public entry points carry the same admission gate; none
    # may fall through the old "non-allegro means reference" branches.
    with pytest.raises(UnsupportedPrimitiveError, match="does not support.*poke"):
        run_probe(backend, "poke", target=0)
    with pytest.raises(UnsupportedPrimitiveError, match="does not support.*poke"):
        poke(backend, target=0)


def test_custom_backend_must_expose_capabilities_to_be_structural() -> None:
    class IncompleteBackend:
        name = "custom"
        scene = SimpleNamespace(config=SimpleNamespace(backend="custom"))

        def reset(self) -> None:
            pass

    with pytest.raises(TypeError, match="expected ProbeBackend"):
        as_backend(IncompleteBackend())
