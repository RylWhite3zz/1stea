"""Run one probe primitive against every candidate in a synthetic scene."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time

from allegro_probe import (
    AllegroProbeScene,
    ProbeCommand,
    ProbeHarness,
    SceneConfig,
    make_demo_scene,
    primitive_for_family,
)
from allegro_probe.models import canonical_family


def main() -> None:
    parser = argparse.ArgumentParser(description="Allegro probe primitive demo")
    parser.add_argument(
        "--family",
        default="mass",
        choices=["stiffness", "mass", "fill", "material", "smoothness"],
    )
    parser.add_argument("--candidates", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--primitive", default=None)
    parser.add_argument(
        "--menagerie-root",
        type=Path,
        default=Path("/home/enovo/robots/sim/mujoco_menagerie/wonik_allegro"),
    )
    parser.add_argument("--reset-between-probes", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--hold-open", action="store_true")
    args = parser.parse_args()

    family = canonical_family(args.family)
    primitive = args.primitive or primitive_for_family(family)
    spec = make_demo_scene(family, args.candidates, args.seed)
    scene = AllegroProbeScene(
        spec,
        SceneConfig(menagerie_root=args.menagerie_root),
    )
    harness = ProbeHarness(scene)

    viewer_context = nullcontext(None)
    if args.viewer:
        try:
            import mujoco.viewer  # type: ignore
        except Exception as exc:
            raise SystemExit(f"mujoco.viewer is unavailable: {exc}") from exc
        viewer_context = mujoco.viewer.launch_passive(scene.model, scene.data)

    with viewer_context as viewer:
        if viewer is not None:
            scene.attach_viewer(viewer, realtime=True)

        print("SCENE")
        print(json.dumps(spec.to_dict(reveal_hidden=True), indent=2))
        print("\nPROBE_RESULTS")
        for target in range(spec.n_candidates):
            if args.reset_between_probes and target > 0:
                scene.reset()
            result = harness.execute(ProbeCommand(primitive=primitive, target=target))
            print(json.dumps(result.to_dict(), indent=2))

        if viewer is not None and args.hold_open:
            print("\nClose the viewer or press Ctrl-C to exit.")
            while viewer.is_running():
                scene.step(1)
        elif viewer is not None:
            deadline = time.time() + 3.0
            while viewer.is_running() and time.time() < deadline:
                scene.step(1)
        scene.detach_viewer()


if __name__ == "__main__":
    main()
