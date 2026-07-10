"""Run safe heft, then the explicitly isolated legacy v1 pick/place path."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time

from allegro_probe import (
    AllegroHandBackend,
    ProbeCommand,
    ProbeHarness,
    ShortCanPickPlaceRequest,
    build_short_can_pick_place_plan,
    execute_short_can_pick_place,
    make_demo_scene,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Probe-conditioned Allegro short-can pick/place demo"
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--candidates", type=int, default=3)
    parser.add_argument("--target", type=int, default=0)
    parser.add_argument(
        "--menagerie-root",
        type=Path,
        default=Path("/home/enovo/robots/sim/mujoco_menagerie/wonik_allegro"),
    )
    parser.add_argument("--include-trace", action="store_true")
    parser.add_argument("--reveal-hidden", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--hold-open", action="store_true")
    args = parser.parse_args()

    spec = make_demo_scene("mass", args.candidates, args.seed)
    if args.target < 0 or args.target >= spec.n_candidates:
        raise SystemExit(
            f"target must be in [0, {spec.n_candidates - 1}], got {args.target}"
        )
    probe_backend = AllegroHandBackend.create(
        spec,
        menagerie_root=args.menagerie_root,
    )
    probe_result = ProbeHarness(probe_backend).execute(
        ProbeCommand("heft", target=args.target)
    )
    # v1 side-wrap is retained only as an explicit compatibility route.  It
    # cannot share the new full-collision, support-free probe model.
    backend = AllegroHandBackend.create(
        spec,
        menagerie_root=args.menagerie_root,
        allegro_grasp_lift=0.090,
        full_hand_collisions=False,
        wrist_roll_limit_rad=0.9,
    )
    scene = backend.scene

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
        print(json.dumps(spec.to_dict(reveal_hidden=args.reveal_hidden), indent=2))

        print("\nPROBE_RESULT")
        print(
            json.dumps(
                probe_result.to_dict(include_trace=args.include_trace),
                indent=2,
            )
        )

        decision = build_short_can_pick_place_plan(
            scene,
            probe_result,
            ShortCanPickPlaceRequest(target=args.target),
        )
        print("\nPLAN_DECISION")
        print(json.dumps(decision.to_dict(), indent=2))
        if not decision.executable or decision.plan is None:
            raise SystemExit(f"no executable plan: {decision.reason}")

        result = execute_short_can_pick_place(scene, decision.plan)
        print("\nMANIPULATION_RESULT")
        print(json.dumps(result.to_dict(include_trace=args.include_trace), indent=2))

        if viewer is not None and args.hold_open:
            print("\nClose the viewer or press Ctrl-C to exit.")
            while viewer.is_running():
                scene.step(1)
        elif viewer is not None:
            deadline = time.time() + 3.0
            while viewer.is_running() and time.time() < deadline:
                scene.step(1)
        scene.detach_viewer()

    if not result.success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
