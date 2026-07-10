"""Run the training-free object-pose -> fixed-place Allegro interface."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time

import numpy as np

from allegro_probe import (
    AllegroHandBackend,
    FixedPlaceSpec,
    ObjectPoseObservation,
    PoseConditionedPickPlaceRequest,
    PoseConditionedShortCanController,
    ProbeCommand,
    ProbeHarness,
    RigidTransform,
    make_demo_scene,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Use one valid heft result and a supplied short-can center pose to "
            "pick from the table and place at one absolute fixed destination"
        )
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--candidates", type=int, default=3)
    parser.add_argument("--target", type=int, default=0)
    parser.add_argument("--source-x", type=float, default=-0.08)
    parser.add_argument("--source-y", type=float, default=-0.12)
    parser.add_argument("--place-x", type=float, default=0.0)
    parser.add_argument("--place-y", type=float, default=0.12)
    parser.add_argument("--place-tolerance", type=float, default=0.035)
    parser.add_argument("--pose-confidence", type=float, default=1.0)
    parser.add_argument(
        "--verify-live-pose",
        action="store_true",
        help="verify an already-populated scene instead of resetting to the request pose",
    )
    parser.add_argument(
        "--menagerie-root",
        type=Path,
        default=Path("/home/enovo/robots/sim/mujoco_menagerie/wonik_allegro"),
    )
    parser.add_argument("--include-trace", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--hold-open", action="store_true")
    args = parser.parse_args()

    spec = make_demo_scene("mass", args.candidates, args.seed)
    if args.target < 0 or args.target >= spec.n_candidates:
        raise SystemExit(
            f"target must be in [0, {spec.n_candidates - 1}], got {args.target}"
        )
    obj = spec.objects[args.target]

    # Probe and manipulation remain separate execution stages.  Both now use
    # support-free, full-collision models; manipulation additionally resets the
    # target to the externally supplied source pose.
    probe_backend = AllegroHandBackend.create(
        spec,
        menagerie_root=args.menagerie_root,
    )
    probe_result = ProbeHarness(probe_backend).execute(
        ProbeCommand("heft", target=args.target)
    )
    print("PROBE_RESULT")
    print(
        json.dumps(
            probe_result.to_dict(include_trace=args.include_trace), indent=2
        )
    )
    if not probe_result.valid:
        raise SystemExit("heft result is invalid; manipulation admission is closed")

    manipulation_backend = AllegroHandBackend.create(
        spec,
        menagerie_root=args.menagerie_root,
        allegro_grasp_lift=0.0,
        full_hand_collisions=True,
        wrist_roll_limit_rad=np.pi,
    )
    scene = manipulation_backend.scene
    source_pose = RigidTransform(
        parent_frame="world",
        child_frame=obj.object_id,
        translation_m=(args.source_x, args.source_y, obj.size[2]),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
    )
    goal_pose = RigidTransform(
        parent_frame="world",
        child_frame=obj.object_id,
        translation_m=(args.place_x, args.place_y, obj.size[2]),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
    )
    observation = ObjectPoseObservation(
        target=args.target,
        object_id=obj.object_id,
        T_world_object=source_pose,
        timestamp_s=0.0,
        confidence=args.pose_confidence,
    )
    fixed_goal = FixedPlaceSpec(
        goal_id="short_can_drop_zone_v1",
        T_world_object_goal=goal_pose,
        surface_z_m=0.0,
        max_position_error_m=args.place_tolerance,
    )
    handoff_policy = (
        "verify_live_pose" if args.verify_live_pose else "reset_to_requested_pose"
    )
    request = PoseConditionedPickPlaceRequest(
        object_pose=observation,
        fixed_goal_id=fixed_goal.goal_id,
        handoff_policy=handoff_policy,
    )
    if args.verify_live_pose:
        scene.set_object_pose(
            args.target,
            center_position_m=source_pose.translation_m,
            quaternion_wxyz=source_pose.quaternion_wxyz,
            record_initial=True,
        )
        scene.step(50)

    controller = PoseConditionedShortCanController(scene, fixed_goal)
    decision = controller.plan(probe_result, request)
    print("\nMANIPULATION_REQUEST")
    print(json.dumps(request.to_dict(), indent=2))
    print("\nPLAN_DECISION")
    print(json.dumps(decision.to_dict(), indent=2))
    if not decision.executable or decision.plan is None:
        raise SystemExit(f"no executable plan: {decision.reason}")

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
        result = controller.execute(decision.plan)
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
