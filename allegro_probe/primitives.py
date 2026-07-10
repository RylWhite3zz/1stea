"""Probe-aware state machines and feature extraction."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Tuple

import numpy as np

from allegro_probe.backends import ProbeBackend, as_backend
from allegro_probe.models import ProbeResult, canonical_family
from allegro_probe.scene import AllegroProbeScene, ContactSnapshot


def primitive_for_family(family: str) -> str:
    fam = canonical_family(family)
    return {
        "stiffness": "poke",
        "mass": "heft",
        "fill": "shake",
        "material": "slide",
    }[fam]


@dataclass
class _Run:
    backend: ProbeBackend
    primitive: str
    target: int
    phase: str = "init"
    violations: List[str] = field(default_factory=list)
    quality: Dict[str, float] = field(default_factory=dict)
    trace: Dict[str, List[Any]] = field(default_factory=dict)

    @property
    def scene(self) -> AllegroProbeScene:
        return self.backend.scene

    def enter(self, phase: str) -> None:
        self.phase = phase
        self.trace.setdefault("phase", []).append(phase)

    def fail(self, violation: str) -> None:
        if violation not in self.violations:
            self.violations.append(violation)

    def sample(self, **values: Any) -> None:
        for name, value in values.items():
            if isinstance(value, np.ndarray):
                value = value.copy()
            self.trace.setdefault(name, []).append(value)

    def result(
        self,
        *,
        features: Dict[str, float],
        contact_seconds: float,
        params: Dict[str, Any],
        raw_summary: Dict[str, Any],
        valid: bool,
        status: str | None = None,
    ) -> ProbeResult:
        controller_status = status or (
            "ok" if valid else (self.violations[0] if self.violations else "invalid")
        )
        return ProbeResult(
            object_id=f"obj{self.target}",
            target=self.target,
            primitive=self.primitive,
            scene_id=self.scene.task.scene_id,
            backend=self.backend.name,
            status=controller_status,
            valid=bool(valid),
            controller_status=controller_status,
            phase_reached=self.phase,
            violations=list(self.violations),
            quality=dict(self.quality),
            features=features,
            contact_seconds=float(contact_seconds),
            params=params,
            raw_summary=raw_summary,
            trace=self.trace,
        )


def _mean_vec(
    scene: AllegroProbeScene,
    reader: Callable[[], np.ndarray],
    steps: int = 60,
) -> np.ndarray:
    values = []
    for _ in range(int(steps)):
        scene.step(1)
        values.append(np.asarray(reader(), dtype=float).copy())
    return np.mean(values, axis=0)


def _ctrl(scene: AllegroProbeScene, actuator: str) -> float:
    return float(scene.data.ctrl[scene.act[actuator]])


def _move_wrist(
    run: _Run,
    *,
    steps: int,
    x: float | None = None,
    y: float | None = None,
    z: float | None = None,
    roll: float | None = None,
    tilt: float | None = None,
    yaw: float | None = None,
    probe: float | None = None,
) -> None:
    """Smoothly interpolate carriage controls; this is not arm planning."""

    scene = run.scene
    targets = {
        "x": x,
        "y": y,
        "z": z,
        "roll": roll,
        "tilt": tilt,
        "yaw": yaw,
        "probe": probe,
    }
    actuator = {
        "x": "act_wx",
        "y": "act_wy",
        "z": "act_wz",
        "roll": "act_wr",
        "tilt": "act_wt",
        "yaw": "act_wyaw",
        "probe": "act_wp",
    }
    starts = {
        key: _ctrl(scene, actuator[key])
        for key, target in targets.items()
        if target is not None
    }
    n = max(int(steps), 1)
    for alpha in np.linspace(0.0, 1.0, n):
        command = {
            key: starts[key] + float(alpha) * (float(target) - starts[key])
            for key, target in targets.items()
            if target is not None
        }
        scene.command(**command)
        scene.step(1)


def _probe_above(
    run: _Run,
    *,
    x: float,
    clearance: float,
) -> None:
    scene = run.scene
    top = scene.object_top_z(run.target)
    _move_wrist(
        run,
        steps=140,
        x=x,
        y=0.0,
        z=scene.wz_for_tip_z(top + clearance),
        roll=0.0,
        tilt=0.0,
        yaw=0.0,
        probe=0.0,
    )
    # Position actuators need to settle laterally before guarded descent; otherwise
    # an edge candidate is contacted off-centre and produces a spurious side load.
    scene.step(120)


def _safe_retreat(run: _Run, *, release: bool) -> None:
    run.enter("retreat")
    scene = run.scene
    _move_wrist(
        run,
        steps=100,
        z=0.10,
        roll=0.0,
        tilt=0.0,
        yaw=0.0,
        probe=0.0,
    )
    if release:
        for alpha in np.linspace(scene._grip_alpha, 0.0, 60):
            scene.command(grip=float(alpha))
            scene.step(2)


def _contact_stable(
    run: _Run,
    *,
    steps: int,
    require_support_free: bool,
    penetration_limit: float,
    quality_prefix: str,
) -> Tuple[bool, ContactSnapshot, float]:
    scene = run.scene
    rel = []
    valid_steps = 0
    last = ContactSnapshot()
    max_penetration = 0.0
    for _ in range(int(steps)):
        scene.step(1)
        last = scene.contact_snapshot(run.target)
        max_penetration = max(max_penetration, last.max_penetration_m)
        opposing = scene.has_opposing_grasp(last)
        support_ok = not require_support_free or (
            not last.support_contact and not last.table_contact
        )
        penetration_ok = last.max_penetration_m <= penetration_limit
        if opposing and support_ok and penetration_ok:
            valid_steps += 1
        rel.append(scene.relative_object_position(run.target).copy())
    rel_arr = np.asarray(rel, dtype=float)
    drift = (
        float(np.max(np.linalg.norm(rel_arr - rel_arr[0], axis=1)))
        if len(rel_arr)
        else float("inf")
    )
    run.quality[f"{quality_prefix}_relative_drift_m"] = drift
    run.quality[f"{quality_prefix}_max_penetration_m"] = max_penetration
    run.quality[f"{quality_prefix}_stable_fraction"] = valid_steps / max(
        int(steps), 1
    )
    return valid_steps >= int(0.85 * steps), last, drift


@dataclass
class _Grasp:
    established: bool
    baseline_force: np.ndarray
    baseline_torque: np.ndarray
    close_alpha: float
    snapshot: ContactSnapshot


def _grasp_pose(scene: AllegroProbeScene, target: int) -> Tuple[float, float, float]:
    obj = scene.task.objects[target]
    x = scene.candidate_x(target)
    if scene.config.backend == "reference":
        y = 0.0
        wrist_to_waist = scene.config.palm_height
    else:
        y = 0.020
        # Place medial/distal links around the narrow waist, below the top lip.
        # This produces form-assisted lifting instead of relying on extreme pinch force.
        wrist_to_waist = 0.399 if obj.family == "mass" else 0.389
    z = float(
        np.clip(scene.object_mid_z(target) - wrist_to_waist, -0.55, 0.14)
    )
    return x, y, z


def _prepare_grasp(
    run: _Run,
    *,
    penetration_limit: float,
    min_grasp_force: float,
) -> _Grasp:
    scene = run.scene
    x, y, z_grasp = _grasp_pose(scene, run.target)

    run.enter("approach")
    _move_wrist(
        run,
        steps=180,
        x=x,
        y=y,
        z=min(z_grasp + 0.08, 0.02),
        roll=0.0,
        tilt=0.0,
        yaw=0.0,
        probe=0.0,
    )
    scene.step(120)

    run.enter("guarded_descent")
    _move_wrist(run, steps=180, z=z_grasp)
    scene.step(100)

    run.enter("contact_establish")
    close_alpha = 0.0
    snapshot = scene.contact_snapshot(run.target)
    established = False
    for close_alpha in np.linspace(0.0, 1.0, 101):
        scene.command(grip=float(close_alpha))
        scene.step(8)
        snapshot = scene.contact_snapshot(run.target)
        run.sample(
            grasp_alpha=float(close_alpha),
            grasp_groups=list(snapshot.hand_groups),
            grasp_force_N=snapshot.hand_normal_force_N,
            grasp_penetration_m=snapshot.max_penetration_m,
        )
        if snapshot.max_penetration_m > penetration_limit:
            run.fail("penetration_limit")
            break
        if (
            scene.has_opposing_grasp(snapshot)
            and snapshot.hand_normal_force_N >= min_grasp_force
        ):
            established = True
            break

    run.enter("contact_quality_gate")
    if not established:
        if not scene.has_opposing_grasp(snapshot):
            run.fail("no_opposing_contact")
        elif snapshot.hand_normal_force_N < min_grasp_force:
            run.fail("insufficient_grasp_force")
    else:
        # Apply a short bounded squeeze stage.  This follows pregrasp -> grasp ->
        # squeeze without blindly driving every hand to its maximum closed pose.
        safe_alpha = float(close_alpha)
        squeeze_margin = 0.09 if run.backend.name == "reference" else 0.01
        for squeeze_alpha in np.linspace(
            close_alpha, min(float(close_alpha) + squeeze_margin, 1.0), 8
        ):
            scene.command(grip=float(squeeze_alpha))
            scene.step(4)
            candidate = scene.contact_snapshot(run.target)
            if candidate.max_penetration_m > penetration_limit:
                scene.command(grip=safe_alpha)
                scene.step(12)
                break
            safe_alpha = float(squeeze_alpha)
            snapshot = candidate
        close_alpha = safe_alpha
        scene.step(40)
        stable, snapshot, drift = _contact_stable(
            run,
            steps=60,
            require_support_free=False,
            penetration_limit=penetration_limit,
            quality_prefix="pregrasp",
        )
        if not stable:
            established = False
            run.fail("unstable_pregrasp")
        if drift > 0.006:
            established = False
            run.fail("pregrasp_slip")

    baseline_force = _mean_vec(scene, scene.wrist_force_vec, 50)
    baseline_torque = _mean_vec(scene, scene.wrist_torque_vec, 50)
    run.quality["grasp_group_count"] = float(len(snapshot.hand_groups))
    run.quality["grasp_normal_force_N"] = snapshot.hand_normal_force_N
    run.quality["prelift_support_contact"] = float(snapshot.support_contact)
    return _Grasp(
        established=established,
        baseline_force=baseline_force,
        baseline_torque=baseline_torque,
        close_alpha=float(close_alpha),
        snapshot=snapshot,
    )


def _lift_and_gate(
    run: _Run,
    grasp: _Grasp,
    *,
    lift_height: float,
    penetration_limit: float,
) -> Tuple[bool, ContactSnapshot]:
    scene = run.scene
    run.enter("primitive_execution")
    z0 = _ctrl(scene, "act_wz")
    _move_wrist(run, steps=220, z=z0 + lift_height)

    run.enter("post_check")
    stable, snapshot, drift = _contact_stable(
        run,
        steps=80,
        require_support_free=True,
        penetration_limit=penetration_limit,
        quality_prefix="postlift",
    )
    lifted_distance = float(
        scene.object_pos(run.target)[2] - scene._initial_object_pos[run.target][2]
    )
    run.quality["lift_distance_m"] = lifted_distance
    run.quality["support_contact_after_lift"] = float(snapshot.support_contact)
    run.quality["table_contact_after_lift"] = float(snapshot.table_contact)
    run.quality["postlift_group_count"] = float(len(snapshot.hand_groups))

    valid = grasp.established and stable
    if lifted_distance < min(0.010, 0.5 * lift_height):
        valid = False
        run.fail("not_lifted")
    if snapshot.support_contact or snapshot.table_contact:
        valid = False
        run.fail("support_contact_after_lift")
    if not scene.has_opposing_grasp(snapshot):
        valid = False
        run.fail("lost_grasp")
    if drift > 0.008:
        valid = False
        run.fail("postlift_slip")
    return valid, snapshot


def poke(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    depth: float = 0.006,
    target_force: float = 3.0,
    force_limit: float = 10.0,
    contact_threshold: float = 0.05,
    lateral_ratio_limit: float = 0.35,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(backend, "poke", int(target))
    scene = run.scene
    x = scene.candidate_x(run.target)

    run.enter("approach")
    _probe_above(run, x=x, clearance=0.030)
    force_baseline = _mean_vec(scene, scene.probe_force_vec, 40)

    run.enter("guarded_contact")
    contact_z = None
    object_z = None
    extension = 0.0
    for extension in np.linspace(0.0, 0.17, 120):
        scene.command(probe=float(extension))
        scene.step(4)
        fn = scene.probe_touch()
        if fn > force_limit:
            run.fail("force_limit")
            break
        if fn >= contact_threshold:
            contact_z = float(scene.probe_tip_pos()[2])
            object_z = float(scene.object_pos(run.target)[2])
            break
    if contact_z is None:
        run.fail("no_contact")

    run.enter("contact_quality_gate")
    trace: List[Tuple[float, float, float, float]] = []
    contact_steps = 0
    if contact_z is not None:
        run.enter("primitive_execution")
        integral = 0.0
        force_stable_steps = 0
        touch_lost_steps = 0
        for _ in range(max(60, int(0.7 / scene.dt))):
            fvec = scene.probe_force_vec() - force_baseline
            fn = float(abs(fvec[2]))
            ft = float(np.linalg.norm(fvec[:2]))
            integral = float(np.clip(integral + (target_force - fn) * scene.dt, -1, 1))
            extension = float(
                np.clip(
                    extension + 0.00028 * (target_force - fn) + 0.00008 * integral,
                    0.0,
                    0.17,
                )
            )
            scene.command(probe=extension)
            scene.step(1)
            z = float(scene.probe_tip_pos()[2])
            indentation = max(contact_z - z, 0.0)
            compression = max(float(object_z) - float(scene.object_pos(run.target)[2]), 0.0)
            touch = scene.probe_touch()
            fvec = scene.probe_force_vec() - force_baseline
            fn = float(abs(fvec[2]))
            ft = float(np.linalg.norm(fvec[:2]))
            trace.append((indentation, compression, fn, ft))
            run.sample(
                force_N=fn,
                touch_force_N=touch,
                probe_force_vector_N=fvec,
                indentation_m=indentation,
                compression_m=compression,
                compression_joint_q=float(
                    scene.data.qpos[
                        scene.joint_qadr[f"obj{run.target}_compress"]
                    ]
                ),
                lateral_force_N=ft,
            )
            if fn > force_limit:
                run.fail("force_limit")
                break
            if touch < 0.2 * contact_threshold and fn < 0.2 * target_force:
                touch_lost_steps += 1
            else:
                touch_lost_steps = 0
            if touch_lost_steps > 20:
                run.fail("lost_contact")
                break
            if fn > 0.5 * target_force and ft / max(fn, 1e-6) > lateral_ratio_limit:
                run.fail("excess_lateral_force")
                break
            contact_steps += 1
            if abs(fn - target_force) <= 0.12 * target_force:
                force_stable_steps += 1
            else:
                force_stable_steps = 0
            if compression >= depth or force_stable_steps >= 30:
                break

    run.enter("post_check")
    if len(trace) >= 3:
        probe_x = np.asarray([p[0] for p in trace])
        comp_x = np.asarray([p[1] for p in trace])
        force = np.asarray([p[2] for p in trace])
        x_fit = comp_x if np.ptp(comp_x) > 1e-5 else probe_x
        mask = (x_fit > 5e-5) & (force > contact_threshold)
        if mask.sum() >= 2 and np.ptp(x_fit[mask]) > 1e-5:
            k_est = float(np.polyfit(x_fit[mask], force[mask], 1)[0])
        else:
            k_est = 0.0
        peak = float(force.max())
        indentation_max = float(probe_x.max())
        compression_max = float(comp_x.max())
    else:
        k_est = peak = indentation_max = compression_max = 0.0

    effective = compression_max if compression_max > 1e-5 else indentation_max
    if k_est <= 0 and effective > 1e-6 and peak > 1e-6:
        k_est = peak / effective
    compliance = effective / peak * 1000.0 if peak > 1e-6 else 0.0
    path_fraction = min(effective / max(depth, 1e-9), 1.0)
    run.quality["path_completion_ratio"] = path_fraction
    run.quality["peak_force_N"] = peak
    run.quality["target_force_ratio"] = peak / max(target_force, 1e-9)
    # Poke is force-controlled: a stiff object is expected to reach the requested
    # force before the nominal maximum depth.  Depth is a safety/feature bound,
    # not a required path length.
    if peak < 0.80 * target_force and path_fraction < 0.80:
        run.fail("target_force_not_reached")
    if len(trace) < 3:
        run.fail("insufficient_samples")
    valid = not run.violations

    _safe_retreat(run, release=False)
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "k_est_N_per_m": max(k_est, 0.0) if valid else 0.0,
            "k_est_N_per_mm": max(k_est / 1000.0, 0.0) if valid else 0.0,
            "compliance_mm_per_N": max(compliance, 0.0) if valid else 0.0,
            "peak_force_N": peak,
            "indentation_mm": indentation_max * 1000.0,
            "compression_mm": compression_max * 1000.0,
        },
        contact_seconds=contact_steps * scene.dt,
        params={
            "depth": depth,
            "target_force": target_force,
            "force_limit": force_limit,
        },
        raw_summary={
            "n_samples": len(trace),
            "contact_z": contact_z,
            "force_baseline": force_baseline,
        },
        valid=valid,
    )


def heft(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    lift_height: float = 0.025,
    hold_time: float = 0.45,
    osc_amp: float = 0.001,
    osc_freq: float = 1.5,
    penetration_limit: float = 0.0055,
    min_grasp_force: float | None = None,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(backend, "heft", int(target))
    scene = run.scene
    required_force = (
        float(min_grasp_force)
        if min_grasp_force is not None
        else (8.0 if backend.name == "allegro" else 3.0)
    )
    grasp = _prepare_grasp(
        run,
        penetration_limit=penetration_limit,
        min_grasp_force=required_force,
    )
    valid, snapshot = _lift_and_gate(
        run,
        grasp,
        lift_height=lift_height,
        penetration_limit=penetration_limit,
    )

    force_trace = []
    if valid:
        z0 = _ctrl(scene, "act_wz")
        n = max(1, int(hold_time / scene.dt))
        invalid_contact_steps = 0
        for k in range(n):
            t = k * scene.dt
            scene.command(z=z0 + osc_amp * np.sin(2 * np.pi * osc_freq * t))
            scene.step(1)
            sample = scene.wrist_force_vec() - grasp.baseline_force
            force_trace.append(sample.copy())
            run.sample(wrist_force_delta_N=sample)
            current = scene.contact_snapshot(run.target)
            if current.support_contact or current.table_contact:
                valid = False
                run.fail("support_contact_during_measurement")
                break
            if not scene.has_opposing_grasp(current):
                invalid_contact_steps += 1
            else:
                invalid_contact_steps = 0
            if invalid_contact_steps > 20:
                valid = False
                run.fail("grasp_lost_during_measurement")
                break
    if force_trace:
        force_arr = np.asarray(force_trace)
        # Sensor sign depends on the welded child convention, so use the stable
        # gravity-axis magnitude after baseline subtraction.
        weight_signal = float(abs(np.median(force_arr[:, 2]))) if valid else 0.0
        force_std = float(np.std(force_arr[:, 2]))
    else:
        weight_signal = force_std = 0.0
    m_est = weight_signal / 9.81

    _safe_retreat(run, release=True)
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "m_est_kg": m_est,
            "weight_signal_N": weight_signal,
            "Fz_delta_median_N": weight_signal,
            "Fz_delta_std_N": force_std,
            "lifted": float(valid),
            "hand_contact_group_count": float(len(snapshot.hand_groups)),
        },
        contact_seconds=len(force_trace) * scene.dt,
        params={
            "lift_height": lift_height,
            "hold_time": hold_time,
            "osc_amp": osc_amp,
            "penetration_limit": penetration_limit,
        },
        raw_summary={
            "baseline_wrist_force": grasp.baseline_force,
            "baseline_wrist_torque": grasp.baseline_torque,
            "close_alpha": grasp.close_alpha,
        },
        valid=valid,
    )


def shake(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    lift_height: float = 0.025,
    tilt_amp: float = 0.07,
    yaw_amp: float = 0.06,
    freq: float = 1.2,
    duration: float = 0.8,
    penetration_limit: float = 0.0055,
    min_grasp_force: float | None = None,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(backend, "shake", int(target))
    scene = run.scene
    required_force = (
        float(min_grasp_force)
        if min_grasp_force is not None
        else (8.0 if backend.name == "allegro" else 3.0)
    )
    grasp = _prepare_grasp(
        run,
        penetration_limit=penetration_limit,
        min_grasp_force=required_force,
    )
    heft_valid, snapshot = _lift_and_gate(
        run,
        grasp,
        lift_height=lift_height,
        penetration_limit=penetration_limit,
    )
    if not heft_valid:
        run.fail("heft_invalid")

    force_trace = []
    torque_trace = []
    valid = heft_valid
    if valid:
        run.enter("primitive_execution")
        n = max(1, int(duration / scene.dt))
        invalid_contact_steps = 0
        for k in range(n):
            t = k * scene.dt
            scene.command(
                tilt=tilt_amp * np.sin(2 * np.pi * freq * t),
                yaw=yaw_amp * np.sin(2 * np.pi * freq * t + np.pi / 2),
            )
            scene.step(1)
            force = scene.wrist_force_vec() - grasp.baseline_force
            torque = scene.wrist_torque_vec() - grasp.baseline_torque
            force_trace.append(force.copy())
            torque_trace.append(torque.copy())
            run.sample(wrist_force_delta_N=force, wrist_torque_delta_Nm=torque)
            current = scene.contact_snapshot(run.target)
            if current.support_contact or current.table_contact:
                valid = False
                run.fail("support_contact_during_shake")
                break
            if not scene.has_opposing_grasp(current):
                invalid_contact_steps += 1
            else:
                invalid_contact_steps = 0
            if invalid_contact_steps > 20:
                valid = False
                run.fail("lost_grasp_during_shake")
                break

    run.enter("post_check")
    if force_trace:
        force_arr = np.asarray(force_trace)
        torque_arr = np.asarray(torque_trace)
        weight_proxy = float(abs(np.median(force_arr[:, 2]))) if valid else 0.0
        torque_rms = np.sqrt(np.mean(torque_arr * torque_arr, axis=0))
        torque_norm = np.linalg.norm(torque_arr, axis=1)
        slosh_proxy = float(np.std(torque_arr[:, 0]) + np.std(torque_arr[:, 1]))
        torque_peak = float(np.max(torque_norm))
        fill_proxy = torque_peak + 0.02 * weight_proxy
    else:
        weight_proxy = slosh_proxy = torque_peak = fill_proxy = 0.0
        torque_rms = np.zeros(3)

    _safe_retreat(run, release=True)
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "weight_proxy_N": weight_proxy,
            "fill_proxy": fill_proxy if valid else 0.0,
            "slosh_proxy": slosh_proxy if valid else 0.0,
            "torque_peak_Nm": torque_peak,
            "torque_rms_x_Nm": float(torque_rms[0]),
            "torque_rms_y_Nm": float(torque_rms[1]),
            "torque_rms_z_Nm": float(torque_rms[2]),
            "lifted": float(heft_valid),
            "hand_contact_group_count": float(len(snapshot.hand_groups)),
        },
        contact_seconds=len(force_trace) * scene.dt,
        params={
            "lift_height": lift_height,
            "tilt_amp": tilt_amp,
            "yaw_amp": yaw_amp,
            "freq": freq,
            "penetration_limit": penetration_limit,
        },
        raw_summary={
            "baseline_wrist_force": grasp.baseline_force,
            "baseline_wrist_torque": grasp.baseline_torque,
            "close_alpha": grasp.close_alpha,
            "heft_valid": heft_valid,
        },
        valid=valid,
    )


def slide(
    executor: ProbeBackend | AllegroProbeScene,
    target: int,
    preload: float = 2.0,
    distance: float = 0.040,
    duration: float = 0.8,
    force_limit: float = 12.0,
    recovery_steps: int = 30,
) -> ProbeResult:
    backend = as_backend(executor)
    run = _Run(backend, "slide", int(target))
    scene = run.scene
    x0 = scene.candidate_x(run.target)
    start_x = x0 - 0.5 * distance

    run.enter("approach")
    _probe_above(run, x=start_x, clearance=0.030)
    force_baseline = _mean_vec(scene, scene.probe_force_vec, 40)

    run.enter("guarded_contact")
    extension = 0.0
    contacted = False
    for extension in np.linspace(0.0, 0.17, 120):
        scene.command(probe=float(extension))
        scene.step(4)
        fn = scene.probe_touch()
        if fn > force_limit:
            run.fail("force_limit")
            break
        if fn >= 0.8 * preload:
            contacted = True
            break
    if not contacted and not run.violations:
        run.fail("no_contact")

    run.enter("contact_quality_gate")
    integral = 0.0
    lost_count = 0
    trace = []
    completed_steps = 0
    n = max(4, int(duration / scene.dt))
    if contacted and not run.violations:
        run.enter("primitive_execution")
        for k, alpha in enumerate(np.linspace(0.0, 1.0, n)):
            fn = scene.probe_touch()
            error = preload - fn
            integral = float(np.clip(integral + error * scene.dt, -2.0, 2.0))
            extension = float(
                np.clip(extension + 0.00025 * error + 0.00008 * integral, 0.0, 0.17)
            )
            scene.command(x=start_x + float(alpha) * distance, probe=extension)
            scene.step(1)
            fn = scene.probe_touch()
            fvec = scene.probe_force_vec() - force_baseline
            ft = float(np.linalg.norm(fvec[:2]))
            if fn < 0.2 * preload:
                lost_count += 1
            else:
                lost_count = 0
            trace.append((fn, fvec.copy(), float(alpha)))
            run.sample(
                normal_force_N=fn,
                tangential_force_N=ft,
                path_fraction=float(alpha),
            )
            if fn > force_limit:
                run.fail("force_limit")
                break
            if lost_count > recovery_steps:
                run.fail("lost_contact")
                break
            completed_steps = k + 1

    run.enter("post_check")
    completion = completed_steps / max(n, 1)
    run.quality["path_completion_ratio"] = completion
    run.quality["max_lost_contact_steps"] = float(lost_count)
    if completion < 0.95:
        run.fail("path_incomplete")

    if trace:
        fn_arr = np.asarray([item[0] for item in trace])
        force_arr = np.asarray([item[1] for item in trace])
        ft_arr = np.linalg.norm(force_arr[:, :2], axis=1)
        mask = fn_arr > max(0.2 * preload, 1e-4)
        if mask.any():
            mu = float(np.median(ft_arr[mask] / (fn_arr[mask] + 1e-6)))
            ft_median = float(np.median(ft_arr[mask]))
            fn_median = float(np.median(fn_arr[mask]))
            vibration = float(np.std(fn_arr[mask]))
            contact_fraction = float(mask.mean())
        else:
            mu = ft_median = fn_median = vibration = contact_fraction = 0.0
    else:
        mu = ft_median = fn_median = vibration = contact_fraction = 0.0
    run.quality["contact_fraction"] = contact_fraction
    if contact_fraction < 0.80:
        run.fail("insufficient_contact_fraction")
    valid = not run.violations

    _safe_retreat(run, release=False)
    run.phase = "complete" if valid else "retreat"
    return run.result(
        features={
            "mu_est": max(mu, 0.0) if valid else 0.0,
            "friction_ratio": max(mu, 0.0) if valid else 0.0,
            "Ft_median_N": ft_median,
            "Fn_median_N": fn_median,
            "slide_vibration": vibration,
        },
        contact_seconds=len(trace) * scene.dt,
        params={
            "preload": preload,
            "distance": distance,
            "duration": duration,
            "recovery_steps": recovery_steps,
        },
        raw_summary={
            "n_samples": len(trace),
            "force_baseline": force_baseline,
        },
        valid=valid,
    )


_DISPATCH: Dict[str, Callable[..., ProbeResult]] = {
    "poke": poke,
    "heft": heft,
    "shake": shake,
    "slide": slide,
}


def run_probe(
    executor: ProbeBackend | AllegroProbeScene,
    primitive: str,
    target: int,
    **params: Any,
) -> ProbeResult:
    backend = as_backend(executor)
    primitive = str(primitive)
    if primitive not in _DISPATCH:
        raise ValueError(
            f"unknown probe primitive {primitive!r}; expected {sorted(_DISPATCH)}"
        )
    expected = primitive_for_family(backend.scene.task.family)
    if primitive != expected:
        raise ValueError(
            f"scene family {backend.scene.task.family!r} requires {expected!r}, "
            f"got {primitive!r}; collision roles are fixed at model compilation"
        )
    target = int(target)
    if not 0 <= target < backend.scene.n:
        raise IndexError(f"target {target} outside [0, {backend.scene.n})")
    return _DISPATCH[primitive](backend, target, **params)
