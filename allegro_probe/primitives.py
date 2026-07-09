"""Probe primitives and feature extraction."""

from __future__ import annotations

from typing import Callable, Dict, List, Tuple

import numpy as np

from allegro_probe.scene import AllegroProbeScene
from allegro_probe.models import ProbeResult, canonical_family


def primitive_for_family(family: str) -> str:
    fam = canonical_family(family)
    return {
        "stiffness": "poke",
        "mass": "heft",
        "fill": "shake",
        "material": "slide",
    }[fam]


def _mean_vec(scene: AllegroProbeScene, reader: Callable[[], np.ndarray], steps: int = 80) -> np.ndarray:
    vals = []
    for _ in range(int(steps)):
        scene.step(1)
        vals.append(np.asarray(reader(), dtype=float).copy())
    return np.mean(vals, axis=0)


def _ctrl(scene: AllegroProbeScene, actuator: str) -> float:
    return float(scene.data.ctrl[scene.act[actuator]])


def _probe_above(scene: AllegroProbeScene, target: int, x: float | None = None, clearance: float = 0.035) -> None:
    scene.set_probe_collision(True)
    x = scene.candidate_x(target) if x is None else float(x)
    top = scene.object_top_z(target)
    scene.command(x=x, y=0.0, z=scene.wz_for_tip_z(top + clearance, 0.0),
                  tilt=0.0, yaw=0.0, probe=0.0, grip=0.0)
    scene.step(140)


def _prepare_grasp(
    scene: AllegroProbeScene,
    target: int,
    grip: float,
    touch_threshold: float,
) -> Tuple[np.ndarray, float]:
    return _prepare_allegro_grasp(
        scene, target, grip=grip, touch_threshold=touch_threshold
    )


def _prepare_allegro_grasp(
    scene: AllegroProbeScene,
    target: int,
    grip: float,
    touch_threshold: float,
) -> Tuple[np.ndarray, float]:
    """Move the real Allegro fingertips around a probe-ready cylinder/cup.

    The constants are wrist-frame offsets measured from the Menagerie right hand in its
    cylinder pre-shape.  They keep the object between the three fingers and thumb without
    requiring IK or motion planning in this initial implementation.
    """

    scene.set_probe_collision(False)
    obj = scene.task.objects[target]
    x = scene.candidate_x(target)
    y = 0.020
    # With the Menagerie right-hand frame mounted under the wrist carriage, a z command
    # around -0.28 places the distal/tip links in the waist groove of the raised cans.
    # Computing from the current object height keeps fill cups and mass cans aligned.
    if obj.family == "fill" and x < -0.05:
        y = 0.040
        fingertip_z_from_wrist_cmd = 0.378
    else:
        fingertip_z_from_wrist_cmd = 0.409 if x < -0.05 else 0.389
    z_grasp = float(np.clip(
        scene.object_mid_z(target) - fingertip_z_from_wrist_cmd,
        -0.55,
        0.14,
    ))
    scene.command(x=x, y=y, z=min(z_grasp + 0.08, 0.02), tilt=0.0, yaw=0.0, probe=0.0, grip=0.0)
    scene.step(220)
    scene.command(z=z_grasp, grip=0.0)
    scene.step(220)

    touch = 0.0
    for g in np.linspace(0.0, grip, 48):
        scene.command(grip=float(g))
        scene.step(12)
        touch = max(touch, scene.finger_touch_total())
    scene.step(180)
    touch = max(touch, scene.finger_touch_total())
    baseline = _mean_vec(scene, scene.wrist_force_vec, 80)
    return baseline, float(touch)


def _release_to_safe(scene: AllegroProbeScene, target: int) -> None:
    scene.command(tilt=0.0, yaw=0.0, probe=0.0, z=0.10)
    scene.step(80)
    scene.command(grip=0.0)
    scene.step(100)
    scene.command(x=scene.candidate_x(target), y=0.0)
    scene.step(40)
    scene.set_probe_collision(True)


def poke(
    scene: AllegroProbeScene,
    target: int,
    depth: float = 0.006,
    force_limit: float = 10.0,
    contact_threshold: float = 0.05,
    object_motion_abort: float = 0.03,
) -> ProbeResult:
    _probe_above(scene, target, clearance=0.030)
    contact_z = None
    contact_top_z = None
    trace: List[Tuple[float, float, float]] = []
    status = "no_contact"
    contact_steps = 0

    for wp in np.linspace(0.0, 0.17, 95):
        scene.command(probe=float(wp))
        scene.step(6)
        f = scene.probe_touch()
        z = float(scene.probe_tip_pos()[2])
        if f >= contact_threshold:
            contact_steps += 6
            if contact_z is None:
                contact_z = z
                contact_top_z = float(scene.object_pos(target)[2])
            indentation = max(contact_z - z, 0.0)
            compression = max((contact_top_z or scene.object_pos(target)[2]) - float(scene.object_pos(target)[2]), 0.0)
            trace.append((indentation, compression, f))
            status = "ok"
            if indentation >= depth:
                break
        if f > force_limit:
            status = "force_limit"
            break
        if scene.task.family != "stiffness" and scene.object_displacement(target) > object_motion_abort:
            status = "object_moved"
            break

    scene.command(probe=0.0, z=0.10)
    scene.step(100)

    if len(trace) >= 3:
        probe_x = np.array([p[0] for p in trace], dtype=float)
        comp_x = np.array([p[1] for p in trace], dtype=float)
        f = np.array([p[2] for p in trace], dtype=float)
        x = comp_x if np.ptp(comp_x) > 1e-5 else probe_x
        mask = (x > 5e-5) & (f > contact_threshold)
        if mask.sum() >= 2 and np.ptp(x[mask]) > 1e-5:
            k_est = float(np.polyfit(x[mask], f[mask], 1)[0])
        else:
            k_est = 0.0
        peak = float(np.max(f))
        indentation_max = float(np.max(probe_x))
        compression_max = float(np.max(comp_x))
    else:
        k_est = 0.0
        peak = 0.0
        indentation_max = 0.0
        compression_max = 0.0

    effective_deflection = compression_max if compression_max > 1e-5 else indentation_max
    if k_est <= 0.0 and effective_deflection > 1e-6 and peak > 1e-6:
        k_est = peak / effective_deflection
    compliance = (effective_deflection / peak * 1000.0) if peak > 1e-6 else 0.0
    return ProbeResult(
        object_id=f"obj{target}",
        target=target,
        primitive="poke",
        status=status,
        features={
            "k_est_N_per_m": max(k_est, 0.0),
            "k_est_N_per_mm": max(k_est / 1000.0, 0.0),
            "compliance_mm_per_N": max(compliance, 0.0),
            "peak_force_N": peak,
            "indentation_mm": indentation_max * 1000.0,
            "compression_mm": compression_max * 1000.0,
        },
        contact_seconds=contact_steps * scene.dt,
        params={"depth": depth, "force_limit": force_limit},
        raw_summary={"n_samples": len(trace), "contact_z": contact_z},
    )


def heft(
    scene: AllegroProbeScene,
    target: int,
    lift_height: float = 0.035,
    hold_time: float = 0.45,
    osc_amp: float = 0.006,
    osc_freq: float = 2.0,
    grip: float = 0.064,
    touch_threshold: float = 0.10,
) -> ProbeResult:
    baseline, touch = _prepare_grasp(scene, target, grip=grip, touch_threshold=touch_threshold)
    status = "ok" if touch >= touch_threshold else "no_touch"
    z0 = _ctrl(scene, "act_wz")
    scene.command(z=z0 + lift_height)
    scene.step(220)

    lifted = scene.object_lifted(target, min_height=0.012)
    if lifted:
        status = "ok"
    elif status == "ok":
        status = "not_lifted"

    n = max(1, int(hold_time / scene.dt))
    force_trace = []
    z_trace = []
    for k in range(n):
        t = k * scene.dt
        scene.command(z=z0 + lift_height + osc_amp * np.sin(2.0 * np.pi * osc_freq * t))
        scene.step(1)
        force_trace.append(scene.wrist_force_vec().copy())
        z_trace.append(float(scene.sensor("wz_pos")[0]))

    force_arr = np.asarray(force_trace, dtype=float)
    fz_median = float(np.median(force_arr[:, 2]))
    fz_delta = force_arr[:, 2] - float(baseline[2])
    fz_delta_median = float(np.median(fz_delta))
    valid = status == "ok"
    weight_signal = max(fz_median, 0.0) if valid else 0.0
    m_est = weight_signal / 9.81
    _release_to_safe(scene, target)

    return ProbeResult(
        object_id=f"obj{target}",
        target=target,
        primitive="heft",
        status=status,
        features={
            "m_est_kg": m_est,
            "weight_signal_N": weight_signal,
            "Fz_median_N": fz_median,
            "Fz_delta_median_N": fz_delta_median,
            "Fz_std_N": float(np.std(fz_delta)),
            "lifted": float(lifted),
            "finger_touch_sum": float(touch),
        },
        contact_seconds=(220 + n) * scene.dt,
        params={"lift_height": lift_height, "hold_time": hold_time, "osc_amp": osc_amp},
        raw_summary={"baseline_wrist_force": baseline, "z_minmax": [min(z_trace), max(z_trace)]},
    )


def shake(
    scene: AllegroProbeScene,
    target: int,
    lift_height: float = 0.035,
    tilt_amp: float = 0.10,
    yaw_amp: float = 0.08,
    freq: float = 1.5,
    duration: float = 1.0,
    grip: float = 0.064,
    touch_threshold: float = 0.10,
) -> ProbeResult:
    baseline_force, touch = _prepare_grasp(scene, target, grip=grip, touch_threshold=touch_threshold)
    status = "ok" if touch >= touch_threshold else "no_touch"
    z0 = _ctrl(scene, "act_wz")
    scene.command(z=z0 + lift_height)
    scene.step(180)
    lifted = scene.object_lifted(target, min_height=0.010)
    if lifted:
        status = "ok"
    elif status == "ok":
        status = "not_lifted"

    n = max(1, int(duration / scene.dt))
    force_trace = []
    torque_trace = []
    tilt_trace = []
    yaw_trace = []
    for k in range(n):
        t = k * scene.dt
        scene.command(
            tilt=tilt_amp * np.sin(2.0 * np.pi * freq * t),
            yaw=yaw_amp * np.sin(2.0 * np.pi * freq * t + np.pi / 2.0),
        )
        scene.step(1)
        force_trace.append(scene.wrist_force_vec().copy())
        torque_trace.append(scene.wrist_torque_vec().copy())
        tilt_trace.append(float(scene.sensor("wt_pos")[0]))
        yaw_trace.append(float(scene.sensor("wyaw_pos")[0]))

    force_arr = np.asarray(force_trace, dtype=float)
    torque_arr = np.asarray(torque_trace, dtype=float)
    fz_delta = force_arr[:, 2] - float(baseline_force[2])
    weight_proxy = float(abs(np.median(fz_delta)))
    torque_rms = np.sqrt(np.mean(torque_arr * torque_arr, axis=0))
    torque_norm = np.linalg.norm(torque_arr, axis=1)
    slosh_proxy = float(np.std(torque_arr[:, 0]) + np.std(torque_arr[:, 1]))
    torque_peak = float(np.max(torque_norm))
    fill_proxy = torque_peak + 0.02 * weight_proxy
    _release_to_safe(scene, target)

    return ProbeResult(
        object_id=f"obj{target}",
        target=target,
        primitive="shake",
        status=status,
        features={
            "weight_proxy_N": weight_proxy,
            "weight_signal_N": weight_proxy,
            "fill_proxy": fill_proxy,
            "slosh_proxy": slosh_proxy,
            "torque_peak_Nm": torque_peak,
            "torque_rms_x_Nm": float(torque_rms[0]),
            "torque_rms_y_Nm": float(torque_rms[1]),
            "torque_rms_z_Nm": float(torque_rms[2]),
            "lifted": float(lifted),
            "finger_touch_sum": float(touch),
        },
        contact_seconds=(180 + n) * scene.dt,
        params={"lift_height": lift_height, "tilt_amp": tilt_amp, "yaw_amp": yaw_amp, "freq": freq},
        raw_summary={
            "baseline_wrist_force": baseline_force,
            "tilt_minmax": [min(tilt_trace), max(tilt_trace)],
            "yaw_minmax": [min(yaw_trace), max(yaw_trace)],
        },
    )


def slide(
    scene: AllegroProbeScene,
    target: int,
    preload: float = 2.0,
    distance: float = 0.040,
    force_limit: float = 12.0,
    object_motion_abort: float = 0.030,
) -> ProbeResult:
    x0 = scene.candidate_x(target)
    start_x = x0 - 0.5 * distance
    _probe_above(scene, target, x=start_x, clearance=0.030)
    force_baseline = _mean_vec(scene, scene.probe_force_vec, 40)

    status = "no_contact"
    wp = 0.0
    for wp in np.linspace(0.0, 0.17, 90):
        scene.command(probe=float(wp))
        scene.step(6)
        fn = scene.probe_touch()
        if fn >= preload:
            status = "ok"
            break
        if fn > force_limit:
            status = "force_limit"
            break

    trace = []
    contact_steps = 0
    if status == "ok":
        n = max(4, int(0.8 / scene.dt))
        for k, alpha in enumerate(np.linspace(0.0, 1.0, n)):
            fn = scene.probe_touch()
            wp = float(np.clip(wp + 0.0012 * (preload - fn), 0.0, 0.17))
            scene.command(x=start_x + alpha * distance, probe=wp)
            scene.step(1)
            fn = scene.probe_touch()
            fvec = scene.probe_force_vec() - force_baseline
            if fn < 0.2 * preload:
                status = "lost_contact"
                break
            if fn > force_limit:
                status = "force_limit"
                break
            if scene.object_displacement(target) > object_motion_abort:
                status = "object_moved"
                break
            contact_steps += 1
            trace.append((fn, fvec.copy()))

    scene.command(probe=0.0, z=0.10, x=x0)
    scene.step(100)

    if trace:
        fn_arr = np.array([p[0] for p in trace], dtype=float)
        f_arr = np.array([p[1] for p in trace], dtype=float)
        ft_arr = np.linalg.norm(f_arr[:, :2], axis=1)
        mask = fn_arr > max(0.2 * preload, 1e-4)
        if mask.any():
            mu = float(np.median(ft_arr[mask] / (fn_arr[mask] + 1e-6)))
            ft_median = float(np.median(ft_arr[mask]))
            fn_median = float(np.median(fn_arr[mask]))
            vib = float(np.std(fn_arr[mask]))
        else:
            mu = ft_median = fn_median = vib = 0.0
    else:
        mu = ft_median = fn_median = vib = 0.0

    raw_status = status
    if status == "lost_contact" and len(trace) >= 50:
        status = "ok"

    return ProbeResult(
        object_id=f"obj{target}",
        target=target,
        primitive="slide",
        status=status,
        features={
            "mu_est": max(mu, 0.0),
            "friction_ratio": max(mu, 0.0),
            "Ft_median_N": ft_median,
            "Fn_median_N": fn_median,
            "slide_vibration": vib,
        },
        contact_seconds=contact_steps * scene.dt,
        params={"preload": preload, "distance": distance},
        raw_summary={"n_samples": len(trace), "force_baseline": force_baseline, "raw_status": raw_status},
    )


_DISPATCH: Dict[str, Callable[..., ProbeResult]] = {
    "poke": poke,
    "heft": heft,
    "shake": shake,
    "slide": slide,
}


def run_probe(scene: AllegroProbeScene, primitive: str, target: int, **params) -> ProbeResult:
    primitive = str(primitive)
    if primitive not in _DISPATCH:
        raise ValueError(
            f"unknown probe primitive {primitive!r}; expected {sorted(_DISPATCH)}"
        )
    return _DISPATCH[primitive](scene, int(target), **params)
