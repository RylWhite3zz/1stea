"""Small synthetic scenes for exercising the four probe primitives."""

from __future__ import annotations

from typing import List

import numpy as np

from allegro_probe.models import ObjectSpec, ProbeSceneSpec, canonical_family


_INSTRUCTIONS = {
    "stiffness": "Find the softest visually matched block.",
    "mass": "Find the heaviest visually matched can.",
    "fill": "Find the under-filled opaque cup.",
    "material": "Find the highest-friction visually matched surface.",
}


def _sample_levels(
    rng: np.random.Generator, levels: List[float], count: int
) -> np.ndarray:
    replace = count > len(levels)
    values = rng.choice(np.asarray(levels, dtype=float), size=count, replace=replace)
    rng.shuffle(values)
    return values


def make_demo_scene(
    family: str = "mass",
    n_candidates: int = 3,
    seed: int = 0,
    *,
    track: str | None = None,
) -> ProbeSceneSpec:
    """Create visually matched primitives with different physical parameters."""

    family = canonical_family(family)
    if n_candidates < 2:
        raise ValueError("n_candidates must be at least 2")
    rng = np.random.default_rng(seed)
    objects: List[ObjectSpec] = []
    resolved_track = str(track or ("content_mobility" if family == "fill" else family))

    if family == "stiffness":
        values = _sample_levels(rng, [80.0, 260.0, 500.0, 900.0, 1400.0], n_candidates)
        for index, stiffness in enumerate(values):
            objects.append(
                ObjectSpec(
                    index=index,
                    family=family,
                    shape="compressible_box",
                    size=(0.026, 0.026, 0.018),
                    mass_kg=0.005,
                    stiffness_N_per_m=float(stiffness),
                    friction_mu=1.0,
                )
            )
    elif family == "mass":
        values = _sample_levels(rng, [0.10, 0.24, 0.30, 0.42, 0.62], n_candidates)
        for index, mass in enumerate(values):
            objects.append(
                ObjectSpec(
                    index=index,
                    family=family,
                    shape="short_can",
                    size=(0.030, 0.030, 0.036),
                    mass_kg=float(mass),
                    stiffness_N_per_m=900.0,
                    friction_mu=1.4,
                )
            )
    elif family == "fill":
        if resolved_track not in {"content_mobility", "fill_ratio"}:
            raise ValueError(
                "fill track must be 'content_mobility' or 'fill_ratio', "
                f"got {resolved_track!r}"
            )
        total_mass = 0.30
        if resolved_track == "content_mobility":
            definitions = [
                ("fixed", 0.012, 2.4, 1.20),
                ("damped", 0.012, 2.4, 1.20),
                ("mobile", 0.012, 2.4, 0.12),
            ]
            choice = rng.choice(
                len(definitions), size=n_candidates, replace=n_candidates > 3
            )
            if n_candidates <= 3:
                choice = rng.permutation(len(definitions))[:n_candidates]
            records = [
                (0.55, 0.16, *definitions[int(which)]) for which in choice
            ]
        else:
            levels = _sample_levels(rng, [0.25, 0.55, 0.90], n_candidates)
            records = []
            for fill_level in levels:
                liquid_mass = float(0.22 * fill_level)
                travel = float(
                    0.003 + 0.010 * (1.0 - abs(fill_level - 0.5) * 2.0)
                )
                records.append(
                    (float(fill_level), liquid_mass, "mobile", travel, 2.4, 0.18)
                )
        for index, record in enumerate(records):
            fill_level, liquid_mass, mobility, travel, natural_frequency, damping = record
            objects.append(
                ObjectSpec(
                    index=index,
                    family=family,
                    shape="opaque_cup",
                    size=(0.034, 0.034, 0.045),
                    # Total mass is deliberately identical across candidates;
                    # the fill/content task cannot be solved by heft alone.
                    mass_kg=total_mass,
                    stiffness_N_per_m=900.0,
                    friction_mu=1.5,
                    fill_level=float(fill_level),
                    liquid_mass_kg=liquid_mass,
                    slosh_range_m=float(travel),
                    rgba=(0.23, 0.24, 0.26, 1.0),
                    container_sealed=True,
                    content_mobility_class=str(mobility),
                    slosh_natural_frequency_Hz=float(natural_frequency),
                    slosh_damping_ratio=float(damping),
                    content_proxy_version="mass_spring_2d.v2",
                )
            )
    else:
        values = _sample_levels(rng, [0.22, 0.65, 0.85, 1.10, 1.55], n_candidates)
        for index, friction in enumerate(values):
            objects.append(
                ObjectSpec(
                    index=index,
                    family=family,
                    shape="surface_block",
                    size=(0.036, 0.026, 0.017),
                    mass_kg=0.30,
                    stiffness_N_per_m=1200.0,
                    friction_mu=float(friction),
                )
            )

    return ProbeSceneSpec(
        scene_id=(
            f"demo_{family}_{resolved_track}_{seed:06d}"
            if family == "fill"
            else f"demo_{family}_{seed:06d}"
        ),
        family=family,
        instruction=(
            "Find the opaque cup with the most mobile contents."
            if family == "fill" and resolved_track == "content_mobility"
            else _INSTRUCTIONS[family]
        ),
        objects=objects,
        seed=seed,
        track=resolved_track,
    )
