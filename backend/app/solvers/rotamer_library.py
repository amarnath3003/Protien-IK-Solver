"""
Rotamer-library-inspired conditional proposal distributions.

Biology basis: amino acid side chains occupy a small number of
empirically-favored conformations (rotamers), and modern rotamer
libraries are BACKBONE-DEPENDENT -- the preferred side-chain dihedral
depends on the local backbone conformation. Rotamer libraries are used
as PROPOSAL MOVES inside global optimization (basin-hopping), not as a
hard discretization of the energy function -- they bias *where* a
stochastic search looks, without restricting what it can ultimately
reach.

This module builds the literal computational analog for a kinematic
chain: for each adjacent joint pair (i-1, i), an empirical, TARGET-
INDEPENDENT distribution of "what angle does joint i tend to take, given
joint i-1's current angle, in well-conditioned (high-manipulability,
collision-free, joint-limit-respecting) configurations". This is built
ONCE per process from a generic structural-quality criterion -- never
from any specific target pose, exactly as a real rotamer library is
built once from a structure database and then reused for any protein/
task, not re-derived per query.

This is a genuinely different mechanism from anything else in ProteinIK:
it doesn't change control flow (staging), doesn't change stuck-recovery
(rescue), and doesn't change problem decomposition (vectorial folding).
It changes WHAT THE RANDOM PROPOSAL DISTRIBUTION IS in stage 3's
perturbation step -- biasing search toward empirically-good neighbor
relationships instead of blind uniform noise.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

from app.core.kinematics import RobotSpec, geometric_jacobian, self_collision_min_distance
from app.solvers.protein_energy import joint_limit_energy


@dataclass
class RotamerLibrary:
    """For each adjacent joint pair (i, i+1), a binned conditional
    distribution: bin_centers[i] are the bin centers for joint i's angle,
    and for each bin, cond_mean[i][bin] / cond_std[i][bin] describe the
    empirical distribution of joint (i+1)'s angle among well-conditioned
    sampled configurations that fell in that bin.
    """
    n_bins: int
    bin_edges: list  # list of (n_bins+1,) arrays, one per joint
    cond_mean: list  # list of (n_bins,) arrays, one per joint pair (i -> i+1)
    cond_std: list   # list of (n_bins,) arrays
    global_std: list  # fallback std per joint pair if a bin has no data


def _quality_score(spec: RobotSpec, q: np.ndarray) -> float:
    """Target-independent structural quality: higher is better.
    Combines manipulability (avoid near-singular configs), self-collision
    distance (avoid steric clash), and joint-limit margin -- exactly the
    kind of generic 'is this conformation sterically/energetically
    reasonable' question a real rotamer library answers, independent of
    what the protein's specific function/target is.
    """
    J = geometric_jacobian(spec, q)
    manip = np.sqrt(max(np.linalg.det(J @ J.T), 0))
    self_dist = self_collision_min_distance(spec, q)
    limit_penalty = joint_limit_energy(spec, q)
    # combine: reward manipulability and self-distance, penalize limit closeness
    return manip * 5.0 + np.clip(self_dist, -0.1, 0.1) * 10.0 - limit_penalty * 0.1


def build_rotamer_library(
    spec: RobotSpec,
    rng: np.random.Generator,
    n_samples: int = 6000,
    n_bins: int = 12,
    keep_fraction: float = 0.2,
) -> RotamerLibrary:
    """Offline construction (run once, cached): sample many random valid
    configurations, score each by target-independent structural quality,
    keep the top fraction, and build per-joint-pair conditional
    distributions from the survivors. Mirrors how real rotamer libraries
    are built once from a database of resolved structures and then reused
    for any subsequent structure prediction, not rebuilt per query.
    """
    n = spec.n_joints
    samples = np.array([spec.random_config(rng) for _ in range(n_samples)])
    scores = np.array([_quality_score(spec, q) for q in samples])

    keep_n = max(50, int(n_samples * keep_fraction))
    good_idx = np.argsort(scores)[-keep_n:]
    good_samples = samples[good_idx]

    bin_edges = []
    cond_mean = []
    cond_std = []
    global_std = []

    for i in range(n - 1):
        lo, hi = spec.joint_limits[i]
        edges = np.linspace(lo, hi, n_bins + 1)
        bin_edges.append(edges)

        bin_idx = np.clip(np.digitize(good_samples[:, i], edges) - 1, 0, n_bins - 1)
        means = np.zeros(n_bins)
        stds = np.zeros(n_bins)
        overall_std = float(np.std(good_samples[:, i + 1])) + 1e-3
        for b in range(n_bins):
            vals = good_samples[bin_idx == b, i + 1]
            if len(vals) >= 3:
                means[b] = np.mean(vals)
                stds[b] = max(np.std(vals), 0.05)
            else:
                # not enough data in this bin -- fall back to global mean/std
                means[b] = np.mean(good_samples[:, i + 1]) if len(good_samples) else 0.0
                stds[b] = overall_std
        cond_mean.append(means)
        cond_std.append(stds)
        global_std.append(overall_std)

    return RotamerLibrary(
        n_bins=n_bins, bin_edges=bin_edges,
        cond_mean=cond_mean, cond_std=cond_std, global_std=global_std,
    )


def propose_conditional(
    library: RotamerLibrary, spec: RobotSpec, q: np.ndarray, joint_i: int,
    rng: np.random.Generator, search_radius: float,
) -> float:
    """Propose a candidate angle for joint_i, biased by the rotamer
    library's empirical conditional distribution given joint (i-1)'s
    CURRENT value, instead of blind uniform noise. Falls back to uniform
    proposal for joint 0 (no predecessor to condition on) and blends with
    the search_radius-scaled uniform proposal so the bias narrows
    naturally as the funnel search radius shrinks (consistent with the
    rest of stage 3's narrowing-search character), rather than the bias
    being either all-or-nothing.
    """
    n = spec.n_joints
    if joint_i == 0:
        return float(np.clip(
            q[joint_i] + rng.uniform(-search_radius, search_radius),
            spec.joint_limits[joint_i, 0], spec.joint_limits[joint_i, 1],
        ))

    pair_idx = joint_i - 1  # library indexes by (i-1 -> i) for i in 1..n-1
    edges = library.bin_edges[pair_idx]
    b = int(np.clip(np.digitize(q[joint_i - 1], edges) - 1, 0, library.n_bins - 1))
    lib_mean = library.cond_mean[pair_idx][b]
    lib_std = min(library.cond_std[pair_idx][b], search_radius * 2.0 + 1e-6)

    # blend: a fraction of proposals come from the library bias, the rest
    # stay uniform local noise around the CURRENT value -- preserves
    # stage 3's existing local-search character while injecting bias,
    # rather than fully replacing it (a real rotamer library biases
    # sampling, it doesn't eliminate the rest of conformational space).
    if rng.uniform() < 0.6:
        cand = rng.normal(lib_mean, lib_std)
    else:
        cand = q[joint_i] + rng.uniform(-search_radius, search_radius)

    return float(np.clip(cand, spec.joint_limits[joint_i, 0], spec.joint_limits[joint_i, 1]))


_CACHED_LIBRARY: dict = {}


def get_or_build_library(spec: RobotSpec) -> RotamerLibrary:
    """Process-level cache so the (relatively expensive, ~6000-sample)
    library is built once and reused across all solves -- mirrors a real
    rotamer library being a fixed, pre-built resource, not something
    recomputed per protein."""
    key = spec.name
    if key not in _CACHED_LIBRARY:
        build_rng = np.random.default_rng(42)  # fixed seed: deterministic, reproducible library
        _CACHED_LIBRARY[key] = build_rotamer_library(spec, build_rng)
    return _CACHED_LIBRARY[key]
