#!/usr/bin/env python3
"""Bayesian risk model for smooth climbing-robot motion planning.

The current planner reduces its commanded speed using two independent,
piecewise-linear mechanisms:

* ``edge_motion_scale`` - a hard linear ramp on the minimum edge
  clearance, which collapses to zero speed as soon as the robot is
  "inside" the surface polygon.
* ``boundary_hold`` - a boolean gate that cuts the speed to zero when
  the adhesion ``boundary_risk`` exceeds ``boundary_hold_risk``.

Both mechanisms are *piecewise* and have a single hard cliff, so the
robot snaps between full speed and zero speed at the boundary.  This
produces jerky motion on the horizontal pipe test (the robot is asked
to curve over the top of the pipe, where the tangential gravity load
rises quickly and the magnetic contact fraction drops as the wheels
ride over the crown).

This module implements a small, *training-free* Bayesian model that
replaces the hard cliffs with a smooth posterior probability of safety
$P(\\mathrm{safe} \\mid \\mathbf{x})$ and a smooth speed scaler derived
from that probability.

## Model

We use a **Gaussian naive Bayes** classifier over a binary latent
state $S \\in \\{\\mathrm{safe}, \\mathrm{unsafe}\\}$.

The observation vector is

$$
\\mathbf{x} = (x_1, x_2, x_3, x_4, x_5, x_6)
$$

where every component is normalized so that **larger is safer**:

* $x_1 = \\mathrm{force\\_margin}$ - friction/torque capacity divided
  by the safety-factored required tangential load.  Values $\\ge 1$
  mean the robot can hold; $<1$ means it is slipping.
* $x_2 = \\mathrm{contact\\_fraction}$ - the wheel-adhesion module's
  reported average contact fraction in $[0, 1]$.
* $x_3 = 1 - \\mathrm{boundary\\_risk}$ - the adhesion boundary risk
  flipped so larger is safer.
* $x_4 = \\mathrm{edge\\_clearance}/\\mathrm{edge\\_margin}$,
  clipped to $[0, 1]$.
* $x_5 = 1 - \\mathrm{load\\_fraction}$ - the fraction of the gravity
  load that is tangential to the surface (i.e. $1 - \\sqrt{1-n_z^2}$
  in surface-normal coordinates, with larger = flatter / safer).
* $x_6 = 1 - \\mathrm{steering\\_change\\_norm}$ - normalized steering
  stability, where larger means the next command keeps the swerve wheel
  angles close to the previous useful command.

For each class $c \\in \\{\\mathrm{safe}, \\mathrm{unsafe}\\}$ the
likelihood of an observation is a product of one-dimensional Gaussians

$$
P(\\mathbf{x} \\mid c) = \\prod_{i=1}^{6}
    \\frac{1}{\\sqrt{2\\pi}\\,\\sigma_{i,c}}
    \\exp\\!\\left(-\\frac{(x_i - \\mu_{i,c})^2}{2\\sigma_{i,c}^2}\\right).
$$

The class means encode the *typical* values of each feature under the
two classes (e.g. $\\mu_{\\mathrm{safe}} = (1.5, 0.9, 0.95, 0.9, 0.95, 0.95)$
and $\\mu_{\\mathrm{unsafe}} = (0.2, 0.1, 0.2, 0.1, 0.1, 0.15)$); the sigmas
encode how much variation in each feature is still consistent with
each class.  We work in the log domain for numerical stability:

$$
\\log P(c \\mid \\mathbf{x}) = \\log P(c) + \\sum_{i=1}^{6}
    \\left[ -\\frac{(x_i - \\mu_{i,c})^2}{2\\sigma_{i,c}^2}
              -\\log(\\sqrt{2\\pi}\\,\\sigma_{i,c}) \\right]
$$

and the final posterior is the softmax over the two classes.

The model is intentionally **configurable** (every mean, sigma, and
the prior can be set from ROS parameters) but ships with physically
motivated defaults that match the existing safety-factor and
contact-fraction gates.  The defaults are tuned so that a feature
vector near the unsafe means returns $P(\\mathrm{safe}) \\le 0.05$ and
a feature vector near the safe means returns $P(\\mathrm{safe}) \\ge
0.95$.

## Smoothing

The posterior is mapped to a speed scale with a **smoothstep**:

$$
s = s_{\\min} + (1 - s_{\\min}) \\cdot
    \\mathrm{smoothstep}(P, P_{\\mathrm{low}}, P_{\\mathrm{high}})
$$

where $\\mathrm{smoothstep}(x, a, b) = 0$ for $x \\le a$, $1$ for $x
\\ge b$, and $3t^2 - 2t^3$ with $t = (x-a)/(b-a)$ in between.  This
gives a C^1-continuous ramp with no discontinuities.

Two emergency gates are exposed alongside the smooth scale:

* ``hold`` - True if the posterior is below ``hold_threshold``
  (defaults to 0.5).  Callers that prefer a hard "stop completely"
  gate can use this in place of a multiplicative scale.
* ``emergency_hold`` - True only if the robot is *clearly* unsafe on
  any single feature (e.g. force_margin < 0.5).  This is a fast
  safety net independent of the smoother posterior, mirroring the
  role of the original ``boundary_hold`` for catastrophic cases.

The class also exposes a ``describe()`` helper that returns a
JSON-serializable dict with the posterior, the per-feature log
likelihood contributions, and the resulting speed scale; the planner
forwards that dict into its debug topic so an operator can introspect
the model in real time.

The module is self-contained and has no ROS dependency, which makes
it trivial to unit-test with plain pytest.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional


# ---------------------------------------------------------------------------
# Numerical helpers
# ---------------------------------------------------------------------------


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _safe_log(x: float) -> float:
    return math.log(max(x, 1e-300))


def _log_gaussian(x: float, mu: float, sigma: float) -> float:
    """Return ``log N(x; mu, sigma)`` without the constant, capped safely."""
    if sigma <= 1e-9:
        # Effectively a delta.  Return 0 if the value matches, -inf
        # otherwise.  Using a generous tolerance so floating-point
        # equality works in practice.
        return 0.0 if abs(x - mu) < 1e-6 else -1e9
    diff = (x - mu) / sigma
    return -0.5 * diff * diff - _safe_log(sigma)


def _smoothstep(x: float, edge0: float, edge1: float) -> float:
    """Hermite smoothstep, monotonic and C^1-continuous."""
    if edge1 - edge0 <= 1e-12:
        return 1.0 if x >= edge1 else 0.0
    t = _clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


#: Names of the observation features, in the order the model
#: expects them in its per-class mean/sigma vectors.
_FEATURE_NAMES = (
    "force_margin",
    "contact_fraction",
    "safety_pressure",
    "edge_clearance_norm",
    "level_pressure",
    "steering_stability",
)


@dataclass
class BayesianMotionConfig:
    """Hyperparameters of the Bayesian motion model.

    All values can be overridden at runtime via ROS parameters; see
    :class:`BayesianMotionModel.from_mapping`.
    """

    # Priors on the two classes.  They need not sum to 1 - we
    # renormalize inside the model.
    prior_safe: float = 0.7
    prior_unsafe: float = 0.3

    # Means per class, per feature.  Order is
    # (force_margin, contact_fraction, 1 - boundary_risk,
    #  edge_clearance_norm, 1 - load_fraction, steering_stability).
    mean_safe: tuple = (1.5, 0.90, 0.95, 0.90, 0.95, 0.95)
    mean_unsafe: tuple = (0.20, 0.10, 0.20, 0.10, 0.10, 0.15)

    # Sigmas per class, per feature.  Larger sigma means the model
    # tolerates more variation in that feature for that class.  The
    # defaults express the fact that the "safe" class is much stricter
    # about force margin than about edge clearance (which can be small
    # without being unsafe if other features are healthy).
    sigma_safe: tuple = (0.50, 0.20, 0.20, 0.30, 0.25, 0.20)
    sigma_unsafe: tuple = (0.35, 0.25, 0.25, 0.25, 0.25, 0.30)

    # Posterior -> speed scale mapping.  P >= ramp_high -> full speed;
    # P <= ramp_low -> min_scale.  Between the two, smoothstep.
    ramp_low: float = 0.30
    ramp_high: float = 0.85
    min_scale: float = 0.10

    # Thresholds for the two emergency gates.
    hold_threshold: float = 0.50
    emergency_force_margin: float = 0.50
    emergency_contact_fraction: float = 0.05

    def __post_init__(self):
        if self.prior_safe <= 0.0 or self.prior_unsafe <= 0.0:
            raise ValueError("Priors must be strictly positive.")
        for name, seq in (
            ("mean_safe", self.mean_safe),
            ("mean_unsafe", self.mean_unsafe),
            ("sigma_safe", self.sigma_safe),
            ("sigma_unsafe", self.sigma_unsafe),
        ):
            if len(seq) != len(_FEATURE_NAMES):
                raise ValueError(
                    f"{name} must have {len(_FEATURE_NAMES)} entries, "
                    f"got {len(seq)}"
                )
            if name.startswith("sigma") and any(s <= 0.0 for s in seq):
                raise ValueError(f"{name} entries must be positive.")
        if not (0.0 < self.min_scale <= 1.0):
            raise ValueError("min_scale must be in (0, 1].")
        if not (0.0 < self.ramp_low < self.ramp_high <= 1.0):
            raise ValueError(
                "ramp_low/ramp_high must satisfy 0 < low < high <= 1."
            )
        if not (0.0 < self.hold_threshold <= 1.0):
            raise ValueError("hold_threshold must be in (0, 1].")


# ---------------------------------------------------------------------------
# Result types
# ---------------------------------------------------------------------------


@dataclass
class BayesianMotionObservation:
    """The features consumed by the model, in the model's order."""

    force_margin: float
    contact_fraction: float
    safety_pressure: float
    edge_clearance_norm: float
    level_pressure: float
    steering_stability: float = 1.0

    def as_vector(self) -> tuple:
        return (
            float(self.force_margin),
            _clamp(float(self.contact_fraction), 0.0, 1.0),
            _clamp(float(self.safety_pressure), 0.0, 1.0),
            _clamp(float(self.edge_clearance_norm), 0.0, 1.0),
            _clamp(float(self.level_pressure), 0.0, 1.0),
            _clamp(float(self.steering_stability), 0.0, 1.0),
        )


@dataclass
class BayesianMotionDecision:
    """The model's output for a single observation."""

    posterior_safe: float
    speed_scale: float
    hold: bool
    emergency_hold: bool
    log_likelihoods: Dict[str, float] = field(default_factory=dict)
    feature_log_likelihoods: Dict[str, float] = field(default_factory=dict)

    def describe(self) -> Dict[str, Any]:
        return {
            "posterior_safe": float(self.posterior_safe),
            "speed_scale": float(self.speed_scale),
            "hold": bool(self.hold),
            "emergency_hold": bool(self.emergency_hold),
            "log_likelihoods": {
                k: float(v) for k, v in self.log_likelihoods.items()
            },
            "feature_log_likelihoods": {
                k: float(v) for k, v in self.feature_log_likelihoods.items()
            },
        }


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


class BayesianMotionModel:
    """Smooth Bayesian speed-scaler for the climbing planner."""

    def __init__(self, config: Optional[BayesianMotionConfig] = None):
        # ``__post_init__`` already validated the config if the caller
        # built it directly; if they passed a dict-like we validate
        # again here.
        self.config = config or BayesianMotionConfig()

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------

    @classmethod
    def from_mapping(cls, params: Mapping[str, Any]) -> "BayesianMotionModel":
        """Build a model from a flat dict of overrides.

        Unknown keys are ignored so the caller can pass the full
        ROS parameter namespace safely.
        """
        cfg = BayesianMotionConfig()
        # Priors.
        if "prior_safe" in params and params["prior_safe"] is not None:
            cfg.prior_safe = float(params["prior_safe"])
        if "prior_unsafe" in params and params["prior_unsafe"] is not None:
            cfg.prior_unsafe = float(params["prior_unsafe"])
        # Per-class per-feature parameters, flattened in the form
        # ``<prefix>_<feature>`` e.g. ``mean_safe_force_margin``.
        for prefix, target in (
            ("mean_safe", "mean_safe"),
            ("mean_unsafe", "mean_unsafe"),
            ("sigma_safe", "sigma_safe"),
            ("sigma_unsafe", "sigma_unsafe"),
        ):
            base = list(getattr(cfg, target))
            mutated = False
            for i, name in enumerate(_FEATURE_NAMES):
                key = f"{prefix}_{name}"
                if key in params and params[key] is not None:
                    base[i] = float(params[key])
                    mutated = True
            if mutated:
                setattr(cfg, target, tuple(base))
        # Smooth-step / emergency parameters.
        for key in (
            "ramp_low",
            "ramp_high",
            "min_scale",
            "hold_threshold",
            "emergency_force_margin",
            "emergency_contact_fraction",
        ):
            if key in params and params[key] is not None:
                setattr(cfg, key, float(params[key]))
        return cls(cfg)

    def update(self, **overrides: Any) -> "BayesianMotionModel":
        """Return a new model with selected parameters overridden."""
        new_cfg = BayesianMotionConfig(
            prior_safe=overrides.get("prior_safe", self.config.prior_safe),
            prior_unsafe=overrides.get(
                "prior_unsafe", self.config.prior_unsafe),
            mean_safe=overrides.get("mean_safe", self.config.mean_safe),
            mean_unsafe=overrides.get(
                "mean_unsafe", self.config.mean_unsafe),
            sigma_safe=overrides.get("sigma_safe", self.config.sigma_safe),
            sigma_unsafe=overrides.get(
                "sigma_unsafe", self.config.sigma_unsafe),
            ramp_low=overrides.get("ramp_low", self.config.ramp_low),
            ramp_high=overrides.get("ramp_high", self.config.ramp_high),
            min_scale=overrides.get("min_scale", self.config.min_scale),
            hold_threshold=overrides.get(
                "hold_threshold", self.config.hold_threshold),
            emergency_force_margin=overrides.get(
                "emergency_force_margin",
                self.config.emergency_force_margin),
            emergency_contact_fraction=overrides.get(
                "emergency_contact_fraction",
                self.config.emergency_contact_fraction),
        )
        return BayesianMotionModel(new_cfg)

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def _log_posteriors(
            self, observation: BayesianMotionObservation
            ) -> Dict[str, float]:
        x = observation.as_vector()
        log_priors = {
            "safe": _safe_log(self.config.prior_safe),
            "unsafe": _safe_log(self.config.prior_unsafe),
        }
        feature_contribs: Dict[str, float] = {}
        for i, name in enumerate(_FEATURE_NAMES):
            mu_s = self.config.mean_safe[i]
            mu_u = self.config.mean_unsafe[i]
            sg_s = self.config.sigma_safe[i]
            sg_u = self.config.sigma_unsafe[i]
            log_priors["safe"] += _log_gaussian(x[i], mu_s, sg_s)
            log_priors["unsafe"] += _log_gaussian(x[i], mu_u, sg_u)
            feature_contribs[name] = (
                _log_gaussian(x[i], mu_s, sg_s) -
                _log_gaussian(x[i], mu_u, sg_u)
            )
        return {**log_priors, "_features": feature_contribs}

    def decide(
        self, observation: BayesianMotionObservation
    ) -> BayesianMotionDecision:
        """Compute the posterior and speed scaler for one observation."""
        x = observation.as_vector()
        log_post = self._log_posteriors(observation)
        log_safe = log_post["safe"]
        log_unsafe = log_post["unsafe"]
        # Softmax to get a proper probability in (0, 1).  The shift
        # keeps the exponentials finite even when the log posteriors
        # are very large or very small.
        m = max(log_safe, log_unsafe)
        exp_safe = math.exp(log_safe - m)
        exp_unsafe = math.exp(log_unsafe - m)
        p_safe = exp_safe / (exp_safe + exp_unsafe)
        cfg = self.config
        scale = cfg.min_scale + (1.0 - cfg.min_scale) * _smoothstep(
            p_safe, cfg.ramp_low, cfg.ramp_high
        )
        hold = p_safe < cfg.hold_threshold
        emergency = (
            x[0] < cfg.emergency_force_margin or
            x[1] < cfg.emergency_contact_fraction
        )
        return BayesianMotionDecision(
            posterior_safe=p_safe,
            speed_scale=scale,
            hold=hold,
            emergency_hold=emergency,
            log_likelihoods={
                "log_p_safe": log_safe,
                "log_p_unsafe": log_unsafe,
            },
            feature_log_likelihoods=log_post["_features"],
        )

    # ------------------------------------------------------------------
    # Debug / introspection
    # ------------------------------------------------------------------

    def describe_config(self) -> Dict[str, Any]:
        cfg = self.config
        return {
            "prior_safe": cfg.prior_safe,
            "prior_unsafe": cfg.prior_unsafe,
            "mean_safe": list(cfg.mean_safe),
            "mean_unsafe": list(cfg.mean_unsafe),
            "sigma_safe": list(cfg.sigma_safe),
            "sigma_unsafe": list(cfg.sigma_unsafe),
            "ramp_low": cfg.ramp_low,
            "ramp_high": cfg.ramp_high,
            "min_scale": cfg.min_scale,
            "hold_threshold": cfg.hold_threshold,
            "emergency_force_margin": cfg.emergency_force_margin,
            "emergency_contact_fraction":
                cfg.emergency_contact_fraction,
            "feature_names": list(_FEATURE_NAMES),
        }


# ---------------------------------------------------------------------------
# Convenience helpers for the planner
# ---------------------------------------------------------------------------


def build_observation(
        *,
        force_margin: float,
        contact_fraction: float,
        boundary_risk: float,
        edge_clearance: float,
        edge_margin: float,
        load_fraction: float,
        steering_change_norm: float = 0.0,
        ) -> BayesianMotionObservation:
    """Convert planner-level quantities into a model observation.

    The function is a thin adapter that handles the sign flips and
    normalizations the model expects.  The planner calls this every
    tick with the current safety evaluation and edge projection
    results.
    """
    if edge_margin <= 0.0:
        edge_norm = 1.0
    elif not math.isfinite(edge_clearance):
        edge_norm = 1.0
    elif edge_clearance <= 0.0:
        edge_norm = 0.0
    else:
        edge_norm = _clamp(edge_clearance / edge_margin, 0.0, 1.0)
    return BayesianMotionObservation(
        force_margin=float(force_margin),
        contact_fraction=_clamp(float(contact_fraction), 0.0, 1.0),
        safety_pressure=_clamp(1.0 - float(boundary_risk), 0.0, 1.0),
        edge_clearance_norm=edge_norm,
        level_pressure=_clamp(1.0 - float(load_fraction), 0.0, 1.0),
        steering_stability=_clamp(1.0 - float(steering_change_norm), 0.0, 1.0),
    )


def serialize_decision(decision: BayesianMotionDecision) -> str:
    """JSON helper for the planner's debug topic."""
    return json.dumps(decision.describe())


__all__ = [
    "BayesianMotionConfig",
    "BayesianMotionModel",
    "BayesianMotionObservation",
    "BayesianMotionDecision",
    "build_observation",
    "serialize_decision",
]
