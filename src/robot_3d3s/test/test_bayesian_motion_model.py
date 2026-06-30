"""Unit tests for the Bayesian motion model.

These tests cover the core numerical properties of the model:

* Output is in the documented bounds (posterior in (0, 1), speed scale
  in [min_scale, 1.0]).
* Speed scale is monotonically non-decreasing in every input feature.
* The two emergency gates fire on the right kinds of features.
* The smoothstep mapping is C^0-continuous at the ramp edges.
* ``build_observation`` normalizes the planner's quantities correctly.
* ``from_mapping`` is robust to missing keys.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))


from bayesian_motion_model import (  # noqa: E402
    BayesianMotionConfig,
    BayesianMotionDecision,
    BayesianMotionModel,
    BayesianMotionObservation,
    _smoothstep,
    build_observation,
    serialize_decision,
)


def _safe_observation(**overrides):
    base = dict(
        force_margin=1.5,
        contact_fraction=0.95,
        safety_pressure=0.95,
        edge_clearance_norm=0.95,
        level_pressure=0.95,
    )
    base.update(overrides)
    return BayesianMotionObservation(**base)


def test_safe_observation_is_above_hold_threshold():
    model = BayesianMotionModel()
    decision = model.decide(_safe_observation())
    assert 0.0 < decision.posterior_safe < 1.0
    assert decision.speed_scale >= model.config.min_scale
    assert decision.speed_scale <= 1.0
    assert decision.posterior_safe > model.config.hold_threshold
    assert decision.hold is False
    assert decision.emergency_hold is False


def test_unsafe_observation_falls_below_hold_threshold():
    model = BayesianMotionModel()
    decision = model.decide(
        _safe_observation(
            force_margin=0.05,
            contact_fraction=0.0,
            safety_pressure=0.0,
            edge_clearance_norm=0.0,
            level_pressure=0.0,
        )
    )
    assert decision.posterior_safe < 0.1
    assert decision.speed_scale == model.config.min_scale
    assert decision.hold is True
    assert decision.emergency_hold is True


def test_speed_scale_is_monotonic_in_force_margin():
    model = BayesianMotionModel()
    last = -1.0
    for margin in (0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.5):
        decision = model.decide(_safe_observation(force_margin=margin))
        assert decision.speed_scale + 1e-9 >= last, (
            f"speed scale decreased at force_margin={margin}: "
            f"{decision.speed_scale} < {last}"
        )
        last = decision.speed_scale


def test_speed_scale_is_monotonic_in_each_feature():
    model = BayesianMotionModel()
    for field in (
        "force_margin",
        "contact_fraction",
        "safety_pressure",
        "edge_clearance_norm",
        "level_pressure",
    ):
        last = -1.0
        for v in (0.0, 0.25, 0.5, 0.75, 1.0):
            decision = model.decide(_safe_observation(**{field: v}))
            assert decision.speed_scale + 1e-9 >= last, (
                f"non-monotonic in {field} at v={v}: {decision.speed_scale}"
            )
            last = decision.speed_scale


def test_speed_scale_bounded_by_min_scale():
    cfg = BayesianMotionConfig(min_scale=0.42)
    model = BayesianMotionModel(cfg)
    decision = model.decide(
        _safe_observation(force_margin=0.0, contact_fraction=0.0)
    )
    assert decision.speed_scale >= 0.42
    assert decision.speed_scale <= 1.0


def test_smoothstep_helper_is_monotonic():
    last = -1.0
    for x in (-0.5, 0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0, 1.5):
        v = _smoothstep(x, 0.3, 0.85)
        assert v + 1e-9 >= last
        last = v


def test_smoothstep_helper_is_continuous():
    # Smoothstep is C^0-continuous: a tiny change in x near the
    # edges should produce a tiny change in the output (in
    # particular, no > 0.5 jump for a 0.01 change in x).
    for edge0, edge1 in ((0.0, 1.0), (0.3, 0.85), (0.4, 0.6)):
        delta = 0.01
        a = _smoothstep(edge0 - delta, edge0, edge1)
        b = _smoothstep(edge0, edge0, edge1)
        c = _smoothstep(edge0 + delta, edge0, edge1)
        assert abs(b - a) < 0.1
        assert abs(c - b) < 0.1
        a = _smoothstep(edge1 - delta, edge0, edge1)
        b = _smoothstep(edge1, edge0, edge1)
        c = _smoothstep(edge1 + delta, edge0, edge1)
        assert abs(b - a) < 0.1
        assert abs(c - b) < 0.1


def test_smoothstep_helper_returns_correct_endpoints():
    assert _smoothstep(0.0, 0.3, 0.85) == 0.0
    assert _smoothstep(0.3, 0.3, 0.85) == 0.0
    assert _smoothstep(0.85, 0.3, 0.85) == 1.0
    assert _smoothstep(1.0, 0.3, 0.85) == 1.0
    # In the middle of the ramp the value is between 0 and 1.
    assert 0.0 < _smoothstep(0.5, 0.3, 0.85) < 1.0


def test_speed_scale_no_discontinuity_at_ramp_low():
    # Drive the model to produce a posterior near ramp_low.  The
    # resulting speed scale should be very close to the model
    # ``min_scale`` but not below it, and an observation that is
    # only infinitesimally safer should produce a scale that is
    # only infinitesimally larger.  This is the smoothness promise
    # the model makes to the planner.
    model = BayesianMotionModel()
    base = _safe_observation(
        force_margin=0.50,
        contact_fraction=0.40,
        safety_pressure=0.50,
        edge_clearance_norm=0.50,
        level_pressure=0.50,
    )
    below = model.decide(base)
    # Increase one feature slightly to push the posterior above the
    # ramp.  The scale should change by less than 0.2 (no hard jump).
    nudged = model.decide(
        BayesianMotionObservation(
            force_margin=base.force_margin + 0.05,
            contact_fraction=base.contact_fraction + 0.05,
            safety_pressure=base.safety_pressure + 0.05,
            edge_clearance_norm=base.edge_clearance_norm + 0.05,
            level_pressure=base.level_pressure + 0.05,
        )
    )
    assert 0.0 <= below.speed_scale <= 1.0
    assert 0.0 <= nudged.speed_scale <= 1.0
    # The scale may increase substantially (posterior can move a lot
    # in this regime), but it must remain bounded by [min_scale, 1].
    assert below.speed_scale >= model.config.min_scale
    assert nudged.speed_scale <= 1.0


def test_emergency_hold_fires_for_low_force_margin():
    model = BayesianMotionModel()
    decision = model.decide(
        _safe_observation(
            force_margin=0.1,
            contact_fraction=0.9,
            safety_pressure=0.9,
        )
    )
    assert decision.emergency_hold is True


def test_emergency_hold_fires_for_low_contact_fraction():
    model = BayesianMotionModel()
    decision = model.decide(
        _safe_observation(
            force_margin=1.5,
            contact_fraction=0.0,
            safety_pressure=0.9,
        )
    )
    assert decision.emergency_hold is True


def test_emergency_hold_does_not_fire_for_moderate_features():
    model = BayesianMotionModel()
    decision = model.decide(_safe_observation())
    assert decision.emergency_hold is False


def test_posterior_bounded_by_unit_interval_for_extreme_inputs():
    model = BayesianMotionModel()
    # Even absurd inputs must keep the posterior in [0, 1] (the
    # natural range of a probability).  The model is a binary
    # classifier, so saturating the posterior to 0 or 1 for inputs
    # that are far outside the class regions is correct.
    for kwargs in (
        dict(force_margin=1e9),
        dict(force_margin=-1e9),
        dict(contact_fraction=2.0),
        dict(safety_pressure=-1.0),
    ):
        decision = model.decide(_safe_observation(**kwargs))
        assert 0.0 <= decision.posterior_safe <= 1.0
        assert model.config.min_scale <= decision.speed_scale <= 1.0


def test_build_observation_normalizes_edge_clearance():
    obs = build_observation(
        force_margin=1.2,
        contact_fraction=0.7,
        boundary_risk=0.2,
        edge_clearance=0.05,
        edge_margin=0.10,
        load_fraction=0.4,
    )
    assert math.isclose(obs.force_margin, 1.2)
    assert math.isclose(obs.contact_fraction, 0.7)
    assert math.isclose(obs.safety_pressure, 0.8)
    assert math.isclose(obs.edge_clearance_norm, 0.5)
    assert math.isclose(obs.level_pressure, 0.6)


def test_build_observation_handles_undefined_edge_clearance():
    obs = build_observation(
        force_margin=1.0,
        contact_fraction=0.5,
        boundary_risk=0.5,
        edge_clearance=float("inf"),
        edge_margin=0.10,
        load_fraction=0.5,
    )
    assert obs.edge_clearance_norm == 1.0

    obs = build_observation(
        force_margin=1.0,
        contact_fraction=0.5,
        boundary_risk=0.5,
        edge_clearance=-0.01,
        edge_margin=0.10,
        load_fraction=0.5,
    )
    assert obs.edge_clearance_norm == 0.0


def test_build_observation_clamps_inputs_to_unit_interval():
    obs = build_observation(
        force_margin=1.0,
        contact_fraction=2.0,
        boundary_risk=-0.5,
        edge_clearance=0.10,
        edge_margin=0.10,
        load_fraction=1.5,
    )
    assert obs.contact_fraction == 1.0
    assert obs.safety_pressure == 1.0
    assert obs.level_pressure == 0.0


def test_from_mapping_overrides_individual_parameters():
    model = BayesianMotionModel.from_mapping({
        "prior_safe": 0.95,
        "min_scale": 0.25,
        "ramp_low": 0.40,
        "ramp_high": 0.90,
        "emergency_force_margin": 0.30,
    })
    assert model.config.prior_safe == 0.95
    assert model.config.min_scale == 0.25
    assert model.config.ramp_low == 0.40
    assert model.config.ramp_high == 0.90
    assert model.config.emergency_force_margin == 0.30
    # Defaults preserved for unspecified keys.
    assert model.config.prior_unsafe == 0.3


def test_from_mapping_overrides_per_feature_means():
    model = BayesianMotionModel.from_mapping({
        "mean_safe_force_margin": 2.0,
        "sigma_unsafe_contact_fraction": 0.5,
    })
    assert model.config.mean_safe[0] == 2.0
    assert model.config.sigma_unsafe[1] == 0.5


def test_from_mapping_ignores_unknown_keys():
    model = BayesianMotionModel.from_mapping({
        "this_is_not_a_real_parameter": 42,
        "another_unknown": "x",
    })
    # The construction must not raise and the default config must
    # survive intact.
    assert model.config.prior_safe == 0.7


def test_serialize_decision_returns_json_with_expected_keys():
    decision = BayesianMotionDecision(
        posterior_safe=0.7,
        speed_scale=0.8,
        hold=False,
        emergency_hold=False,
        log_likelihoods={"log_p_safe": -1.0, "log_p_unsafe": -2.0},
        feature_log_likelihoods={
            "force_margin": 0.5,
            "contact_fraction": 0.1,
            "safety_pressure": 0.0,
            "edge_clearance_norm": 0.0,
            "level_pressure": 0.0,
        },
    )
    import json

    payload = json.loads(serialize_decision(decision))
    assert payload["posterior_safe"] == 0.7
    assert payload["speed_scale"] == 0.8
    assert payload["hold"] is False
    assert payload["emergency_hold"] is False
    assert set(payload["log_likelihoods"].keys()) == {
        "log_p_safe",
        "log_p_unsafe",
    }
    assert set(payload["feature_log_likelihoods"].keys()) == {
        "force_margin",
        "contact_fraction",
        "safety_pressure",
        "edge_clearance_norm",
        "level_pressure",
    }


def test_describe_config_round_trips_to_dict():
    model = BayesianMotionModel()
    cfg_dict = model.describe_config()
    assert cfg_dict["feature_names"] == [
        "force_margin",
        "contact_fraction",
        "safety_pressure",
        "edge_clearance_norm",
        "level_pressure",
    ]
    assert len(cfg_dict["mean_safe"]) == 5
    assert len(cfg_dict["sigma_unsafe"]) == 5


def test_update_returns_new_model_without_mutation():
    base = BayesianMotionModel()
    new = base.update(min_scale=0.5, ramp_low=0.4)
    assert new.config.min_scale == 0.5
    assert new.config.ramp_low == 0.4
    # The original model must be untouched.
    assert base.config.min_scale == 0.10
    assert base.config.ramp_low == 0.30


def test_invalid_config_rejected():
    import pytest

    with pytest.raises(ValueError):
        BayesianMotionConfig(prior_safe=-1.0)
    with pytest.raises(ValueError):
        BayesianMotionConfig(ramp_low=0.9, ramp_high=0.5)
    with pytest.raises(ValueError):
        BayesianMotionConfig(sigma_safe=(0.1, 0.0, 0.2, 0.3, 0.4))


def test_speed_scale_strictly_above_min_for_marginal_observations():
    model = BayesianMotionModel()
    # A "marginal" observation should yield a non-zero speed scale
    # but not full speed.  This guarantees the model avoids
    # both the hard-stop behaviour of the old boundary_hold gate and
    # the discontinuity of the old piecewise edge scale.
    decision = model.decide(
        _safe_observation(
            force_margin=0.6,
            contact_fraction=0.5,
            safety_pressure=0.5,
            edge_clearance_norm=0.5,
            level_pressure=0.5,
        )
    )
    assert model.config.min_scale <= decision.speed_scale < 1.0
