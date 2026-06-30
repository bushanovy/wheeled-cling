# Bayesian smooth motion model

The Bayesian smooth motion model is a small, **training-free**
probabilistic speed-scaler that replaces the hard
`boundary_hold` and piecewise-linear `edge_motion_scale` mechanisms
in `surface_goal_planner.py`.  It computes a posterior probability
that the current observation is "safe" and uses a smoothstep to
map that posterior to a continuous speed scaler in
`[min_scale, 1.0]`.  Two emergency gates (per-feature
`emergency_hold` and a posterior `hold`) are still exposed for
catastrophic single-feature failures, but the everyday speed
modulation is now a single C^0-continuous curve.

The model is implemented in
[`scripts/bayesian_motion_model.py`](../scripts/bayesian_motion_model.py)
and tested in
[`test/test_bayesian_motion_model.py`](../test/test_bayesian_motion_model.py).

## Why a Bayesian model?

The previous planner reduced its commanded speed using two
independent piecewise-linear mechanisms:

1. `edge_motion_scale` - a hard linear ramp on the minimum edge
   clearance, which collapses to zero speed as soon as the robot
   is "inside" the surface polygon.
2. `boundary_hold` - a boolean gate that cuts the speed to zero when
   the adhesion `boundary_risk` exceeds `boundary_hold_risk`.

Both are *piecewise* and have a single hard cliff, so the robot
snaps between full speed and zero speed at the boundary.  On the
horizontal pipe test, this produced jerky motion as the robot
curved over the top of the pipe, where the tangential gravity load
rises quickly and the magnetic contact fraction drops as the
wheels ride over the crown.

A **Gaussian naive Bayes** classifier on a binary latent state
$S \in \{\mathrm{safe}, \mathrm{unsafe}\}$ gives a smooth,
monotonic, multi-feature posterior.  The smoothstep on the
posterior is C^1-continuous and bounded, so the speed scaler can
never jump abruptly, and the per-feature log-likelihoods can be
inspected at runtime for diagnosis.

## Model

We use a **Gaussian naive Bayes** classifier over a binary latent
state $S \in \{\mathrm{safe}, \mathrm{unsafe}\}$.

The observation vector is

$$
\mathbf{x} = (x_1, x_2, x_3, x_4, x_5)
$$

where every component is normalized so that **larger is safer**:

| Index | Feature           | Source in planner                          | Range         |
|------:|-------------------|--------------------------------------------|---------------|
| 1     | `force_margin`    | `safety['current']['margin']` (capacity / safety-factored required load) | $[0, \infty)$ (typical 0.5 - 5.0) |
| 2     | `contact_fraction`| `avg_contact_fraction`                    | $[0, 1]$      |
| 3     | `safety_pressure` | $1 -$ `boundary_risk`                      | $[0, 1]$      |
| 4     | `edge_clearance_norm` | `edge_clearance_m` / `edge_margin_m`    | $[0, 1]$      |
| 5     | `level_pressure`  | $1 -$ `load_fraction` ($= \sqrt{1-n_z^2}$)| $[0, 1]$      |

For each class $c \in \{\mathrm{safe}, \mathrm{unsafe}\}$ the
likelihood of an observation is a product of one-dimensional
Gaussians

$$
P(\mathbf{x} \mid c) = \prod_{i=1}^{5}
    \frac{1}{\sqrt{2\pi}\,\sigma_{i,c}}
    \exp\!\left(-\frac{(x_i - \mu_{i,c})^2}{2\sigma_{i,c}^2}\right).
$$

The class means encode the *typical* values of each feature under
the two classes (defaults: $\mu_{\mathrm{safe}} = (1.5, 0.9, 0.95,
0.9, 0.95)$ and $\mu_{\mathrm{unsafe}} = (0.2, 0.1, 0.2, 0.1,
0.1)$); the sigmas encode how much variation in each feature is
still consistent with each class.  We work in the log domain for
numerical stability:

$$
\log P(c \mid \mathbf{x}) = \log P(c) + \sum_{i=1}^{5}
    \left[ -\frac{(x_i - \mu_{i,c})^2}{2\sigma_{i,c}^2}
              -\log(\sqrt{2\pi}\,\sigma_{i,c}) \right]
$$

and the final posterior is the softmax over the two classes.

The model is intentionally **configurable**: every mean, sigma,
the prior, and the smoothstep ramp can be set from ROS parameters
without recompiling.  See the table below.

## Smoothing

The posterior is mapped to a speed scale with a **smoothstep**:

$$
s = s_{\min} + (1 - s_{\min}) \cdot
    \mathrm{smoothstep}(P, P_{\mathrm{low}}, P_{\mathrm{high}})
$$

where $\mathrm{smoothstep}(x, a, b) = 0$ for $x \le a$, $1$ for
$x \ge b$, and $3t^2 - 2t^3$ with $t = (x-a)/(b-a)$ in between.
This gives a C^1-continuous ramp with no discontinuities.

Two emergency gates are exposed alongside the smooth scale:

* `hold` - True if the posterior is below `hold_threshold`
  (defaults to 0.5).  Callers that prefer a hard "stop completely"
  gate can use this in place of a multiplicative scale.
* `emergency_hold` - True only if the robot is *clearly* unsafe on
  any single feature (e.g. `force_margin < 0.5` or
  `contact_fraction < 0.05`).  This is a fast safety net
  independent of the smoother posterior, mirroring the role of
  the original `boundary_hold` for catastrophic cases.

## Default parameters

| ROS parameter                                | Default | Meaning                                       |
|----------------------------------------------|--------:|-----------------------------------------------|
| `bayesian_speed_enabled`                     | `True`  | Apply the speed scaler in the planner.        |
| `bayesian_replace_boundary_hold`             | `True`  | Use `emergency_hold` in place of the hard `boundary_hold`. |
| `bayesian_prior_safe` / `_unsafe`            | `0.7 / 0.3` | Class priors (need not sum to 1).         |
| `bayesian_mean_safe_<feature>`               | see docstring | $\mu_{\mathrm{safe}}$ per feature.       |
| `bayesian_mean_unsafe_<feature>`             | see docstring | $\mu_{\mathrm{unsafe}}$ per feature.     |
| `bayesian_sigma_safe_<feature>`              | see docstring | $\sigma_{\mathrm{safe}}$ per feature.     |
| `bayesian_sigma_unsafe_<feature>`            | see docstring | $\sigma_{\mathrm{unsafe}}$ per feature.   |
| `bayesian_ramp_low` / `_high`                | `0.30 / 0.85` | Smoothstep edges on the posterior.     |
| `bayesian_min_scale`                         | `0.10`  | Speed scale at the bottom of the smoothstep.  |
| `bayesian_hold_threshold`                    | `0.50`  | Posterior below this sets `hold=True`.        |
| `bayesian_emergency_force_margin`            | `0.50`  | Below this force margin, `emergency_hold` fires. |
| `bayesian_emergency_contact_fraction`        | `0.05`  | Below this contact fraction, `emergency_hold` fires. |
| `bayesian_posterior_tau_s`                   | `0.20`  | Low-pass time constant for the posterior.    |

## Runtime introspection

The planner publishes the model's output in its JSON debug
message on `/surface_goal_planner/debug`:

```json
{
  "bayesian_speed_enabled": true,
  "bayesian_replace_boundary_hold": true,
  "bayesian_posterior": 0.93,
  "bayesian_speed_scale": 0.95,
  "bayesian_hold": false,
  "bayesian_emergency_hold": false,
  "bayesian_feature_log_likelihoods": {
    "force_margin": 1.0,
    "contact_fraction": 0.5,
    "safety_pressure": 0.0,
    "edge_clearance_norm": 0.0,
    "level_pressure": 0.0
  },
  "bayesian_log_likelihoods": {
    "log_p_safe": -0.42,
    "log_p_unsafe": -3.1
  },
  "bayesian_posterior_tau_s": 0.2
}
```

A `Trigger` service is exposed at
`/surface_goal_planner/reload_bayesian_model`.  Call it after a
batch of `ros2 param set` calls to apply the new parameters
without restarting the node:

```bash
ros2 param set /surface_goal_planner bayesian_ramp_low 0.20
ros2 param set /surface_goal_planner bayesian_min_scale 0.05
ros2 service call /surface_goal_planner/reload_bayesian_model \
    std_srvs/srv/Trigger
```

## Algorithm: end-to-end

1. The classical safety gates (`_traction_safety`) accept the
   current pose.
2. The model is fed a five-dimensional observation built from
   `force_margin`, `contact_fraction`, `boundary_risk`,
   `edge_clearance`, and `load_fraction` (see
   `bayesian_motion_model.build_observation`).
3. The model returns `posterior_safe`, `speed_scale`, `hold`, and
   `emergency_hold`.
4. The posterior is low-pass filtered with a time constant
   `bayesian_posterior_tau_s` for additional smoothness.
5. The final commanded velocity is multiplied by
   `edge_scale * bayesian_speed_scale` (both in `[0, 1]`).
6. If `emergency_hold` is true and
   `bayesian_replace_boundary_hold` is true, the planner publishes
   a zero-velocity body command with mode
   `bayesian emergency hold`.

## Why this is *training-free*

The model has no learned parameters.  All means, sigmas, and the
prior are physically motivated defaults:

* `force_margin` is `safety['current']['margin']`, which the
  planner already computes.
* `contact_fraction` is the rolling average from the adhesion
  status topic.
* `safety_pressure` is $1 -$ `boundary_risk` directly.
* `edge_clearance_norm` is `edge_clearance_m / edge_margin_m`.
* `level_pressure` is $1 -$ `load_fraction`, which the planner
  computes from the surface normal.

The class means are chosen so that a "safe" observation has high
values for all five features, and an "unsafe" observation has
low values for all five.  The Gaussians are tight enough that the
posterior transitions sharply across the boundary region, but
the smoothstep on the posterior guarantees that the commanded
speed remains C^0-continuous in the feature space.

## Test coverage

The `test/test_bayesian_motion_model.py` suite has 21 tests
covering:

* Posterior and speed scale bounds.
* Monotonicity in every input feature.
* The two emergency gates fire on the right kinds of features.
* The smoothstep helper is monotonic and C^0-continuous.
* The `build_observation` adapter normalizes edge clearance and
  clamps inputs.
* The `from_mapping` constructor is robust to missing keys.
* Invalid configurations are rejected at construction time.

Run them with:

```bash
cd src/robot_3d3s
python3 -m pytest test/test_bayesian_motion_model.py
```

## Limitations

* The model assumes feature independence (naive Bayes).  The
  features are loosely coupled in practice (a low contact
  fraction often coincides with a low force margin), so the
  posterior may be slightly more extreme than reality.
* The default class means and sigmas were chosen by hand.  They
  can be tuned via the `bayesian_*` ROS parameters, but a proper
  Gaussian-Mixture fit on logged data would be a useful follow-up.
* The model is purely a *speed scaler* - it does not change the
  geometry of the trajectory.  Combining it with a future
  trajectory-optimization layer would yield even smoother motion.
