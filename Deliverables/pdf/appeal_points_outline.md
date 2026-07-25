# Appeal-Points Outline (draft material, not final content)

Structured list of technical facts/decisions from the implementation, organized by topic.
This is source material to write from — not final appeal-points prose. Each bullet is a
fact or design decision, not an argument; the framing/argument is for the final PDF.

## 1. Equations of motion / physics approach

- Robot moves exclusively via `Rigidbody.AddForce` / `AddTorque`; no `Transform` position
  or rotation writes anywhere in the control path.
- Differential-drive kinematics: `v = (vL + vR) / 2`, `omega = (vL - vR) / TrackWidth`.
- Chassis-level force/torque split (translation and rotation driven as two independent
  channels through the CoM) rather than applying force at the literal wheel contact
  points. Tried the per-wheel-contact-point model first; with CoM 0.5 m above a
  ~0.04 m wheel plane, off-axis force there creates a large tipping moment and
  destabilizes the chassis (observed 167° tip in an isolated forward-drive test).
- `AddForce` with no position argument acts through `centerOfMass`; `AddTorque` is a
  pure couple (no net force) — this is why the split is safe from tipping in a way the
  per-wheel model wasn't.
- Continuous (not ContinuousDynamic) collision detection: only tunneling concern is the
  single dynamic robot against static course geometry, which plain Continuous covers.
  ContinuousDynamic combined with a convex MeshCollider silently suppressed
  `OnCollisionEnter`/`Stay` dispatch while still resolving contact physically.

## 2. Mass properties (CoM, inertia tensor)

- Mass 10 kg, CoM overridden to 0.50 m above the floor (per spec), inertia tensor
  modeled as a uniform solid cylinder spanning floor to camera-top (radius = body
  radius, height = pole height) — consistent with the given CoM height, since a
  uniform cylinder's centroid sits at half its height.
  - Yaw inertia: `0.5 * m * r^2`
  - Tilt inertia: `(1/12) * m * (3*r^2 + h^2)`
- `Rigidbody.automaticCenterOfMass` / `automaticInertiaTensor` default to `true` and
  silently recompute both from collider geometry on every physics re-evaluation,
  discarding manual overrides unless explicitly disabled first — a real bug hit and
  fixed early (manual CoM/inertia "stuck" only for an instant after assignment).

## 3. Trajectory generation

- Waypoint/pivot sequence fixed by the confirmed course geometry (not re-derived).
- Minimum-jerk (quintic) velocity profile per straight segment (Flash & Hogan model):
  `position(tau) = 6*tau^5 - 15*tau^4 + 10*tau^3`, zero velocity and acceleration at
  both endpoints — chosen so the profile itself is smooth (bounded jerk) rather than
  relying on the controller to smooth a trapezoidal/step profile after the fact.
- Turn segments: angular velocity profile sized so the camera-top point's own
  acceleration (not the chassis's) stays at the 1.00 m/s² cap — solved from the
  kinematic relationship between chassis motion and camera-top point acceleration,
  not assumed or guessed.
- `StraightSegment` takes an explicit fixed heading + computed length (via projection)
  rather than deriving heading from `(start, targetPoint)` geometry — fixes a heading
  discontinuity bug where pivot turns land ~0.17 m off the nominal waypoint, so
  geometry-derived heading for the next straight didn't match the turn's actual ending
  heading. Matches physical reality that heading can't snap instantaneously.

## 4. Closed-loop control (PID / state-feedback) vs. open-loop

- Replaced open-loop waypoint following with a Kanayama-style unicycle trajectory-
  tracking control law (Lyapunov-stable state feedback), not a naive independent-axis
  PID:
  - Robot-frame error state: `e1` (longitudinal), `e2` (lateral), `e3` (heading).
  - `v = v_r*cos(e3) + K1*e1`
  - `omega = omega_r + K2*v_r*sinc(e3)*e2 + K3*e3`
- Why state feedback over plain PID-on-position: the reference trajectory already
  encodes a feasible velocity/heading profile (`v_r`, `omega_r`); tracking control
  corrects the error around that moving reference rather than driving position error
  to zero from scratch, which is both more accurate mid-trajectory and avoids fighting
  the reference profile's own dynamics.
- Below the trajectory tracker, a second closed loop converts commanded chassis
  `v`/`omega` into actual force/torque: proportional velocity-error control on linear
  velocity (chassis-frame, gain-scaled, force-clamped) and angular-velocity error on
  yaw (gain-scaled, torque-clamped) — i.e., two nested loops, not one.

## 5. Discrete-time control-loop stability

- Explicit-Euler integration of a first-order lag: `x[n+1] = x[n]*(1-k) + target*k`
  where `k = gain * fixedDeltaTime / inertia`. `k > 2` makes the loop numerically
  oscillate/overshoot every step regardless of how "correct" the gain looks in
  continuous time — a discrete-time constraint independent of and in addition to
  continuous-time tuning.
- This is a concrete, checkable design constraint (not just "tuned until it looked
  right"): e.g. with yaw inertia ≈ 0.1125 kg·m² and `fixedDeltaTime = 0.02 s`, angular
  gain must stay under `0.1125*2/0.02 = 11.25` for numerical stability.
- Root-cause example this constraint explained: a pivot-turn drift bug that persisted
  even after fixing the CoM/inertia override bug — turned out to be `k = 2.67 > 2`
  after the inertia fix made the loop "too fast" for the fixed timestep, causing
  oscillation. Fixed by lowering angular gain, not by further inertia changes.

## 6. Acceleration-cap handling (1.00 m/s² at camera-top) and tip-over physics

- Cap applies to the camera-top point specifically, not the chassis/CoM — camera-top
  acceleration = chassis linear acceleration + rotational (tangential + centripetal)
  contribution from the pole's offset above the CoM.
- Zero-Moment-Point (ZMP) tip-over constraint: `a * CoMHeight / g > footprint_radius`
  causes tip-over regardless of "correct" force direction. With CoM height 0.50 m and
  footprint radius ≈0.15 m, tip threshold ≈ `0.15*9.81/0.5 ≈ 2.94 m/s²` — a real
  physical constraint from this robot's unusually high CoM vs. small footprint, not an
  arbitrary safety margin.
- Controller-side acceleration/angular-acceleration clamps exist for two independent
  reasons: (a) backstop against runaway torque/force on abnormal transient error, and
  (b) staying under the ZMP tip threshold. They are deliberately NOT sized directly
  against the 1.00 m/s² camera-top cap.
- Key lesson (found via regression, not assumed up front): clamping controller
  authority at/below the reference trajectory's own peak requirement is self-defeating.
  The M6 planner already sizes turn duration so the reference profile's own peak
  angular acceleration (~5–6 rad/s² for a 90° pivot) keeps camera-top acceleration at
  exactly the cap. A controller clamped below that (tried 0.6 m/s² / 3.0 rad/s²) can't
  even follow its own safe reference, let alone correct tracking error on top of it —
  produced 30°+ heading error and a wall collision. Fix was headroom *above* the
  reference's own peak (1.5 m/s² / 15 rad/s²), wide enough that ordinary tracking never
  saturates it while still catching genuinely pathological errors.

## 7. Simulation-fidelity debugging (physics-engine-level findings)

- `OnCollisionEnter`/`Stay` dispatch to the GameObject owning the **Rigidbody**, not
  the child GameObject owning the **Collider** — opposite of a common assumption;
  invalidation logic had to live on the root component, not a child collision
  forwarder.
- Convex `MeshCollider` resting-contact jitter: a low-poly (~20-side) cylinder's flat
  bottom face is really a ring of large planar facets; PhysX's contact manifold for a
  resting convex hull shifts between adjacent facet vertices frame to frame, injecting
  small but real torque impulses every step. Diagnosed by comparing chassis
  acceleration (stayed smooth, <0.6 m/s²) against camera-top acceleration (spiked to
  6.2 m/s²) during the same window — the discrepancy located the fault to the
  rotational/contact side, not the drive force itself.
- A bounding `BoxCollider` was tried as a fix and eliminated the jitter, but a box
  circumscribing a 0.15 m-radius circular footprint has corners at
  `0.15*sqrt(2) ≈ 0.212 m` — 41% further out than the true footprint at diagonal
  headings — which clipped a wall during a turn the true circular footprint clears.
  Reverted in favor of a purpose-built, higher-resolution (32-sided) convex cylinder
  collider mesh, generated procedurally and used only for physics (visual mesh
  unchanged): every vertex sits at the true radius, worst-case edge-midpoint shortfall
  ≈0.48% (vs. the box's 41% overshoot).
- Even with the finer collider, residual jitter remained (spike reduced from 6.2 to
  3.85 m/s²) — resolved by raising `Rigidbody` solver iteration counts (16 position /
  8 velocity, up from Unity's defaults of 6/1). This is a pure numerical-convergence
  increase on the same physical model, not an added damping term that would resist
  real motion.
- Default `PhysicsMaterial` friction was found to double-count resistance: the
  drive controller's chassis force already represents the abstracted net wheel-
  traction force (brief says to ignore rolling resistance), so nonzero collider
  friction on top of that treated the chassis as a block additionally resisting its
  own drive force — strong enough to fully cancel a torque-limited drive force at one
  point. See §7b for the full modelling argument and measurements.

## 7b. Wheel/floor friction — modelling rationale (IMPORTANT for the writeup)

This is the one place where the implementation deliberately diverges from a naive
reading of the requirement, so it deserves an explicit paragraph rather than a
footnote. The argument has two independent legs; both are worth stating.

**Leg 1 — what the requirement actually grants.** The wording is
「車輪と床面の間には駆動に必要な摩擦が存在するものとし、転がり抵抗、軸受損失、空気抵抗
などの軽微な損失は無視してよい」. Read carefully, this is a *modelling assumption granted
to the student*, not a mandate to tune a Coulomb coefficient:
  - 「〜が存在するものとし」 = "assume that ... exists" — the grammatical form of a
    given premise, not a specification to satisfy.
  - What it grants is specifically 駆動に必要な摩擦 — the friction *needed for driving*,
    i.e. sufficient grip to transmit traction without wheel slip.
  - The same sentence then explicitly permits ignoring 転がり抵抗 (rolling resistance)
    and other minor losses.
  - A net-traction-at-chassis model is exactly that premise made concrete: it assumes
    perfect grip (the commanded traction is always delivered, wheels never slip) and
    omits the dissipative terms the clause says to omit. The friction is *represented
    by the drive force itself*, not by a separate sliding-contact term.

**Leg 2 — why adding sliding friction is physically wrong here, with measurements.**
A rolling wheel's contact patch has zero sliding velocity, so real traction friction
transmits force without dissipating energy. Unity has no rolling constraint on a plain
collider, so a nonzero coefficient models a *skidding* body — a sled, not a wheeled
robot — and charges losses the assignment says to ignore. Measured consequence
(full course, identical trajectory, only the coefficient varied):

| μ | peak camera-top accel | peak jerk |
|------|------------------------|-----------|
| 0.00 | 0.729 m/s² ✔ | 2.28 m/s³ |
| 0.02 | 1.194 m/s² ✗ over cap | 98.5 m/s³ |
| 0.05 | 2.201 m/s² ✗ over cap | 148.1 m/s³ |

The mechanism is stick-slip: every start-from-rest, stop-at-corner and spin-onset
releases static friction abruptly, and that step is amplified by the camera-top
point's 0.5 m lever arm above the CoM. Attribution was confirmed by re-running with
μ = 0 while keeping every other change — jerk fell from 98.5 to 2.28.

The one remaining hope was that the continuous-motion trajectory (arc corners, which
removed the four corner stops — see §6) would reduce stick-slip enough to carry a
nonzero coefficient. It was re-tested once on that faster trajectory: μ = 0.05 still
peaked at **2.11 m/s²** (cap 1.00) with jerk 134.8. Even with only two stick-slip
events left — the flying start and the final stop — a single abrupt static-friction
release at the camera-top lever arm is enough to blow the cap. Friction and the
1.00 m/s² requirement are fundamentally incompatible under this contact model,
independent of how few stops the route has.

So modelling sliding friction does not make the simulation *more* faithful; it makes
it less faithful (a skidding sled) **and** breaks the 1.00 m/s² 必須条件 while
destroying the 加速度・ジャークの小ささ score. Choosing μ = 0 is the choice that
honours both the letter of the friction clause and the acceleration requirement.

**Honest caveat worth including** (shows judgement rather than hiding the weak point):
the tradeoff is that a grader inspecting the `PhysicsMaterial` sees a zero
coefficient. The contact pair is therefore declared explicitly on *both* surfaces
(robot body and floor) with the coefficient and this reasoning documented in code, so
the choice is visible and deliberate rather than an oversight. Note also that leaving
the floor on Unity's default (μ=0.6) while only setting the robot's material silently
produced μ=0.36 via Average combine — a real latent bug this work surfaced.

## 8. Invalidation and timing

- Invalidation conditions checked independently: wall contact (via root-level
  `OnCollisionEnter`, tag check on `Wall`-tagged geometry), off-course bounds
  (rectangular X/Z backstop outside the known course extents), tip-over (chassis tilt
  angle vs. up-vector exceeding a threshold, 30°).
- On invalidation, driving is explicitly zeroed (`SetWheelSpeeds(0, 0)`) so the robot
  doesn't continue accumulating motion after a run is already invalid.
- Timing: clock starts on `StartLine` trigger touch, stops on full `GoalLine`
  clearance (not just first contact) — implemented via trigger-enter/exit events on
  a `LineTrigger` component that checks for the robot via
  `GetComponentInParent<RobotRig>()`.

## 9. Telemetry / metrics logging

- Per-`FixedUpdate` capture: time, position (X/Z), heading, chassis speed/acceleration,
  camera-top speed/acceleration/jerk, angular speed — one row per physics step, not
  sampled at a lower rate, so the acceleration-cap check operates on the same
  resolution the cap is defined at.
- Camera-top kinematics computed via `Rigidbody.GetPointVelocity()` (folds chassis
  linear motion and rotational contribution together for an arbitrary world point)
  finite-differenced across steps for acceleration and jerk, rather than hand-deriving
  `a = a_CoM + alpha × r + omega × (omega × r)` term by term.
- CSV export uses culture-invariant number formatting so the file parses correctly
  regardless of the machine's locale settings.

## 10. Overall milestone structure (for describing the build process itself)

- Built bottom-up in gated stages, each with an automated PlayMode regression test
  that had to pass before proceeding to the next: bare Rigidbody movement proof →
  real robot geometry/mass properties → differential drive kinematics → course
  geometry → camera-top kinematics module → trajectory generation → closed-loop
  tracking control → invalidation/timing → telemetry → full-course regression.
- Headless verification (`-batchmode -nographics -runTests -testPlatform PlayMode`)
  used throughout in place of manual/visual play-mode checks — every stage is a
  standing regression test, not a one-time manual verification, and the same suite
  re-validates earlier stages whenever a later-stage fix touches shared code (e.g. the
  M10 collider/solver fixes were re-verified against the full 35-test suite, not just
  the one full-course test).

## 11. Compliance/optimization pass (the material with the most "appeal")

This is the part of the work most worth writing up: an independent audit against the
assignment PDF (not our internal brief) drove a series of measured, principled changes.

### 11a. Continuous-motion arc corners — the big time win (④ trajectory, ③ min-time)

- The original route was stop / turn-in-place / go: every corner decelerated to zero,
  spun, then re-accelerated. Segment analysis of the telemetry showed **~39% of the
  timed run was spent at a crawl or standstill** — almost entirely corner stops.
- Replaced with a **continuous-motion trajectory**: clothoid (Euler-spiral) corners
  joined by speed-profiled straights, speed never returning to zero between start and
  goal. Result: **走破時間 16.96 s → 11.08 s (−35%)**, and — critically — **max jerk also
  fell, 6.25 → 4.28 m/s³**. Both scored axes improved at once; no trade.
- Why clothoid, not a plain circular arc: a constant-radius arc steps curvature from 0
  to 1/R in one physics step, so the centripetal term v²/R (and thus lateral
  acceleration) appears instantaneously → a ~7× jerk spike at every corner entry/exit.
  The clothoid ramps curvature smoothly (raised-cosine in arc length), so lateral
  acceleration and jerk both start and end at exactly zero and join the straights with
  no discontinuity. This is the same idea as a railway/highway transition spiral.
- Corner sizing is geometry-first and counter-intuitive: the 0.60 m corridor with a
  0.30 m robot leaves only 0.15 m of clearance, and a measured sweep (with corners
  correctly placed on the centrelines) showed the body gap is fixed at 0.15 m by the
  centreline-to-wall distance for *any* curvature ≥ ~5 — the tight corner never cuts
  closer to the inner block than the straights do. So a gentler corner is strictly
  better on both scored axes (higher corner speed √(maxAccel/κ), lower clothoid jerk
  ∝ √κ), bounded only by the setback fitting the shortest straight.

### 11b. Jerk-limited minimum-time, and why we did NOT take the fastest option (③)

- The straights are bang-bang in acceleration (min-jerk S-curve to a length-limited
  peak, hitting the acceleration budget on both the accelerate and decelerate halves),
  and the corners sit exactly at the lateral-acceleration bound. Both constraints are
  therefore *active* — the hallmark of a time-optimal solution — making this a genuine
  **jerk-limited minimum-time trajectory** (the S-curve / practical form of minimum-time
  control, not naive bang-bang).
- The one remaining freedom, corner curvature κ, was pushed to the clearance boundary
  and *measured*: the pure-minimum-time choice (κ = 5) cuts course time 11.10 → 10.58 s
  (−5%) but pushes measured jerk 4.25 → 5.24 m/s³ (+23%). Because 走破時間 and
  ジャークの小ささ are co-equal criteria, that is a net loss, so κ = 6 (the knee of the
  time-vs-jerk curve) was chosen deliberately. Worth presenting as evidence of
  optimizing the *right* objective rather than just the clock.

### 11c. Genuine PID inner loop (② PID), distinct from the Kanayama law (⑤)

- Two nested closed loops, and they are genuinely different controllers — worth keeping
  distinct in the writeup so ② and ⑤ are both clearly earned:
  - Outer: the Kanayama unicycle trajectory-tracking law (Lyapunov state feedback) —
    decides *what* chassis (v, ω) to command from the robot-frame pose error. (⑤)
  - Inner: a full **PID** on each chassis channel (linear velocity, yaw rate) that turns
    those commands into force/torque. It has a real integral term (removes the steady
    lag pure P leaves while chasing an accelerating reference), a real derivative term
    taken on the *measurement* (−d(v)/dt, so a setpoint step gives no derivative-kick
    jerk spike), and **anti-windup** by conditional integration + integral clamp. (②)
- Integral gains held well overdamped (ζ ≈ 4.7 linear, 3.2 yaw) specifically so adding I
  cannot make the loop ring — ringing would show up directly as jerk.
- Two regression tests prove the terms are real: one shows the integral rejects a
  constant disturbance that pure-P leaves a steady error under; the other shows
  anti-windup keeps the integral bounded under sustained saturation.

### 11d. Explicit camera-top acceleration EOM, cross-validated (① EOM modelling)

- The camera-top acceleration is computed the primary way by finite-differencing
  `Rigidbody.GetPointVelocity()`, and *also* the explicit rigid-body way, term by term:
  **a_point = a_CoM + α × r + ω × (ω × r)** (base + tangential + centripetal). A
  regression test asserts the two agree during a pivot — the built-in method is thereby
  cross-validated against the hand-derived EOM.
- The explicit form makes an otherwise-hidden result visible and is the crux of the
  acceleration-cap strategy: r points from the CoM straight up the pole to the camera on
  the **yaw axis**, so for a pure yaw (ω and r both vertical) *both* rotational terms
  vanish (ω × r = 0). That is exactly why a spin-in-place adds essentially no camera-top
  acceleration, and why camera-top ≈ chassis acceleration except for the small term from
  any body tilt. Mounting the camera on the yaw axis is what makes the whole
  acceleration budget spendable on translation.

### 11e. Deliverable correctness (from the audit)

- 走破時間 in the metrics PDF was originally the whole telemetry span (spawn runway +
  post-goal tail), ~4 s too long on a 20-point criterion; now the true StartLine-touch →
  GoalLine-clearance interval from RaceManager. Fixed Timestep (0.02 s) added. Metrics
  relabelled in the assignment's own Japanese wording, and every acceleration metric
  reported for **both** the camera-top point and the chassis (合成加速度 is ambiguous).
- Finish condition tightened to require passing *through* the goal line (opposite side),
  not merely leaving its trigger — so reversing back out cannot stop the clock.
