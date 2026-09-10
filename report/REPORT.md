# Pure Pursuit vs. Stanley for DV Autocross

A comparison of the two classical lateral controllers on a cone-delimited
Formula Student Driverless Autocross layout, in Simulink, decided on five KPIs
and the competition's own scoring formula.

---

## 1. What was built, and why this way

The deliverable is one Simulink model, `autox_lateral.slx`, in which **only the
steering law changes**. The track, the speed reference, the longitudinal
controller, the steering actuator, the vehicle model, the control rate and the
solver are identical for both controllers. Any difference in the results is
therefore attributable to the steering law, which is the only claim a controller
comparison can honestly make.

![Simulink model](../figures/fig0_model.png)

| Element | Implementation |
|---|---|
| state | one six-element `Integrator`: `x, y, psi, v, delta, eInt` |
| plant | `plant_block` → `vehicle_ode.m`, the continuous derivative |
| controllers | two MATLAB Function blocks loading `ctrl_pure_pursuit.m` and `ctrl_stanley.m` |
| control rate | 100 Hz, via a zero-order hold on the state vector |
| selection | zero-based `Multiport Switch` on `CTRL_SEL` (0 = pure pursuit, 1 = Stanley) |
| index state | `Unit Delay` carrying the centreline index between control steps |
| solver | fixed-step `ode4` at 1 ms |

Both controllers are evaluated every step and one is selected, so a single run
also records what the other law would have commanded at every instant.

**No toolboxes beyond MATLAB and Simulink are used.** The two library blocks
that would have been the obvious shortcut were rejected deliberately:

- `robotalgslib/Pure Pursuit` (Navigation Toolbox) takes a pose and a waypoint
  array and returns *linear and angular velocity*, not a steering angle. Driving
  a bicycle-model car with it requires an added `delta = atan(omega*L/v)`
  conversion layer on top of a block whose interior cannot be inspected.
- `drivingvehiclecontroller/Lateral Controller Stanley` (Automated Driving
  Toolbox) is not the textbook law: it carries its own position-gain and
  yaw-rate terms and its own input convention. Comparing it against the
  Navigation Toolbox pure pursuit block would compare two blocks with different
  internal assumptions, which quietly destroys the fairness of the comparison.
- Both require the reference path as distance-indexed lookup tables or waypoint
  arrays, and both then need heading-unwrapping pre-processing. On a closed loop
  the heading passes through ±180° several times per lap, and that
  pre-processing - not the control law - is where the defects would come from.

Writing the two laws directly costs ten lines of trigonometry and buys the
ability to prove them correct (§4) and to state them in this report.

---

## 2. The track

`src/track_autox.m` builds the centreline as a **closed polygon with circular
fillets** at its vertices. Closure is then exact because the polygon closes,
straights are exactly straight, each corner radius is an explicit design input,
and heading and curvature are analytic (`kappa = 0` on a straight, `±1/R` on an
arc) rather than finite-differenced.

![Track layout](../figures/fig1_track.png)

The layout follows the guidelines the rules give for the DV Autocross
(**D 6.1.3**, which refers the DV layout to **D 8.1**):

| property | measured | rule |
|---|---|---|
| lap length | 212.6 m | D 8.1.2: approximately 200-500 m |
| minimum turning diameter | 12.00 m | D 8.1.1: >= 9 m |
| longest straight | 53.2 m | D 8.1.1: <= 80 m |
| track width | 3.50 m | D 8.1.1: >= 3 m |
| total turn | 360.0000 deg | closed loop |
| closure error | 2.3·10⁻⁴ m | closed loop |

The closure figure is worth a line: because the centreline is a filleted polygon
rather than a fitted curve, the loop closes to a quarter of a millimetre with no
correction applied. A lap time measured on a track that does not close is not a
lap time.

Four features were placed deliberately, because they are where the two laws
diverge:

| Feature | What it exposes |
|---|---|
| long straight into a tight hairpin | pure pursuit cutting the corner |
| slalom of alternating short-radius corners | Stanley's steering activity |
| decreasing-radius corner | the limits of a fixed lookahead |
| fast open sweep | baseline, both should be clean |

Cones sit on both boundaries at the rule colours - blue on the left of the
direction of travel, yellow on the right, orange for the start/finish gate -
5 m apart on open sections and 3 m apart in corners.

---

## 3. Vehicle, actuator and speed reference

The vehicle is a kinematic bicycle referenced to the rear axle:

```
xdot = v*cos(psi)      psidot = v*tan(delta)/L
ydot = v*sin(psi)      vdot   = a
```

**Pure pursuit is derived at the rear axle and Stanley at the front axle**, so
the front axle position is formed explicitly for Stanley,
`(x + L*cos(psi), y + L*sin(psi))`. Feeding both laws from the same point is the
most common way to produce an unfair comparison, and `tests/run_tests.m`
contains a check that fails if the front-axle reference is dropped.

The steering actuator is a rate-limited first-order lag,

```
ddelta/dt = sat( (sat(deltaCmd, dMax) - delta)/tauSteer , dRateMax )
```

written as a single ODE rather than as a `Rate Limiter` in series with a
transfer function. Two reasons: without an actuator, pure pursuit is handed an
unphysical advantage (infinitely fast steering), and in this form RK4 and `ode4`
integrate the identical equation, which is what makes the agreement in §5
meaningful.

The speed reference is a function of arc length only - grip limit
`v <= sqrt(aLatMax/|kappa|)`, then braking and acceleration passes that **wrap
around the start/finish line** - and is therefore identical for both
controllers. A non-wrapping pass would leave an artificial speed notch at `s = 0`
and a fake braking event in both lap times.

Parameters are engineering assumptions for a representative FS driverless car:
wheelbase 1.55 m, steering ±24° at up to 120°/s with a 0.05 s lag, lateral
acceleration limit 11.8 m/s² (≈1.2 g on slicks), 5 m/s² acceleration, 8 m/s²
braking, 15 m/s speed cap, body 2.90 × 1.20 m for cone contact.

---

## 4. Correctness of the two laws

The controllers are checked against closed-form results, not against another
implementation. `tests/run_tests.m` runs 21 checks; the ones that carry the
argument:

| Check | Why it is conclusive |
|---|---|
| **Ackermann on a circle** | With the rear axle on a circle of radius `R` and zero cross-track error, the goal point at lookahead `Ld` subtends `sin(alpha) = Ld/(2R)`, so `atan(2*L*sin(alpha)/Ld)` must return exactly `atan(L/R)` for **any** `Ld`. Verified at R = 8, 15, 30, 60 m to within 4·10⁻⁵ rad. |
| **zero error ⇒ zero steering** | On the path with zero heading error both laws must return exactly 0. Catches offset and sign slips. |
| **mirror symmetry** | Mirroring the track in `y` must negate every lateral signal and leave the KPIs bit-identical. This is the check that catches a wrong sign in the cross-track error, the left normal, or the Stanley feedback term - a defect that survives any single-direction test. |
| **step response** | Released 1 m off a straight, both must converge below 1 cm without overshooting past the starting offset. |
| **index continuity** | The windowed nearest-point search must not jump across the loop at a hairpin. |

The Ackermann check earned its place immediately: it failed at R = 8 m on the
first run and exposed a real defect. The goal point was being marched `Ld` along
the **arc length**, whereas pure pursuit's `Ld` is the straight-line **chord**
from the vehicle. Since arc > chord on any curve, the goal point sat too close
and the commanded angle was biased low in tight corners - by 0.29° at R = 8 m,
growing as the radius shrank. Finding the goal point on the chord fixed it and
made the check exact.

---

## 5. Does the Simulink model do what the code says?

`verify_equivalence.m` runs `run_reference.m` - the same controller and plant
files integrated with RK4 in plain MATLAB - and the Simulink model at the same
fixed step, then measures the difference over a full lap:

| Deviation over one lap | Pure Pursuit | Stanley | tolerance |
|---|---|---|---|
| position | 1·10⁻¹³ m | 4.4·10⁻¹¹ m | 10⁻³ m |
| heading | 1·10⁻¹³ rad | 3.3·10⁻¹² rad | 10⁻⁴ rad |
| speed | 1·10⁻¹³ m/s | 7.4·10⁻¹² m/s | 10⁻³ m/s |
| steering | 5.0·10⁻¹³ rad | 2.8·10⁻¹² rad | 10⁻⁴ rad |

Agreement at machine precision, eleven orders of magnitude inside the
tolerances, so the tolerances are not doing any work. The MATLAB reference loop
also runs unmodified in GNU Octave, which is how the numbers in this report were
produced before MATLAB was available.

---

## 6. KPIs and the decision rule

<!-- RESULTS -->

---

## 7. Limitations

Stated plainly, because they bound what the verdict above can claim:

1. **Kinematic vehicle model.** No tyre slip, so understeer at the grip limit
   does not appear. The speed profile caps lateral acceleration at
   `aLatMax` instead of letting the tyres saturate. A 2-DOF dynamic bicycle
   model plugs into the same `vehicle_ode` interface and is the obvious next
   step; the ranking here should be re-checked against it.
2. **Perfect state estimate apart from the injected noise and latency.** No
   SLAM drift, no cone-detection error, no map that has to be built during the
   first lap. In a real DV Autocross run the first lap is driven on perception
   alone.
3. **One layout.** The features were chosen to separate the two laws, but a
   different Autocross track - tighter, or faster - may shift the balance. The
   track generator is parametric, so this is cheap to re-run.
4. **Gains tuned on this track.** The sweep optimises for this layout and these
   three conditions. It says which law is easier to tune, not what the gains
   should be at a different event.
5. **Longitudinal control is shared and simple.** A PI on a fixed speed
   profile, not a combined-slip optimal controller. This is deliberate - it
   isolates the lateral comparison - but it means neither controller is being
   pushed to the true limit of the car.

---

## 8. Reproducing this

```matlab
run_tests            % 21 analytical checks
run_all              % tune, run, sweep, plot, tabulate
build_model('open',true)
verify_equivalence   % model vs. reference loop
```

Every number and figure in this report is produced by `run_all.m`; the tables in
§2 and §6 are generated into `report/kpi_tables.md` by the same run, so the
prose cannot drift from the code.
