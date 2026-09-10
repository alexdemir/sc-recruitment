# Pure Pursuit vs. Stanley on a DV Autocross track

A self-contained Simulink and MATLAB study comparing the two classical lateral
controllers on a cone-delimited Formula Student Driverless Autocross layout,
scored with the competition's own metric.

**Requires MATLAB + Simulink only.** No additional toolboxes. Every algorithm in
here - track generation, both steering laws, the speed profile, the vehicle
model, the KPIs - is in this repository, so the control laws can be read,
checked and defended rather than taken on trust from a library block. The plain
MATLAB parts also run unmodified in GNU Octave.

## Result

**Stanley**, on the width the rules actually guarantee.

| | Pure Pursuit | Stanley |
|---|---|---|
| lap time | **17.96 s** | 18.18 s |
| max cross-track error | 0.902 m | **0.129 m** |
| RMS cross-track error | 0.408 m | **0.055 m** |
| RMS steering rate | **8.73 °/s** | 21.12 °/s |
| cones down or out (3.5 m track) | 0 | 0 |
| DV Autocross points, 3.5 m track | **100.00** | 98.87 |
| DV Autocross points, **3.0 m track** (D 8.1.1 minimum) | 80.29 | **100.00** |
| the same, at +30 % speed | 50.53 | **100.00** |

Pure pursuit is 0.22 s a lap faster and leads by 1.13 points - but only on a
track wider than the rules promise. At the 3 m minimum its 0.25 m of clearance
to the cone line goes negative and it starts collecting 2 s penalties, while
Stanley never drops below 0.77 m of clearance at any width or speed tested.
Full argument, and the case for pure pursuit where the steering actuator or an
uncharacterised loop delay is the binding constraint, in
[report/REPORT.md](report/REPORT.md).

## Quick start

```matlab
run_tests            % 21 analytical checks on the track and the two laws
run_all              % tune, run, sweep, plot, tabulate  -> figures/, report/
build_model('open',true)   % generate and open the Simulink model
verify_equivalence   % Simulink model vs. the MATLAB reference loop
```

## What is being compared

| | Pure Pursuit | Stanley |
|---|---|---|
| reference point | rear axle | front axle |
| law | `delta = atan(2*L*sin(alpha)/Ld)` | `delta = e_psi + atan(-ke*ey/(ksoft+v))` |
| gains | `Ld0`, `kv` (speed-scheduled lookahead) | `ke`, `ksoft` |

Everything else is deliberately identical: the same track, the same
curvature-limited speed profile, the same longitudinal PI, the same steering
actuator (saturation, rate limit, first-order lag) and the same 100 Hz control
rate. Any difference in the results is therefore attributable to the steering
law, which is the whole point of the exercise.

Both controllers are tuned by grid search against the **same** objective, so
neither can be accused of having been handed an advantage.

## KPIs

1. lap time [s]
2. maximum cross-track error [m]
3. RMS cross-track error [m]
4. RMS steering rate [deg/s] - actuator load and ride smoothness
5. cones Down or Out [-]

turned into the metric the rules actually award:

- **effective time** = lap time + 2 s per cone + 10 s per off-course
  (FS Rules 2026, D 10.1.7)
- **FSG DV Autocross points** = `0.9*Pmax*(Tmax-T)/(Tmax-Tmin) + 0.1*Pmax`
  with `Tmax` the lap driven at 6 m/s (D 9.3.2)

## The track

Built by `src/track_autox.m` as a closed polygon with circular fillets, which
makes closure exact and every corner radius an explicit design input. Checked
against the layout guidelines the rules give for the DV Autocross (D 6.1.3 ->
D 8.1): closed loop, lap 200-500 m, straights <= 80 m, minimum turning diameter
9 m, minimum width 3 m. The layout intentionally contains a long straight into a
tight hairpin, a slalom, a decreasing-radius corner and a fast sweep, because
those are the features that separate the two laws.

## Layout

```
src/    track_autox.m      cone track and centreline geometry
        params_vehicle.m   vehicle and actuator parameters
        speed_profile.m    curvature / braking / acceleration limited profile
        ctrl_pure_pursuit.m, ctrl_stanley.m    the two steering laws
        path_nearest.m     nearest-point search and signed cross-track error
        vehicle_ode.m      kinematic bicycle + actuator + longitudinal PI
        run_reference.m    RK4 reference loop (also runs in Octave)
        kpi_compute.m      KPIs, cone contact, off-course, effective time
        fsg_points.m       DV Autocross score
        tune_gains.m       grid search, one shared objective
        tuning_robustness.m  how much of each gain space is usable
        sweep_width.m      points vs. track width, down to the 3 m minimum
        sweep_gain_latency.m  stability boundary in gain-vs-delay space
        plot_*.m           figures
        fig_new.m, fig_axes.m, fig_legend.m   figure surface and furniture
build_model.m              generates autox_lateral.slx from the same sources
verify_equivalence.m       Simulink vs. reference-loop agreement
run_all.m                  end-to-end driver
tests/run_tests.m          analytical unit tests
report/REPORT.md           the report, with the numbers run_all produced
```

The Simulink model is **generated by a script** rather than committed as a
binary: it diffs in git, it is not locked to one MATLAB release, and its
controller blocks read the same `.m` files as the reference loop, so the two
implementations cannot drift apart.
