# Robot Model Collision-Check Comparison

Comparison of collision-check throughput between the JSON-embedded UR10e baseline, the same UR10e reloaded from `assets/ur10e_robot/`, and the candidate `assets/ur12e_robot/` with reduced-polygon collision meshes.

## Run configuration

- checks per variant: **2000**
- random seed: **20260529**
- backend: PyBullet `direct` (headless), single process
- configs: uniformly sampled in URDF joint limits; identical set across all variants and modes
- one untimed warmup check before timing begins; setup time is reported separately and excluded from `check_hz`

## Results

| variant | mode | N | collisions | setup_s | total_s | check_hz | mean_ms | p50_ms | p95_ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| json_ur10e | fail_fast | 2000 | 1250 | 0.938 | 2.946 | 678.81 | 1.4694 | 1.4458 | 2.0304 |
| json_ur10e | full_report | 2000 | 1250 | 0.895 | 4.052 | 493.58 | 2.0221 | 1.9522 | 2.4325 |
| assets_ur10e | fail_fast | 2000 | 1250 | 0.870 | 2.928 | 682.96 | 1.4605 | 1.4342 | 2.0220 |
| assets_ur10e | full_report | 2000 | 1250 | 0.843 | 4.035 | 495.65 | 2.0137 | 1.9507 | 2.4200 |
| assets_ur12e | fail_fast | 2000 | 1284 | 0.771 | 2.775 | 720.66 | 1.3840 | 1.3514 | 1.9582 |
| assets_ur12e | full_report | 2000 | 1284 | 0.770 | 3.871 | 516.64 | 1.9317 | 1.8643 | 2.2541 |

## Speedup vs json_ur10e baseline

| variant | fail_fast hz | x baseline | full_report hz | x baseline |
|---|---:|---:|---:|---:|
| json_ur10e | 678.81 | 1.000x | 493.58 | 1.000x |
| assets_ur10e | 682.96 | 1.006x | 495.65 | 1.004x |
| assets_ur12e | 720.66 | 1.062x | 516.64 | 1.047x |

## Notes

- `assets_ur10e` should match `json_ur10e` within run-to-run noise. A significant gap there would mean the model-swap path is not faithful and any ur12e numbers are suspect.
- `assets_ur12e` uses reduced-polygon collision meshes. The expectation is a measurable speedup; if there is none, the PyBullet broadphase or the convex-hull cache is already saturating before triangle count matters.
- Collision counts are reported per mode; identical configs are fed to all variants, so identical collision counts also serve as a consistency check between models.
