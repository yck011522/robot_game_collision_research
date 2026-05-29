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
| json_ur10e | fail_fast | 2000 | 1241 | 0.912 | 9.866 | 202.72 | 4.9257 | 4.8147 | 7.1510 |
| json_ur10e | full_report | 2000 | 1241 | 0.954 | 10.846 | 184.40 | 5.4158 | 5.1762 | 6.1992 |
| assets_ur10e | fail_fast | 2000 | 1241 | 0.869 | 9.281 | 215.50 | 4.6338 | 4.5984 | 5.4763 |
| assets_ur10e | full_report | 2000 | 1241 | 0.866 | 10.868 | 184.02 | 5.4270 | 5.2399 | 5.9784 |
| assets_ur12e | fail_fast | 2000 | 1277 | 0.892 | 9.226 | 216.78 | 4.6065 | 4.4851 | 5.5563 |
| assets_ur12e | full_report | 2000 | 1277 | 0.791 | 10.629 | 188.17 | 5.3074 | 5.0999 | 5.9317 |

## Speedup vs json_ur10e baseline

| variant | fail_fast hz | x baseline | full_report hz | x baseline |
|---|---:|---:|---:|---:|
| json_ur10e | 202.72 | 1.000x | 184.40 | 1.000x |
| assets_ur10e | 215.50 | 1.063x | 184.02 | 0.998x |
| assets_ur12e | 216.78 | 1.069x | 188.17 | 1.020x |

## Notes

- `assets_ur10e` should match `json_ur10e` within run-to-run noise. A significant gap there would mean the model-swap path is not faithful and any ur12e numbers are suspect.
- `assets_ur12e` uses reduced-polygon collision meshes. The expectation is a measurable speedup; if there is none, the PyBullet broadphase or the convex-hull cache is already saturating before triangle count matters.
- Collision counts are reported per mode; identical configs are fed to all variants, so identical collision counts also serve as a consistency check between models.
