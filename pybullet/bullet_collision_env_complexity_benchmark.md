# Environment Complexity Sensitivity

Effect of progressively removing static rigid bodies on collision-check throughput, measured in checks per second. Two robot model variants are run side-by-side on an identical, seeded set of random configurations.

## Run configuration

- checks per case: **1000**
- random seed: **20260529**
- backend: PyBullet `direct` (headless), single process
- mode: fail_fast
- starting rigid bodies (14): `RB0, RB1, RB2, RB3, RB4, RB5, RB6, RB7, RB8, RB9, RB10, RB11, RB12, RB13`
- removal order: descending by name (`RB13` removed first, then `RB12`, ...)
- tool `Bucket` is kept in all cases
- each case uses a fresh PyBullet client; setup time is reported but excluded from `check_hz`

## Results

| kept | last_removed | ur10e_hz | ur10e_x | ur10e_ms | ur10e_coll | ur12e_hz | ur12e_x | ur12e_ms | ur12e_coll |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 14 | - | 163.19 | 1.000x | 6.1190 | 624 | 152.50 | 1.000x | 6.5477 | 641 |
| 13 | RB13 | 173.32 | 1.062x | 5.7609 | 622 | 173.36 | 1.137x | 5.7601 | 639 |
| 12 | RB12 | 180.51 | 1.106x | 5.5313 | 617 | 180.85 | 1.186x | 5.5208 | 634 |
| 11 | RB11 | 186.23 | 1.141x | 5.3606 | 617 | 197.37 | 1.294x | 5.0584 | 634 |
| 10 | RB10 | 205.63 | 1.260x | 4.8550 | 614 | 205.26 | 1.346x | 4.8634 | 631 |
| 9 | RB9 | 208.66 | 1.279x | 4.7835 | 611 | 237.49 | 1.557x | 4.2037 | 628 |
| 8 | RB8 | 249.86 | 1.531x | 3.9954 | 510 | 255.83 | 1.678x | 3.9020 | 537 |
| 7 | RB7 | 270.78 | 1.659x | 3.6860 | 417 | 277.99 | 1.823x | 3.5904 | 447 |
| 6 | RB6 | 276.24 | 1.693x | 3.6122 | 411 | 301.09 | 1.974x | 3.3145 | 441 |
| 5 | RB5 | 318.21 | 1.950x | 3.1362 | 395 | 325.15 | 2.132x | 3.0689 | 425 |
| 4 | RB4 | 365.52 | 2.240x | 2.7289 | 370 | 439.89 | 2.885x | 2.2677 | 401 |
| 3 | RB3 | 313.99 | 1.924x | 3.1759 | 335 | 400.05 | 2.623x | 2.4932 | 370 |
| 2 | RB2 | 448.51 | 2.748x | 2.2233 | 333 | 452.74 | 2.969x | 2.2025 | 368 |
| 1 | RB1 | 516.96 | 3.168x | 1.9280 | 293 | 528.84 | 3.468x | 1.8849 | 328 |
| 0 | RB0 | 618.89 | 3.792x | 1.6099 | 293 | 630.10 | 4.132x | 1.5812 | 328 |

## Notes

- `kept` counts only rigid bodies. The robot itself (always self-checked) and the `Bucket` tool are present in every row.
- `ur10e_x` and `ur12e_x` are speedup multiples vs the full-scene row (top row) for that variant.
- A flat curve means environment rigid bodies contribute very little to the per-check cost — most time is spent on robot self-collision and the tool. A steep curve means environment pairs dominate, and reducing/simplifying environment geometry would pay off in the integration loop.
- Collision counts will drop as bodies are removed; this is expected and does not affect timing fairness because per-check wall-clock is measured around the call regardless of outcome.
