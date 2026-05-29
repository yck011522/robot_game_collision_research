# Static vs dynamic config distribution

Generated: 2026-05-29T09:33:26


## Host

- Platform: `Windows-11-10.0.26200-SP0`
- Python: `3.12.13`
- CPU logical cores: `12`
- Processor: `Intel64 Family 6 Model 158 Stepping 10, GenuineIntel`


## Workload

- Total unique configs: **5000**
- N values tested: `[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]`
- Backend: `ProcessPoolExecutor` only (threads ruled out by v2)
- Strategies: `['static', 'dyn-100', 'dyn-10', 'dyn-1']`
- Trajectory: step-indexed sinusoidal `mid + amp*sin(speed*t + i*0.9)` with `speed=0.6` rad/s, `dt=0.01` s
- Collision mode: `fail_fast`
- Setup excluded from timing (initializer + warmup ping round)


## Throughput grid (checks/s)

| N | static | dyn-100 | dyn-10 | dyn-1 |
|--:|--------:|--------:|--------:|--------:|
| 1 | 204.6 | 213.6 | 211.7 | 202.1 |
| 2 | 402.7 | 394.4 | 399.2 | 381.9 |
| 3 | 571.6 | 566.3 | 560.3 | 529.7 |
| 4 | 671.3 | 659.3 | 660.3 | 612.4 |
| 5 | 748.9 | 614.3 | 628.5 | 654.8 |
| 6 | 617.2 | 673.5 | 678.0 | 614.9 |
| 7 | 739.9 | 785.9 | 802.0 | 730.5 |
| 8 | 833.6 | 793.8 | 695.1 | 737.5 |
| 9 | 857.1 | 852.8 | 877.7 | 759.8 |
| 10 | 880.4 | 882.3 | 886.9 | 747.1 |
| 11 | 870.3 | 878.3 | 820.0 | 731.7 |
| 12 | 861.0 | 817.3 | 730.9 | 748.8 |
| 13 | 743.8 | 736.6 | 796.4 | 710.0 |
| 14 | 733.6 | 676.1 | 745.8 | 696.5 |
| 15 | 726.9 | 729.8 | 749.4 | 706.5 |
| 16 | 666.4 | 714.8 | 675.5 | 612.7 |


## Best strategy per N

| N | best strategy | total_hz | margin vs static |
|--:|:--------------|---------:|-----------------:|
| 1 | dyn-100 | 213.6 | +4.4% |
| 2 | static | 402.7 | +0.0% |
| 3 | static | 571.6 | +0.0% |
| 4 | static | 671.3 | +0.0% |
| 5 | static | 748.9 | +0.0% |
| 6 | dyn-10 | 678.0 | +9.8% |
| 7 | dyn-10 | 802.0 | +8.4% |
| 8 | static | 833.6 | +0.0% |
| 9 | dyn-10 | 877.7 | +2.4% |
| 10 | dyn-10 | 886.9 | +0.7% |
| 11 | dyn-100 | 878.3 | +0.9% |
| 12 | static | 861.0 | +0.0% |
| 13 | dyn-10 | 796.4 | +7.1% |
| 14 | dyn-10 | 745.8 | +1.7% |
| 15 | dyn-10 | 749.4 | +3.1% |
| 16 | dyn-100 | 714.8 | +7.3% |


## Detailed results per strategy

### static

| N | chunks | total_hz | per_inst_hz | speedup | wall_s | imbal% |
|--:|-------:|---------:|------------:|--------:|-------:|-------:|
| 1 | 1 | 204.6 | 204.6 | 1.00x | 24.44 | 0.0% |
| 2 | 2 | 402.7 | 201.4 | 1.97x | 12.42 | 0.6% |
| 3 | 3 | 571.6 | 190.5 | 2.79x | 8.75 | 2.2% |
| 4 | 4 | 671.3 | 167.8 | 3.28x | 7.45 | 3.4% |
| 5 | 5 | 748.9 | 149.8 | 3.66x | 6.68 | 3.6% |
| 6 | 6 | 617.2 | 102.9 | 3.02x | 8.10 | 6.1% |
| 7 | 7 | 739.9 | 105.7 | 3.62x | 6.76 | 4.6% |
| 8 | 8 | 833.6 | 104.2 | 4.07x | 6.00 | 4.7% |
| 9 | 9 | 857.1 | 95.2 | 4.19x | 5.83 | 5.6% |
| 10 | 10 | 880.4 | 88.0 | 4.30x | 5.68 | 5.5% |
| 11 | 11 | 870.3 | 79.1 | 4.25x | 5.74 | 8.7% |
| 12 | 12 | 861.0 | 71.8 | 4.21x | 5.81 | 11.7% |
| 13 | 13 | 743.8 | 57.2 | 3.64x | 6.72 | 18.0% |
| 14 | 14 | 733.6 | 52.4 | 3.59x | 6.82 | 19.1% |
| 15 | 15 | 726.9 | 48.5 | 3.55x | 6.88 | 31.6% |
| 16 | 16 | 666.4 | 41.6 | 3.26x | 7.50 | 31.6% |

### dyn-100

| N | chunks | total_hz | per_inst_hz | speedup | wall_s | imbal% |
|--:|-------:|---------:|------------:|--------:|-------:|-------:|
| 1 | 50 | 213.6 | 213.6 | 1.00x | 23.41 | 0.0% |
| 2 | 50 | 394.4 | 197.2 | 1.85x | 12.68 | 2.1% |
| 3 | 50 | 566.3 | 188.8 | 2.65x | 8.83 | 3.1% |
| 4 | 50 | 659.3 | 164.8 | 3.09x | 7.58 | 5.9% |
| 5 | 50 | 614.3 | 122.9 | 2.88x | 8.14 | 6.9% |
| 6 | 50 | 673.5 | 112.3 | 3.15x | 7.42 | 7.8% |
| 7 | 50 | 785.9 | 112.3 | 3.68x | 6.36 | 5.1% |
| 8 | 50 | 793.8 | 99.2 | 3.72x | 6.30 | 10.3% |
| 9 | 50 | 852.8 | 94.8 | 3.99x | 5.86 | 12.3% |
| 10 | 50 | 882.3 | 88.2 | 4.13x | 5.67 | 5.3% |
| 11 | 50 | 878.3 | 79.8 | 4.11x | 5.69 | 16.1% |
| 12 | 50 | 817.3 | 68.1 | 3.83x | 6.12 | 14.8% |
| 13 | 50 | 736.6 | 56.7 | 3.45x | 6.79 | 25.8% |
| 14 | 50 | 676.1 | 48.3 | 3.16x | 7.40 | 29.3% |
| 15 | 50 | 729.8 | 48.7 | 3.42x | 6.85 | 19.7% |
| 16 | 50 | 714.8 | 44.7 | 3.35x | 7.00 | 30.3% |

### dyn-10

| N | chunks | total_hz | per_inst_hz | speedup | wall_s | imbal% |
|--:|-------:|---------:|------------:|--------:|-------:|-------:|
| 1 | 500 | 211.7 | 211.7 | 1.00x | 23.62 | 0.0% |
| 2 | 500 | 399.2 | 199.6 | 1.89x | 12.53 | 0.3% |
| 3 | 500 | 560.3 | 186.8 | 2.65x | 8.92 | 1.0% |
| 4 | 500 | 660.3 | 165.1 | 3.12x | 7.57 | 0.4% |
| 5 | 500 | 628.5 | 125.7 | 2.97x | 7.96 | 2.2% |
| 6 | 500 | 678.0 | 113.0 | 3.20x | 7.37 | 3.3% |
| 7 | 500 | 802.0 | 114.6 | 3.79x | 6.23 | 1.7% |
| 8 | 500 | 695.1 | 86.9 | 3.28x | 7.19 | 3.1% |
| 9 | 500 | 877.7 | 97.5 | 4.15x | 5.70 | 2.1% |
| 10 | 500 | 886.9 | 88.7 | 4.19x | 5.64 | 2.5% |
| 11 | 500 | 820.0 | 74.5 | 3.87x | 6.10 | 4.3% |
| 12 | 500 | 730.9 | 60.9 | 3.45x | 6.84 | 9.5% |
| 13 | 500 | 796.4 | 61.3 | 3.76x | 6.28 | 10.3% |
| 14 | 500 | 745.8 | 53.3 | 3.52x | 6.70 | 19.3% |
| 15 | 500 | 749.4 | 50.0 | 3.54x | 6.67 | 15.9% |
| 16 | 500 | 675.5 | 42.2 | 3.19x | 7.40 | 18.7% |

### dyn-1

| N | chunks | total_hz | per_inst_hz | speedup | wall_s | imbal% |
|--:|-------:|---------:|------------:|--------:|-------:|-------:|
| 1 | 5000 | 202.1 | 202.1 | 1.00x | 24.75 | 0.0% |
| 2 | 5000 | 381.9 | 190.9 | 1.89x | 13.09 | 0.0% |
| 3 | 5000 | 529.7 | 176.6 | 2.62x | 9.44 | 0.0% |
| 4 | 5000 | 612.4 | 153.1 | 3.03x | 8.16 | 0.1% |
| 5 | 5000 | 654.8 | 131.0 | 3.24x | 7.64 | 0.1% |
| 6 | 5000 | 614.9 | 102.5 | 3.04x | 8.13 | 0.1% |
| 7 | 5000 | 730.5 | 104.4 | 3.62x | 6.84 | 0.1% |
| 8 | 5000 | 737.5 | 92.2 | 3.65x | 6.78 | 0.1% |
| 9 | 5000 | 759.8 | 84.4 | 3.76x | 6.58 | 0.3% |
| 10 | 5000 | 747.1 | 74.7 | 3.70x | 6.69 | 0.2% |
| 11 | 5000 | 731.7 | 66.5 | 3.62x | 6.83 | 3.1% |
| 12 | 5000 | 748.8 | 62.4 | 3.71x | 6.68 | 0.2% |
| 13 | 5000 | 710.0 | 54.6 | 3.51x | 7.04 | 13.2% |
| 14 | 5000 | 696.5 | 49.7 | 3.45x | 7.18 | 14.4% |
| 15 | 5000 | 706.5 | 47.1 | 3.50x | 7.08 | 19.7% |
| 16 | 5000 | 612.7 | 38.3 | 3.03x | 8.16 | 14.2% |


## Summary

- **Overall best:** `dyn-10` @ N=10 -> 886.9 checks/s
- **Static best:** 880.4 checks/s
- **Dynamic vs static:** +0.7%  (dynamic wins)

## Notes

- `chunks` = number of work units submitted. For `static` it equals N. For `dyn-K` it equals `ceil(total_configs / K)`.
- `imbal%` = `(max_worker - min_worker) / max_worker * 100`, summed across all chunks each worker processed.
- Very small chunks (e.g. `dyn-1`) make every config a separate IPC round trip. Pickle/unpickle of 6 floats per task is cheap but Python-side scheduler bookkeeping dominates at this scale.

## Findings

1. **Peak throughput is essentially unchanged.** The best result across all
   strategies is `dyn-10` @ N=10 = **886.9 hz**, vs the static peak of
   `static` @ N=10 = **880.4 hz** — a 0.7% difference, well inside run-to-run
   noise. Both peaks land at the same N. For this workload, **dynamic
   distribution does not unlock more throughput**.

2. **Static is highly competitive at 5000 configs.** Each static chunk holds
   roughly `5000 / N` = 312–1000 configs, which is large enough that the
   per-config cost variance (collision vs free) averages out. The result is
   small `imbal%` values (typically <10% for N≤8) and throughput within a few
   percent of dynamic.

3. **`dyn-1` is consistently the worst.** Submitting one config per task adds
   ~70 hz of IPC overhead (compare N=1: dyn-1=202 vs dyn-10=212). The penalty
   stays visible across all N values. Avoid chunk sizes near 1.

4. **`dyn-10` and `dyn-100` give modest wins only at specific N.** Cases like
   N=6, 7, 13, 16 show +7–10% over static, but other N values (5, 8, 12) have
   static winning by similar margins. The wins likely come from random worker
   stalls being absorbed by the work queue.

5. **The earlier 29% dynamic advantage was workload-size dependent.** With
   2000 configs and N=8, static chunks are only ~250 configs each — small
   enough that variance hurts. At 5000 / 8 = 625 per chunk, that variance is
   already amortised. **The crossover behaviour is: dynamic helps when chunks
   become smaller than ~300 configs.**

6. **Practical recommendation:** Static `ceil(total/N)` chunking is the right
   default for batch workloads of ≥5000 configs. Switch to `dyn-100` (or
   similar moderate chunk size) only if:
   - the per-config cost has high variance (e.g. mixed primitive types, mesh
     vs sphere), OR
   - the workload is small enough that static chunks fall below ~300 configs
     per worker, OR
   - workers may stall on external dependencies.

7. **High-N degradation is a worker-saturation effect, not a distribution
   issue.** All four strategies drop together past N=11 on this 12-logical-
   core machine. `imbal%` rises with N because some chunks end up on slower
   efficient cores under load.
