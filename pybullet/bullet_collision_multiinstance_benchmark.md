# Multi-instance PyBullet collision throughput

Generated: 2026-05-29T09:01:05


## Host

- Platform: `Windows-11-10.0.26200-SP0`
- Python: `3.12.13`
- CPU logical cores: `12`
- Processor: `Intel64 Family 6 Model 158 Stepping 10, GenuineIntel`


## Workload

- Total unique configs: **5000**
- N values tested: `[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]`
- Distribution: even-split contiguous chunks, one submit per worker
- Trajectory: step-indexed sinusoidal `mid + amp*sin(speed*t + i*0.9)` with `speed=0.6` rad/s, `dt=0.01` s
- Collision mode: `fail_fast` (default)
- Setup (client init, robot cell load, 1 warmup check): **excluded** from timing


## Results - Process backend (`ProcessPoolExecutor`)

| N | total_hz | per_inst_hz | effic | speedup | wall_s | imbal% |
|--:|---------:|------------:|------:|--------:|-------:|-------:|
| 1 | 208.0 | 208.0 | 100.0% | 1.00x | 24.04 | 0.0% |
| 2 | 392.3 | 196.1 | 94.3% | 1.89x | 12.75 | 0.3% |
| 3 | 551.0 | 183.7 | 88.3% | 2.65x | 9.07 | 3.1% |
| 4 | 678.8 | 169.7 | 81.6% | 3.26x | 7.37 | 1.8% |
| 5 | 736.1 | 147.2 | 70.8% | 3.54x | 6.79 | 2.3% |
| 6 | 770.5 | 128.4 | 61.7% | 3.70x | 6.49 | 2.9% |
| 7 | 797.9 | 114.0 | 54.8% | 3.84x | 6.27 | 3.2% |
| 8 | 845.2 | 105.7 | 50.8% | 4.06x | 5.92 | 4.2% |
| 9 | 866.3 | 96.3 | 46.3% | 4.16x | 5.77 | 5.1% |
| 10 | 889.0 | 88.9 | 42.7% | 4.27x | 5.62 | 7.1% |
| 11 | 866.1 | 78.7 | 37.9% | 4.16x | 5.77 | 10.3% |
| 12 | 696.0 | 58.0 | 27.9% | 3.35x | 7.18 | 14.9% |
| 13 | 767.7 | 59.1 | 28.4% | 3.69x | 6.51 | 25.4% |
| 14 | 732.1 | 52.3 | 25.1% | 3.52x | 6.83 | 27.2% |
| 15 | 701.3 | 46.8 | 22.5% | 3.37x | 7.13 | 30.5% |
| 16 | 673.6 | 42.1 | 20.2% | 3.24x | 7.42 | 30.0% |


## Results - Thread backend (`ThreadPoolExecutor`)

| N | total_hz | per_inst_hz | effic | speedup | wall_s | imbal% |
|--:|---------:|------------:|------:|--------:|-------:|-------:|
| 1 | 203.8 | 203.8 | 100.0% | 1.00x | 24.53 | 0.0% |
| 2 | 175.6 | 87.8 | 43.1% | 0.86x | 28.47 | 0.7% |
| 3 | 193.9 | 64.6 | 31.7% | 0.95x | 25.78 | 4.7% |
| 4 | 191.2 | 47.8 | 23.5% | 0.94x | 26.15 | 8.1% |
| 5 | 192.0 | 38.4 | 18.8% | 0.94x | 26.04 | 14.6% |
| 6 | 195.6 | 32.6 | 16.0% | 0.96x | 25.56 | 15.6% |
| 7 | 194.6 | 27.8 | 13.6% | 0.95x | 25.70 | 11.0% |
| 8 | 194.6 | 24.3 | 11.9% | 0.95x | 25.69 | 11.6% |
| 9 | 194.9 | 21.7 | 10.6% | 0.96x | 25.66 | 16.0% |
| 10 | 175.8 | 17.6 | 8.6% | 0.86x | 28.45 | 19.1% |
| 11 | 158.5 | 14.4 | 7.1% | 0.78x | 31.55 | 10.7% |
| 12 | 178.7 | 14.9 | 7.3% | 0.88x | 27.98 | 22.6% |
| 13 | 184.2 | 14.2 | 7.0% | 0.90x | 27.14 | 19.6% |
| 14 | 183.4 | 13.1 | 6.4% | 0.90x | 27.27 | 21.8% |
| 15 | 176.7 | 11.8 | 5.8% | 0.87x | 28.30 | 29.8% |
| 16 | 195.4 | 12.2 | 6.0% | 0.96x | 25.59 | 30.7% |


## Summary

- **Process best:** N=10 -> 889.0 checks/s (4.27x over N=1 process, 5.62s wall for 5000 configs)
- **Thread best:** N=1 -> 203.8 checks/s (1.00x over N=1 thread, 24.53s wall for 5000 configs)
- **Winner:** **process** backend (process 889.0 hz vs thread 203.8 hz, ratio 4.36x)

## Notes

- `total_hz` = `total_configs / wall_time_of_main_thread`. Wall time covers dispatch + chunk IPC + all worker compute + result gather, matching end-to-end throughput as seen from the caller.
- `imbal%` = `(max_worker - min_worker) / max_worker * 100`. High values indicate uneven collision-check cost per chunk (free vs collision configs).
- The thread backend shares one Python process. Empirically (see table) adding threads gives effectively **no speedup** for this workload — the GIL serialises either the PyBullet Python bindings, the compas_fab wrapper layer, or both. If you need parallelism, use the process backend.
- The process backend has higher per-call IPC cost (chunk pickling) but no GIL contention. It is the clear winner for N > 1 on this workload.
- Process throughput plateaus around N = number-of-physical-cores. On this 6P+6E (12 logical) machine the peak was near **N=10** (889 hz), then degraded as work spilled onto efficient/heterogeneous cores and OS scheduling cost rose. Treat N ≈ cores/2 as a safe sweet spot if other apps share the CPU.
- compas_fab's PyBullet setup is **NOT thread-safe**; this script serialises setup with a lock to avoid mesh-upload corruption. The lock affects warmup only and is not in the timed region.
