# Touch-list benchmark: baseline vs discovered touch lists (N sweep)

Generated: 2026-05-29T15:51:31


## Host

- Platform: `Windows-11-10.0.26200-SP0`
- Python: `3.12.13`
- CPU logical cores: `12`
- Processor: `Intel64 Family 6 Model 158 Stepping 10, GenuineIntel`


## Workload

- Total unique configs: **5000** per scenario
- N sweep: `[2, 4, 6, 8, 10, 12]`
- Chunk size: **10**  (500 chunks)
- Backend: `ProcessPoolExecutor`, `fail_fast` collision mode
- Setup excluded from timing (initializer + warmup pings)


## Discovery source

- File: `bullet_collision_pair_discovery.json`
- Discovery run: workers=10, duration/worker=600.0s, total_checks=633074, distinct pairs=58
- Bodies patched: **11**
- Tools patched: **1**
- Total `touch_links` entries added: **95**
- Total `touch_bodies` entries added: **110**

## Sweep results

| N | base_hz | touch_hz | speedup | base_wall_s | touch_wall_s | wall_save | sanity |
|--:|--------:|---------:|--------:|------------:|-------------:|----------:|:-------|
| 2 | 1212.5 | 1536.3 | 1.27x | 4.12 | 3.25 | +21.1% | OK |
| 4 | 2128.9 | 2631.1 | 1.24x | 2.35 | 1.90 | +19.1% | OK |
| 6 | 2612.6 | 3215.6 | 1.23x | 1.91 | 1.55 | +18.8% | OK |
| 8 | 2912.6 | 3492.1 | 1.20x | 1.72 | 1.43 | +16.6% | OK |
| 10 | 2897.3 | 3418.2 | 1.18x | 1.73 | 1.46 | +15.2% | OK |
| 12 | 2593.3 | 3420.0 | 1.32x | 1.93 | 1.46 | +24.2% | OK |

## Per-N detail


### N = 2

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 1212.5 | 606.3 | 4.12 | 3479 | 0.3% |
| with_touch | 1536.3 | 768.2 | 3.25 | 3479 | 0.1% |

### N = 4

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 2128.9 | 532.2 | 2.35 | 3479 | 1.7% |
| with_touch | 2631.1 | 657.8 | 1.90 | 3479 | 0.5% |

### N = 6

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 2612.6 | 435.4 | 1.91 | 3479 | 8.2% |
| with_touch | 3215.6 | 535.9 | 1.55 | 3479 | 11.4% |

### N = 8

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 2912.6 | 364.1 | 1.72 | 3479 | 6.9% |
| with_touch | 3492.1 | 436.5 | 1.43 | 3479 | 11.5% |

### N = 10

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 2897.3 | 289.7 | 1.73 | 3479 | 8.5% |
| with_touch | 3418.2 | 341.8 | 1.46 | 3479 | 15.3% |

### N = 12

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 2593.3 | 216.1 | 1.93 | 3479 | 27.6% |
| with_touch | 3420.0 | 285.0 | 1.46 | 3479 | 22.1% |

## Summary

- **Best speedup:** 1.32x at N=12
- **Worst speedup:** 1.18x at N=10
- A speedup < 1.0 means the touch-list bookkeeping cost exceeds the pair-check cost it saves for that worker count.

## Notes

- The `with_touch` scenario fills in `RigidBodyState.touch_links` / `touch_bodies` for every rigid body and tool using the `*_candidates` lists in the discovery JSON. These are pairs the discovery run never observed colliding, so they are skipped by the planner.
- Because every skipped pair was previously NEVER observed in collision, the count of colliding configurations should match between the two scenarios. A mismatch (`sanity != OK`) indicates the touch lists are too aggressive — rerun discovery for longer before trusting them.
- The N sweep isolates whether the touch-list cost/benefit ratio depends on worker count. It should not, in theory — both scenarios pay the same per-worker overhead — but if the per-call cost is dominated by Python scheduling, sweep behaviour can diverge.
