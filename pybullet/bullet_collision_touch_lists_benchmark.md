# Touch-list benchmark: baseline vs discovered touch lists (N sweep)

Generated: 2026-05-29T12:17:48


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
| 2 | 333.2 | 384.7 | 1.15x | 15.01 | 13.00 | +13.4% | OK |
| 4 | 750.7 | 761.2 | 1.01x | 6.66 | 6.57 | +1.4% | OK |
| 6 | 858.5 | 872.1 | 1.02x | 5.82 | 5.73 | +1.6% | OK |
| 8 | 973.9 | 1033.0 | 1.06x | 5.13 | 4.84 | +5.7% | OK |
| 10 | 1023.9 | 1063.2 | 1.04x | 4.88 | 4.70 | +3.7% | OK |
| 12 | 614.1 | 863.9 | 1.41x | 8.14 | 5.79 | +28.9% | OK |

## Per-N detail


### N = 2

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 333.2 | 166.6 | 15.01 | 3479 | 0.0% |
| with_touch | 384.7 | 192.4 | 13.00 | 3479 | 0.1% |

### N = 4

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 750.7 | 187.7 | 6.66 | 3479 | 0.4% |
| with_touch | 761.2 | 190.3 | 6.57 | 3479 | 1.0% |

### N = 6

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 858.5 | 143.1 | 5.82 | 3479 | 2.5% |
| with_touch | 872.1 | 145.3 | 5.73 | 3479 | 1.1% |

### N = 8

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 973.9 | 121.7 | 5.13 | 3479 | 4.6% |
| with_touch | 1033.0 | 129.1 | 4.84 | 3479 | 3.1% |

### N = 10

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 1023.9 | 102.4 | 4.88 | 3479 | 3.7% |
| with_touch | 1063.2 | 106.3 | 4.70 | 3479 | 5.0% |

### N = 12

- Sanity: OK (3479 colliding configs)
| scenario | total_hz | per_inst_hz | wall_s | collisions | imbal% |
|:---------|---------:|------------:|-------:|-----------:|-------:|
| baseline | 614.1 | 51.2 | 8.14 | 3479 | 11.2% |
| with_touch | 863.9 | 72.0 | 5.79 | 3479 | 16.0% |

## Summary

- **Best speedup:** 1.41x at N=12
- **Worst speedup:** 1.01x at N=4
- A speedup < 1.0 means the touch-list bookkeeping cost exceeds the pair-check cost it saves for that worker count.

## Notes

- The `with_touch` scenario fills in `RigidBodyState.touch_links` / `touch_bodies` for every rigid body and tool using the `*_candidates` lists in the discovery JSON. These are pairs the discovery run never observed colliding, so they are skipped by the planner.
- Because every skipped pair was previously NEVER observed in collision, the count of colliding configurations should match between the two scenarios. A mismatch (`sanity != OK`) indicates the touch lists are too aggressive — rerun discovery for longer before trusting them.
- The N sweep isolates whether the touch-list cost/benefit ratio depends on worker count. It should not, in theory — both scenarios pay the same per-worker overhead — but if the per-call cost is dominated by Python scheduling, sweep behaviour can diverge.
