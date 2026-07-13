# Collision pair discovery

Generated: 2026-07-13T14:09:53


## Run

- Platform: `Windows-11-10.0.26200-SP0`
- Python: `3.12.13`
- Workers (processes): **10**
- Per-worker duration: **600.0 s**
- Wall time (incl. setup): **606.96 s**
- Total checks: **3274849**
- Total collisions: **2424868** (74.0%)
- Total reported pair-instances: **10241793**
- Distinct pairs observed: **65**
- Sampling check rate: **5396 configs/s aggregate**

### Per-worker stats

| pid | checks | collisions | pair-instances | distinct pairs |
|----:|-------:|-----------:|---------------:|---------------:|
| 51064 | 327707 | 242656 | 1024354 | 65 |
| 62708 | 328599 | 243289 | 1028044 | 65 |
| 28636 | 327750 | 242672 | 1025733 | 65 |
| 30132 | 328876 | 243462 | 1028631 | 65 |
| 64968 | 327522 | 242035 | 1022487 | 65 |
| 72236 | 324573 | 240662 | 1017514 | 65 |
| 68908 | 328764 | 243259 | 1027092 | 65 |
| 45652 | 323431 | 239562 | 1010592 | 65 |
| 20656 | 328738 | 243470 | 1027396 | 65 |
| 50564 | 328889 | 243801 | 1029950 | 65 |

## Caveat

Pairs that did NOT appear in this run are **unobserved**, not proven unreachable. The `touch_*_candidates` below are *suggestions* — they are only safe to commit into `robot_cell_and_state.json` after a long enough sampling run (e.g. 1 hour) AND a sanity check against known kinematic constraints.

## Per rigid body

### `bucket_ground`

- Collided with links: `[]`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- **`touch_bodies_candidates` (never seen):** `['buckets', 'ceiling', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `ceiling`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `right_player`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_wall']`

### `mid_player`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'ground', 'left_player', 'left_wall', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `left_player`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'ground', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `front_wall`

- Collided with links: `[]`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `ground`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `pedestal`

- Collided with links: `['forearm_link', 'upper_arm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pyramid_ground', 'right_player', 'right_wall']`

### `pyramid_ground`

- Collided with links: `['forearm_link', 'upper_arm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'right_player', 'right_wall']`

### `left_wall`

- Collided with links: `['wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'ground', 'left_player', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `right_wall`

- Collided with links: `['wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player']`

### `buckets`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'ceiling', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`


## Per tool

### `Bucket`

- Collided with links: `['base_link_inertia', 'forearm_link', 'shoulder_link', 'upper_arm_link', 'wrist_1_link']`
- Collided with other bodies/tools: `['bucket_ground', 'buckets', 'ceiling', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'flange', 'tool0', 'wrist_2_link', 'wrist_3_link']`
- **`touch_bodies_candidates` (never seen):** `[]`


## Skip matrix (body x link)

`.` = never observed colliding (safe to add to `touch_links`); `X` = observed.

| body \\ link | base_link | base_link_inertia | shoulder_link | upper_arm_link | forearm_link | wrist_1_link | wrist_2_link | wrist_3_link | base | flange | tool0 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `bucket_ground` | . | . | . | . | . | . | . | . | . | . | . |
| `ceiling` | . | . | . | . | X | X | X | X | . | . | . |
| `right_player` | . | . | . | . | X | X | X | X | . | . | . |
| `mid_player` | . | . | . | . | X | X | X | X | . | . | . |
| `left_player` | . | . | . | . | X | X | X | X | . | . | . |
| `front_wall` | . | . | . | . | . | . | . | . | . | . | . |
| `ground` | . | . | . | . | X | X | X | X | . | . | . |
| `pedestal` | . | . | . | X | X | X | X | X | . | . | . |
| `pyramid_ground` | . | . | . | X | X | X | X | X | . | . | . |
| `left_wall` | . | . | . | . | . | . | X | X | . | . | . |
| `right_wall` | . | . | . | . | . | . | X | X | . | . | . |
| `buckets` | . | . | . | . | X | X | X | X | . | . | . |

### Skip matrix (tool x link)

| body \\ link | base_link | base_link_inertia | shoulder_link | upper_arm_link | forearm_link | wrist_1_link | wrist_2_link | wrist_3_link | base | flange | tool0 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `Bucket` | . | X | X | X | X | X | . | . | . | . | . |

## All distinct observed pairs

Sorted alphabetically. Each entity tagged with kind (`link:`, `body:`, `tool:`).

- `body:bucket_ground`  <->  `tool:Bucket`
- `body:buckets`  <->  `link:forearm_link`
- `body:buckets`  <->  `link:wrist_1_link`
- `body:buckets`  <->  `link:wrist_2_link`
- `body:buckets`  <->  `link:wrist_3_link`
- `body:buckets`  <->  `tool:Bucket`
- `body:ceiling`  <->  `link:forearm_link`
- `body:ceiling`  <->  `link:wrist_1_link`
- `body:ceiling`  <->  `link:wrist_2_link`
- `body:ceiling`  <->  `link:wrist_3_link`
- `body:ceiling`  <->  `tool:Bucket`
- `body:front_wall`  <->  `tool:Bucket`
- `body:ground`  <->  `link:forearm_link`
- `body:ground`  <->  `link:wrist_1_link`
- `body:ground`  <->  `link:wrist_2_link`
- `body:ground`  <->  `link:wrist_3_link`
- `body:ground`  <->  `tool:Bucket`
- `body:left_player`  <->  `link:forearm_link`
- `body:left_player`  <->  `link:wrist_1_link`
- `body:left_player`  <->  `link:wrist_2_link`
- `body:left_player`  <->  `link:wrist_3_link`
- `body:left_player`  <->  `tool:Bucket`
- `body:left_wall`  <->  `link:wrist_2_link`
- `body:left_wall`  <->  `link:wrist_3_link`
- `body:left_wall`  <->  `tool:Bucket`
- `body:mid_player`  <->  `link:forearm_link`
- `body:mid_player`  <->  `link:wrist_1_link`
- `body:mid_player`  <->  `link:wrist_2_link`
- `body:mid_player`  <->  `link:wrist_3_link`
- `body:mid_player`  <->  `tool:Bucket`
- `body:pedestal`  <->  `link:forearm_link`
- `body:pedestal`  <->  `link:upper_arm_link`
- `body:pedestal`  <->  `link:wrist_1_link`
- `body:pedestal`  <->  `link:wrist_2_link`
- `body:pedestal`  <->  `link:wrist_3_link`
- `body:pedestal`  <->  `tool:Bucket`
- `body:pyramid_ground`  <->  `link:forearm_link`
- `body:pyramid_ground`  <->  `link:upper_arm_link`
- `body:pyramid_ground`  <->  `link:wrist_1_link`
- `body:pyramid_ground`  <->  `link:wrist_2_link`
- `body:pyramid_ground`  <->  `link:wrist_3_link`
- `body:pyramid_ground`  <->  `tool:Bucket`
- `body:right_player`  <->  `link:forearm_link`
- `body:right_player`  <->  `link:wrist_1_link`
- `body:right_player`  <->  `link:wrist_2_link`
- `body:right_player`  <->  `link:wrist_3_link`
- `body:right_player`  <->  `tool:Bucket`
- `body:right_wall`  <->  `link:wrist_2_link`
- `body:right_wall`  <->  `link:wrist_3_link`
- `body:right_wall`  <->  `tool:Bucket`
- `link:base_link_inertia`  <->  `link:forearm_link`
- `link:base_link_inertia`  <->  `link:wrist_2_link`
- `link:base_link_inertia`  <->  `link:wrist_3_link`
- `link:base_link_inertia`  <->  `tool:Bucket`
- `link:forearm_link`  <->  `link:shoulder_link`
- `link:forearm_link`  <->  `link:wrist_2_link`
- `link:forearm_link`  <->  `link:wrist_3_link`
- `link:forearm_link`  <->  `tool:Bucket`
- `link:shoulder_link`  <->  `link:wrist_3_link`
- `link:shoulder_link`  <->  `tool:Bucket`
- `link:upper_arm_link`  <->  `link:wrist_1_link`
- `link:upper_arm_link`  <->  `link:wrist_2_link`
- `link:upper_arm_link`  <->  `link:wrist_3_link`
- `link:upper_arm_link`  <->  `tool:Bucket`
- `link:wrist_1_link`  <->  `tool:Bucket`
