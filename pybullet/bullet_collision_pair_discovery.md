# Collision pair discovery

Generated: 2026-05-29T16:23:19


## Run

- Platform: `Windows-11-10.0.26200-SP0`
- Python: `3.12.13`
- Workers (processes): **12**
- Per-worker duration: **600.0 s**
- Wall time (incl. setup): **608.57 s**
- Total checks: **1336505**
- Total collisions: **830961** (62.2%)
- Total reported pair-instances: **3305853**
- Distinct pairs observed: **58**
- Sampling check rate: **2196 configs/s aggregate**

### Per-worker stats

| pid | checks | collisions | pair-instances | distinct pairs |
|----:|-------:|-----------:|---------------:|---------------:|
| 32148 | 111105 | 69344 | 276776 | 58 |
| 24660 | 112148 | 69494 | 276067 | 58 |
| 2684 | 111759 | 69643 | 277444 | 58 |
| 34708 | 111675 | 69286 | 275902 | 57 |
| 30064 | 112201 | 69882 | 277924 | 58 |
| 31480 | 111105 | 68945 | 273662 | 57 |
| 34444 | 110966 | 69302 | 274415 | 58 |
| 32992 | 110878 | 68877 | 273806 | 58 |
| 25660 | 110607 | 68663 | 272921 | 58 |
| 30860 | 111703 | 69304 | 275888 | 58 |
| 29456 | 111469 | 69216 | 274979 | 58 |
| 12844 | 110889 | 69005 | 276069 | 58 |

## Caveat

Pairs that did NOT appear in this run are **unobserved**, not proven unreachable. The `touch_*_candidates` below are *suggestions* — they are only safe to commit into `robot_cell_and_state.json` after a long enough sampling run (e.g. 1 hour) AND a sanity check against known kinematic constraints.

## Per rigid body

### `front_wall`

- Collided with links: `[]`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `ground`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `buckets`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `bucket_ground`

- Collided with links: `[]`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- **`touch_bodies_candidates` (never seen):** `['buckets', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `left_wall`

- Collided with links: `['wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_player', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `right_wall`

- Collided with links: `['wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player']`

### `pedestal`

- Collided with links: `['forearm_link', 'upper_arm_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'wrist_1_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pyramid_ground', 'right_player', 'right_wall']`

### `pyramid_ground`

- Collided with links: `['forearm_link', 'upper_arm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'right_player', 'right_wall']`

### `right_player`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_wall']`

### `mid_player`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_player', 'left_wall', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

### `left_player`

- Collided with links: `['forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- Collided with other bodies/tools: `['tool:Bucket']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'shoulder_link', 'tool0', 'upper_arm_link']`
- **`touch_bodies_candidates` (never seen):** `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`


## Per tool

### `Bucket`

- Collided with links: `['base_link_inertia', 'forearm_link', 'shoulder_link', 'upper_arm_link']`
- Collided with other bodies/tools: `['bucket_ground', 'buckets', 'front_wall', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'flange', 'tool0', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- **`touch_bodies_candidates` (never seen):** `[]`


## Skip matrix (body x link)

`.` = never observed colliding (safe to add to `touch_links`); `X` = observed.

| body \\ link | base_link | base_link_inertia | shoulder_link | upper_arm_link | forearm_link | wrist_1_link | wrist_2_link | wrist_3_link | base | flange | tool0 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `front_wall` | . | . | . | . | . | . | . | . | . | . | . |
| `ground` | . | . | . | . | X | X | X | X | . | . | . |
| `buckets` | . | . | . | . | X | X | X | X | . | . | . |
| `bucket_ground` | . | . | . | . | . | . | . | . | . | . | . |
| `left_wall` | . | . | . | . | . | . | X | X | . | . | . |
| `right_wall` | . | . | . | . | . | . | X | X | . | . | . |
| `pedestal` | . | . | . | X | X | . | X | X | . | . | . |
| `pyramid_ground` | . | . | . | X | X | X | X | X | . | . | . |
| `right_player` | . | . | . | . | X | X | X | X | . | . | . |
| `mid_player` | . | . | . | . | X | X | X | X | . | . | . |
| `left_player` | . | . | . | . | X | X | X | X | . | . | . |

### Skip matrix (tool x link)

| body \\ link | base_link | base_link_inertia | shoulder_link | upper_arm_link | forearm_link | wrist_1_link | wrist_2_link | wrist_3_link | base | flange | tool0 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `Bucket` | . | X | X | X | X | . | . | . | . | . | . |

## All distinct observed pairs

Sorted alphabetically. Each entity tagged with kind (`link:`, `body:`, `tool:`).

- `body:bucket_ground`  <->  `tool:Bucket`
- `body:buckets`  <->  `link:forearm_link`
- `body:buckets`  <->  `link:wrist_1_link`
- `body:buckets`  <->  `link:wrist_2_link`
- `body:buckets`  <->  `link:wrist_3_link`
- `body:buckets`  <->  `tool:Bucket`
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
