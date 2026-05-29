# Collision pair discovery

Generated: 2026-05-29T11:48:02


## Run

- Platform: `Windows-11-10.0.26200-SP0`
- Python: `3.12.13`
- Workers (processes): **10**
- Per-worker duration: **60.0 s**
- Wall time (incl. setup): **67.39 s**
- Total checks: **61793**
- Total collisions: **38347** (62.1%)
- Total reported pair-instances: **152897**
- Distinct pairs observed: **57**
- Sampling check rate: **917 configs/s aggregate**

### Per-worker stats

| pid | checks | collisions | pair-instances | distinct pairs |
|----:|-------:|-----------:|---------------:|---------------:|
| 30240 | 6111 | 3764 | 15022 | 55 |
| 24164 | 6234 | 3832 | 15033 | 53 |
| 32488 | 6238 | 3901 | 15113 | 54 |
| 5984 | 6128 | 3833 | 15342 | 55 |
| 32112 | 6133 | 3782 | 15096 | 56 |
| 5484 | 6194 | 3863 | 15600 | 55 |
| 12460 | 6136 | 3836 | 15508 | 53 |
| 21972 | 6188 | 3874 | 15642 | 54 |
| 32028 | 6202 | 3818 | 15242 | 55 |
| 29348 | 6229 | 3844 | 15299 | 57 |

## Caveat

Pairs that did NOT appear in this run are **unobserved**, not proven unreachable. The `touch_*_candidates` below are *suggestions* — they are only safe to commit into `robot_cell_and_state.json` after a long enough sampling run (e.g. 1 hour) AND a sanity check against known kinematic constraints.

## Per rigid body

### `front_wall`

- Collided with links: `[]`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'base_link_inertia', 'flange', 'forearm_link', 'shoulder_link', 'tool0', 'upper_arm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- **`touch_bodies_candidates` (never seen):** `['Bucket', 'bucket_ground', 'buckets', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`

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
- Collided with other bodies/tools: `['bucket_ground', 'buckets', 'ground', 'left_player', 'left_wall', 'mid_player', 'pedestal', 'pyramid_ground', 'right_player', 'right_wall']`
- **`touch_links_candidates` (never seen):** `['base', 'base_link', 'flange', 'tool0', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']`
- **`touch_bodies_candidates` (never seen):** `['front_wall']`


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
