# Research Plan

## 1. Objective

Build a focused prototype environment for evaluating fast collision queries and
collision-aware control responses for a 6-axis haptically jogged robot arm.

The first phase is about evidence, not ideology. The new repository should make
it easy to answer:

- what primitive combinations are fast enough
- what query outputs are useful
- what response rules feel stable and intuitive
- which narrow-passage and corner cases break naive approaches

## 2. Core Hypotheses To Test

### H1: Analytic primitives are fast enough

Simple primitives such as spheres, oriented boxes, and planes can likely
support the required query rates for interactive control without mesh collision.

### H2: Distance information is more useful than boolean collision

Signed distance, penetration depth, contact normals, or nearest points are
likely needed to create usable collision-aware jogging.

### H3: A good response method matters as much as a good detector

Even if collision queries are fast, the overall behavior may still feel poor if
the response method causes freezing, oscillation, or direction reversals near
constraints.

### H4: Narrow passages are the real stress test

The system will likely appear to work in open space long before it behaves well
in corners, corridors, and trapped configurations.

## 3. Candidate Geometry Sets

Start with the simplest sets first.

### Set A: Spheres only

Robot:

- represent each link with one or more spheres

Environment:

- represent obstacles with spheres

Pros:

- simplest math
- easiest to batch
- easiest to gradientize

Cons:

- poor fit for flat walls and narrow passages unless many spheres are used

### Set B: Robot spheres + environment planes

Robot:

- spheres on links

Environment:

- infinite or bounded planes for simple walls/floors

Pros:

- very cheap wall interaction
- good for sliding tests

Cons:

- limited for localized obstacles

### Set C: Robot spheres + environment oriented boxes

Robot:

- spheres on links

Environment:

- oriented boxes for walls, posts, bins, bucket boundaries, and fixtures

Pros:

- much more practical
- still analytically simple enough to benchmark

Cons:

- more transform work per query

### Set D: Add capsules if needed

Only add this if sphere counts become excessive or behavior becomes too coarse.

## 4. Minimum Query API To Prototype

The repository should quickly converge on a small common API.

Suggested baseline:

```text
distance(scene, q) -> scene clearance report
distance_pair(a, b) -> pair clearance report
step_filter(q_current, q_desired, dt, scene) -> q_filtered
axis_sensitivity(q_current, q_desired, dt, scene) -> per-joint response hints
```

Suggested contents of a clearance report:

- minimum signed distance
- colliding / not colliding
- penetration depth if colliding
- nearest primitive pair identifiers
- contact normal or push-out direction
- optional per-joint sensitivity estimate

## 5. Response Methods To Compare

The repository should compare multiple response styles, not just one.

### Method 1: Hard reject

If the proposed step violates collision, reject or clamp the step.

Expected issue:

- likely too sticky
- poor sliding behavior

### Method 2: Clearance-weighted repulsion

Use a distance-based repulsive response that increases as clearance drops.

Expected benefit:

- smoother feel
- easier to back out

Expected issue:

- may distort motion too early

### Method 3: Projected motion

Remove only the component of desired motion that goes into the obstacle, while
preserving tangential motion.

Expected benefit:

- better surface following
- better "scratch along the wall" behavior

Expected issue:

- needs reliable normals and stable projection rules

### Method 4: Small constrained solve

Treat collision avoidance as a small optimization or constrained step in each
control cycle.

Expected benefit:

- may handle competing constraints better

Expected issue:

- may be too slow or too complex for first-stage work

## 6. Benchmark Plan

### Query-Level Benchmarks

Measure raw primitive query performance for:

- sphere vs sphere
- sphere vs plane
- sphere vs oriented box

Test at multiple scales:

- single query
- tens of queries
- hundreds of queries
- thousands of queries per cycle

Record:

- average time
- p95 / p99 time
- allocation behavior
- single-thread vs parallel speedup

### Scene-Level Benchmarks

Measure full-scene minimum-clearance queries using:

- a small robot model
- a medium robot model
- a crowded scene

Vary:

- number of robot primitives
- number of environment primitives
- number of active pair checks
- broad-phase culling on/off

### Control-Loop Benchmarks

Measure whether the full control step can stay comfortably inside budget for:

- 100 Hz host loop
- possible future higher-rate inner loops

Track:

- total planner time per cycle
- worst-case cycle time
- how performance changes near dense contact

## 7. Scenario Library

The new repository should define a reusable set of scenarios.

### Scenario 1: Open space

Purpose:

- establish baseline speed and behavior with no nearby obstacles

### Scenario 2: Single flat wall

Purpose:

- test push-into-wall and slide-along-wall behavior

### Scenario 3: Corner

Purpose:

- test competing constraints and backing out

### Scenario 4: Narrow corridor

Purpose:

- test whether the arm can move through a constrained passage without jitter

### Scenario 5: Cup / pocket / trap

Purpose:

- test escape behavior from partially enclosed geometry

### Scenario 6: Self-collision approach

Purpose:

- test arm-arm or wrist-link interference handling

### Scenario 7: Rapid direction reversal

Purpose:

- test stability when user intent flips sign quickly

### Scenario 8: Multi-axis competing inputs

Purpose:

- test whether multiple joint commands produce reasonable filtered motion

## 8. Success Criteria

The first phase can be considered successful if it produces evidence for all of
the following:

- at least one geometry set is fast enough for the target loop budget
- at least one response method supports sliding and backing out
- the system remains stable in corners and narrow passages
- the detector/response stack does not require mesh collision for this use case
- the resulting API is simple enough to integrate into the main game repo

## 9. Failure Criteria

The first phase should explicitly record failure if:

- query cost is too high even with simple primitives
- the behavior is too sticky or too unstable to be usable
- narrow passages consistently produce oscillation or deadlock
- the required gradient/sensitivity information is too noisy or too expensive

Those failures are still useful outcomes if they are documented clearly.

## 10. Suggested Repository Layout

```text
collision-research/
  README.md
  docs/
    decisions.md
    benchmark_notes.md
    scenarios.md
  collision/
    geometry/
    scenes/
    queries/
    response/
    benchmarks/
  tests/
    unit/
    scenarios/
    performance/
  notebooks_or_scripts/
```

This is only a suggestion. The key requirement is to keep:

- primitive query code
- response methods
- benchmark harnesses
- scenario definitions

cleanly separated.

## 11. Suggested Early Milestones

### Milestone 1: Primitive query harness

- implement sphere vs sphere
- implement sphere vs plane
- implement sphere vs oriented box
- benchmark them in isolation

### Milestone 2: Robot/environment scene mock-up

- define a simple robot-link primitive layout
- define a few environment scenes
- compute minimum scene clearance

### Milestone 3: First collision-aware filter

- compare hard reject vs projected motion vs repulsive method
- run interactive scenario tests

### Milestone 4: Stress and corner cases

- narrow passages
- corners
- rapid reversals
- multi-axis conflicting inputs

### Milestone 5: Integration recommendation

- choose preferred geometry set
- choose preferred response method
- define the integration API for the main game repo

## 12. Outputs To Bring Back To The Main Repo

The new repository should eventually hand back:

- a recommended geometry representation
- a recommended query API
- a recommended collision-aware step/filter algorithm
- benchmark evidence for why that choice was made
- a small set of validated scenarios that should become regression tests

