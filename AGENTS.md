# AGENTS.md

This file is intended to live at the root of the collision research repository.
It gives coding agents and human contributors a compact orientation to the
project, its priorities, and the rules for making progress.

## Purpose

This repository exists to research and prototype fast collision detection and
collision-aware motion filtering for a 6-axis haptically controlled robot arm.

This is a research-and-engineering repository, not a polished application
repository. The main goal is to discover what methods are fast enough,
controllable enough, and simple enough to integrate back into the main game
controller.

## Fixed Context

- Robot: UR12e
- Geometry assumption: treat UR12e as geometrically equivalent to UR10e unless
  a later measurement proves otherwise
- Degrees of freedom: 6
- Input model: 6 haptic dials controlling 6 robot joints
- Host control loop target: 100 Hz
- Haptic command/telemetry loop: 50 Hz
- Simulated robot physics reference: 200 Hz
- Current production integration target:
  insert a collision-aware planner/filter after rate limiting and before final
  robot command output

## Primary Objective

Answer these questions with evidence:

1. Can simple analytic primitives provide collision queries fast enough for the
   required control rates?
2. What query outputs are required to produce usable behavior?
3. What response method behaves well under simultaneous multi-axis jogging?
4. What geometry representation is simple enough to maintain and tune?

## Design Bias

Start simple and stay analytic by default.

Preferred first primitives:

- sphere
- plane
- oriented box

Possible later additions only if needed:

- capsule
- other simple convex primitives

Do not start with mesh collision unless the simpler approaches have already
been shown inadequate.

## Expected Behaviors To Optimize For

- safe motion near hard constraints
- ability to slide along surfaces
- ability to back out of corners and narrow passages
- stable response to rapid direction changes
- stable response to competing multi-axis inputs
- predictable behavior with low latency and low jitter

## Repository Priorities

Priority order:

1. Correctness of geometric queries
2. Reproducible benchmark coverage
3. Stable collision-aware response behavior
4. Clear scenario-based testing
5. Ease of integration back into the main repo

Polish is lower priority than evidence.

## Working Rules

- Keep benchmark code and production-candidate code separate where possible.
- Prefer deterministic scenario tests over ad-hoc manual experiments alone.
- Record failed ideas briefly instead of deleting them without notes.
- Avoid introducing heavyweight dependencies without a clear benchmark-based
  reason.
- Preserve small, inspectable geometry and scenario definitions.
- If performance claims are made, include timing evidence.

## Expected Outputs

Useful outputs from this repository include:

- primitive query implementations
- benchmark harnesses
- scenario definitions
- collision-response experiments
- a recommended query API
- a recommended integration shape for the main game repo

## Integration Back To Main Game Repo

The main game repo currently has a planning slot where:

- `planned_deg` is presently just equal to the rate-limited target

This repository should aim to produce a module or algorithm that can replace
that pass-through behavior with a collision-aware filtered target.

## First Things An Agent Should Read

1. `README.md`
2. `docs/PROJECT_BRIEF.md` or equivalent
3. `docs/RESEARCH_PLAN.md` or equivalent
4. `ai_context.json`

## First Things An Agent Should Build

1. Primitive query benchmark harness
2. Simple robot/environment scene representation
3. Scenario library for walls, corners, corridors, and self-collision cases
4. One or more candidate collision-aware step filters

