# Project Brief

## 1. Purpose

This project exists to research, prototype, benchmark, and de-risk collision
detection and collision-aware motion filtering for a haptically controlled
robotic arm game.

The expected output is not just a collision checker. The long-term production
need is a motion-planning or motion-filtering stage that can sit between
"user jogging intent" and "robot command output" while preserving:

- real-time responsiveness
- safety against self-collision and environment collision
- intuitive behavior under multi-axis human input
- stable backing-out behavior when trapped or near hard constraints
- the ability to slide along allowed surfaces instead of freezing

## 2. Why A Separate Repository

The collision problem is likely to require:

- multiple geometry mock-ups
- benchmarking of many primitive/query combinations
- rapid prototyping of alternative planners and constraint handlers
- scenario libraries for corner cases
- aggressive performance experiments, possibly including parallel methods

Keeping that work separate will make it easier to:

- keep the main game repository smaller and more focused
- iterate on research code without destabilizing production-facing code
- document failed ideas as well as successful ones

## 3. Fixed Context From The Main Game

These assumptions are already decided unless explicitly changed later.

### Robot and Control Context

- The robot is a UR12e.
- For geometry and kinematics purposes, treat the UR12e as the same shape class
  as the UR10e unless a later measurement proves otherwise.
- The game uses a 6-axis robotic arm.
- Each axis is controlled by one haptic dial.
- In the museum/game setting there may be fewer than 6 active players, but the
  system should still assume 6 controllable axes.

### Input / Control Pipeline Context

- User input is continuous jogging-like joint intent coming from haptic dials.
- The current host-side pipeline is:
  `raw dial -> joint command -> joint clamp -> joint rate limit -> planned target`
- The future collision module is expected to operate after the current
  rate-limiter stage and before the final robot command is sent.
- In the current codebase, `planned_deg` is the intended insertion point for the
  future collision-aware planner/filter.

### Real-Time Constraints

- The current game loop target is 100 Hz on the host.
- Haptic command/telemetry exchange runs at 50 Hz.
- The current simulated robot physics loop runs at 200 Hz.
- Human input can change abruptly and can reverse direction suddenly.
- Multiple joints can be driven simultaneously in competing directions.

### Current Default Tuning In The Main Repo

- Global gear ratio: 10.0 dial degrees per 1.0 joint degree
- Dial-side rate limit: 5.0 deg/s in joint space
- Robot max velocity: 30.0 deg/s
- Default static joint limits: -180 deg to +180 deg for all 6 joints

These values may change later, but they are a valid starting point for test
scenarios and mock-ups.

### Game / Environment Context

- The robot uses a scoop end-effector for moving balls.
- Collision avoidance is required for robot self-collision and fixed
  environment collision.
- The balls themselves are not currently considered hard collision objects.
- The environment is expected to include narrow spaces, edges, corners, and
  surfaces that the robot may need to move along.
- After gameplay the robot should be able to back out and reset safely.

## 4. Main Technical Question

Can a simple analytic collision representation plus a lightweight real-time
constraint method provide good enough behavior for this application, without
needing full mesh-based collision or heavyweight motion-planning stacks?

## 5. Expected Query Outputs

The collision system probably needs more than:

- `is_colliding = true/false`

Likely useful outputs include:

- signed distance or at least a monotonic separation metric
- penetration depth
- contact normal or push-out direction
- nearest point or witness points
- per-link or per-primitive contribution
- a way to estimate the effect of small motion in each joint direction
- a query result that can be turned into a gradient-like repulsive response

The new repository should explicitly test which of these are actually required.

## 6. Candidate Geometry Direction

The current design preference is to start from very simple analytic geometry,
not mesh collision.

Primary candidates:

- sphere vs sphere
- sphere vs oriented box
- sphere vs plane
- sphere aggregates attached to robot links
- environment represented by spheres, oriented boxes, and planes

Secondary candidates, only if necessary:

- capsules for elongated links
- axis-aligned boxes where orientation is not needed
- convex analytic primitives if they add major value without major complexity

The default bias is:

- prefer simpler queries
- prefer primitives with cheap distance queries
- prefer representations that are easy to hand-tune and inspect

## 7. Desired Behavior

The finished system should ideally support all of the following:

- stop motion into hard constraints
- allow motion that increases clearance
- allow sliding tangentially along surfaces
- handle multiple simultaneous axis inputs without jitter or unsafe tunneling
- remain controllable in corners and narrow passages
- let the user back out of bad configurations
- behave consistently when the user rapidly changes direction
- avoid large discontinuities in output when entering or leaving constraint zones

## 8. Non-Goals For The First Stage

The first stage of the new repository should not assume it must deliver:

- a production-ready full robot integration
- perfect physical realism
- full mesh-based collision
- a polished UI
- museum-ready visuals

The first stage should answer feasibility and design questions quickly.

## 9. Integration Target Back Into The Main Repo

The expected production integration point is a module that can consume:

- current joint state
- desired post-rate-limit joint target
- robot/environment model

And produce:

- collision-filtered joint target
- optional diagnostics for UI/debugging

Conceptually:

```text
raw dial
-> gearing
-> static joint clamp
-> rate limit
-> collision-aware planner/filter
-> robot command
```

The planner/filter should eventually be able to replace:

- `planned_deg = throttled_deg`

with something collision-aware.

## 10. Initial Deliverables For The New Repo

The first useful deliverables should be:

1. A benchmark harness for analytic primitive queries.
2. A scene format for robot and environment primitives.
3. A set of reproducible corner-case scenarios.
4. One or more candidate collision-response methods.
5. A short report on what is fast enough and what feels controllable.
6. A recommendation for the geometry set and query API to carry forward.

## 11. Key Open Questions

- Are spheres alone enough for both robot and environment?
- Do oriented boxes provide enough value to justify their extra complexity?
- Is a signed-distance style response enough, or is a more explicit constrained
  solver needed?
- Should the planner operate directly in joint space, Cartesian proxy space, or
  both?
- How much parallelization is actually needed after basic analytic optimization?
- What level of gradient quality is required for stable multi-axis behavior?

