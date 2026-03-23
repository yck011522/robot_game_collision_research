# Collision Repo Handoff

This folder is a seed package for a new repository focused on collision
detection and collision-aware real-time jogging for the robot game.

The goal of the new repository is not to immediately solve the full production
problem. Its first job is to answer the research and engineering questions that
the main game repository should not have to carry:

- Can simple analytic collision geometry be fast enough for interactive control?
- What query outputs are needed beyond a boolean collision flag?
- Can the system provide stable behavior when multiple axes are being driven at
  once, including reversals and narrow-passage situations?
- What representation and algorithm are simple enough to ship and maintain?

Files:

- `PROJECT_BRIEF.md`
  - Fixed context from the current game
  - Decisions already made
  - Goals, non-goals, and integration target
- `RESEARCH_PLAN.md`
  - Proposed experiments and benchmark plan
  - Candidate geometry abstractions
  - Success criteria and deliverables
- `new_repo_root/AGENTS.md`
  - Copy to the root of the new repository
  - Human-readable instructions for coding agents
- `new_repo_root/ai_context.json`
  - Copy to the root of the new repository
  - Structured machine-readable context and fixed assumptions

Recommended use:

1. Copy this folder into the new repository.
2. Treat `PROJECT_BRIEF.md` as the source of truth for fixed assumptions.
3. Treat `RESEARCH_PLAN.md` as the initial backlog for prototypes and tests.
4. Copy `new_repo_root/AGENTS.md` into the new repo root.
5. Copy `new_repo_root/ai_context.json` into the new repo root.
6. Update the documents as soon as any fixed assumption changes.

Current source repository context:

- Main game loop target: 100 Hz
- Haptic command/telemetry loop: 50 Hz
- Simulated robot physics: 200 Hz
- Current control pipeline:
  `raw dial -> geared command -> clamp -> rate limit -> planned -> robot -> haptic feedback`
- Current planner slot:
  the future motion planner is expected to overwrite `planned_deg`, which is
  currently just a pass-through of the throttled target
