# Subagent Onboarding — Role-based Crew, Started Small

## What "subagent" means here (corrected)

You want a small team of specialized Claude agents — coder, planner, reviewer, etc. — that collaborate to solve harder tasks than I can comfortably handle in a single conversation thread.

Important constraint to lock in before designing: **in Claude Code, subagents do not chat directly with each other.** Each invocation is a one-shot:

1. I (the main thread, "orchestrator") spawn a subagent with a self-contained prompt.
2. The subagent has its own context window, runs its work to completion.
3. It returns ONE summary message to me.
4. I read it, decide what to do next, and possibly spawn another agent with the relevant pieces baked into the new prompt.

So the team's "conversation" is really *me routing artifacts* (a plan doc, a code diff, a review note) between specialised one-shots. This is still very powerful — each agent can be deeply specialised, can run in parallel, and stays out of our shared chat — but it shapes how we design roles.

## Why role-based custom subagents help

A custom subagent is just a Markdown file at `.claude/agents/<name>.md` with frontmatter (name, description, allowed tools, model) plus a body that is the system prompt for that agent. Two reasons to author them:

1. **Primed context.** The system prompt encodes our conventions (`tasks/yh_lesson.md`, `tasks/cc_lessons.md`, `# r:` pinning rules, capture/replay workflow, no-purge policy, etc.) so I don't have to re-explain on every call.
2. **Tool sandboxing.** A reviewer doesn't get `Edit` / `Write` — it physically can't change the code, only read it. A planner doesn't get `Bash` — it can't run tests, only design. This both safeguards intent and makes each agent's behaviour predictable.

## Phase 1 — Start with three roles

Map directly onto the dev loop we've been running together:

```
   ┌────────────┐      ┌────────────────┐      ┌─────────────┐
   │  planner   │ ───▶ │   implementer  │ ───▶ │   reviewer  │
   └────────────┘      └────────────────┘      └─────────────┘
        ▲                                              │
        └──────────── (orchestrator routes) ───────────┘
```

| Role | Purpose | Tools | When I'd spawn it |
|---|---|---|---|
| **planner** | Reads relevant code + lessons; produces a step-by-step plan with file paths and edit sketches. No code changes. | Read, Glob, Grep, WebFetch (no Edit/Write/Bash) | "Design how X should change", multi-file refactors, new features. Output: a plan I can hand to the implementer. |
| **implementer** | Takes a plan + context, writes the code. Knows our conventions (yh_lesson, cc_lessons, `# r:` pinning). | Read, Edit, Write, Bash, Glob, Grep | "Apply the plan", "fix this bug given file X line Y". Output: a diff. |
| **reviewer** | Reads the diff and any test output, flags concrete issues against our conventions. Read-only. | Read, Glob, Grep, Bash | "Review this change before I commit." Output: a list of concrete issues or LGTM. |

Why these three first: they cover 90% of how we already work. Plan → Implement → Review is a one-line loop that catches most mistakes I make on my own.

## Where the role files live

```
.claude/
  agents/
    planner.md
    implementer.md
    reviewer.md
```

Frontmatter format (each file is ~30-60 lines):

```markdown
---
name: planner
description: Designs implementation plans for this project before any code is written. Use proactively for non-trivial multi-file changes.
tools: Read, Glob, Grep, WebFetch
model: sonnet
---

You are the planning agent for the bar-joint Rhino design workflow.

Project conventions you MUST internalise before producing a plan:
- Read `CLAUDE.md`, `tasks/cc_lessons.md`, `tasks/yh_lesson.md` first.
- All `# r:` requirements in `scripts/rs_*.py` are pinned to exact versions.
- compas_fab is loaded from `external/compas_fab` via sys.path; do NOT propose `# r: compas_fab`.
- ...

When asked to plan a change:
1. Identify the files involved with file:line references.
2. Propose minimal edits (do not rewrite working code).
3. Call out reuse of existing helpers (e.g. `core.transforms`, `core.rhino_helpers`, `core.rhino_bar_registry`).
4. Output a numbered plan with explicit file paths.

Never write code. Never run shell commands.
```

(Implementer and reviewer follow the same shape with appropriately scoped tools and prompts.)

## How a typical task flows

User: "Add a 'rotation' option to RSJointPlace that lets the user nudge the female by ±5° via arrow keys."

I would:

1. **Spawn planner** with: the user request + a hint to read `scripts/rs_joint_place.py` and `tasks/cc_lessons.md`. Wait for the plan.
2. **Read the plan**, ask user to confirm if non-trivial. (Subagents cannot pause for user input — only I can.)
3. **Spawn implementer** with: the plan + relevant file paths. It writes the diff in foreground.
4. **Spawn reviewer** with: the diff (`git diff`), pointed at conventions. Wait for findings.
5. If reviewer flags issues, **re-spawn implementer** with reviewer notes. Loop until reviewer says LGTM (max 2 iterations to avoid infinite back-and-forth).
6. Hand the result back to the user for the actual click-test.

For trivial tasks (typo fix, single-line tweak), I skip the agents entirely. The orchestration overhead exceeds the benefit below ~3 file edits.

## Phase 2 (~1-2 weeks in) — Specialise the implementer

The single fork most likely to pay off, based on our recent dev:

| New role | Why split | Phase-1 fallback |
|---|---|---|
| **rhino-implementer** | Knows Rhino API quirks (`Constrain(Plane, bool)` vs `Constrain(Brep, ...)`, extrusion-to-brep, `BlockInstance.InstanceXform`), `# venv` site-env semantics, capture-mode hooks. | implementer |
| **headless-implementer** | Pure-Python: `core/*.py`, `tests/replay_ik*.py`, pytest setup. Free to call `python -m pytest tests/` to self-validate before reporting back. | implementer |

Add this only if the generic implementer starts confusing the two contexts (e.g. proposing `rs.GetUserText` in a pytest file).

## Phase 3 — Add a debugger only if the same diagnosis repeats

A **debugger** subagent specialised on:
- Reading `tests/captures/*.json`
- Running `tests/replay_ik.py <file>`
- Mapping `[X]` failures to known categories from `tasks/yh_lesson.md` (mm/m unit, OCF→tool0 dispatch, base-frame composition, approach offset sign, IK solver itself)
- Returning a one-line diagnosis instead of a wall of solver output

Build this when we hit our third "Claude please debug this capture" moment, not before.

## Anti-patterns to avoid

- **A team of 6 from day one.** Each role is upkeep (definition file, prompt drift, deciding when to use which). Three is genuinely the right starting size.
- **Expecting agents to chat.** They can't. The orchestrator (me) is the message bus. If you find yourself wanting two agents to "discuss," that's a signal to put their shared knowledge in a file (a plan doc, a lesson) instead.
- **Specialising before pain.** rhino-implementer vs headless-implementer is a real split, but only after the generic one stumbles. Roles created for hypothetical future work go stale.
- **Putting our shared todo list inside an agent prompt.** Subagents get a fresh context — they don't see our open items. Pass relevant pieces explicitly each call.

## Alternative orchestration: a "team-lead" sub-agent

Some users build a coordinator sub-agent whose only job is to spawn other subagents. We don't need this — the main thread (me) is already the coordinator and can see the full conversation. Skip.

## What success looks like in two weeks

- We've used the planner→implementer→reviewer chain for 5+ non-trivial tasks.
- The reviewer has flagged at least one bug before it landed (proves the chain has value).
- We have NOT added a fourth role just to feel busy.
- Our chat history feels lighter — search/exploration noise has migrated to subagent runs.

## Concrete first day

1. I will draft `.claude/agents/planner.md`, `implementer.md`, `reviewer.md` based on our project conventions. ~10 minutes.
2. Pick one open todo from `tasks/test_pipeline_plan.md` Phase A1 (extracting `_translate_frame` etc. into `core/ik_workflow_math.py` plus its pytest).
3. Use it as the first end-to-end test of the chain: planner produces plan, implementer writes diff, reviewer checks against the plan and conventions.
4. Reflect: where did the chain feel right vs friction? Tune the prompts.

Want me to draft the three agent files now? They land at `.claude/agents/{planner,implementer,reviewer}.md` and we can iterate on their prompts as we use them.
