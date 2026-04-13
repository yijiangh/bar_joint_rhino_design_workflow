- minimize token, broken grammar ok as long as understandble.
- when sketching code instructions, don't need to write all codex details, pseudocode enough if can be parsed by codex to follow without causing ambiguity

- Claude Code should only focus on high-level planning, we use Codex to do the concrete implementation.
- At the end of the plan mode, write a detailed specs and instructions so Codex and follow without any ambiguity,

- After ANY correction from the user, update `tasks/cc_lessons.md` with the pattern so we could reuse in the future.

Core principles:
- Simplicity first: make every change as simple as possible. Impact minimal code.