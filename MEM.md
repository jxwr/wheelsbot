#  Operating Rules
This repository uses persistent decision tracking and project memory recording.
You must follow the rules below whenever working in this repository.

---

# 1. Persistent Decision Recording (ADR)

Whenever an important decision is made, You MUST create or update a file under:

docs/decisions/

Each decision must be stored as a separate file using the filename format:

YYYY-MM-DD-short-decision-name.md

Example:
docs/decisions/2026-02-12-balance-controller-structure.md

Each ADR file MUST follow this format:

## Decision: <Title>

Date: <YYYY-MM-DD>
Author: You Code

### Context
Describe the problem, background, or motivation.

### Decision
Describe the selected solution.

### Alternatives considered
List alternative approaches and why they were rejected.

### Impact
Describe modules, APIs, parameters, or workflows affected.

### Follow-ups
List tasks required after this decision.

---

# 2. Project Memory Recording

You MUST maintain a persistent project memory file:

docs/project_memory.md

Whenever the user mentions any of the following, You MUST append them:

- important constraints
- rules that must not be violated
- long-term architecture requirements
- important parameter meanings
- known technical debt
- important workflow agreements

Entries must include date and short description.

Example entry:

2026-02-12  
Balance loop must run at >= 200Hz to maintain stability.

---

# 3. Automatic Recording Requirement

After implementing any major change, You MUST:

1. Update or create the relevant ADR
2. Update project_memory.md if new long-term rules appear
3. Commit both the code change and documentation

Major change includes:

- architecture modifications
- control-loop structure changes
- parameter system redesign
- new subsystem introduction
- cross-module refactoring
- workflow or tooling decisions

---

# 4. Commit Documentation Rule

When creating commits related to significant changes, You SHOULD include:

Decision summary:
- short explanation of the decision
- reference to ADR file path

Example:

Decision summary:
Adopt 3-layer control loop for balancing system  
ADR: docs/decisions/2026-02-12-balance-controller-structure.md

---

# 5. Priority Rule

Persistent files (ADR and project_memory.md) are the authoritative memory of the project.
Chat conversation is NOT considered persistent memory.
All important decisions MUST be written into repository documentation.
