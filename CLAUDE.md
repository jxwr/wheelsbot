As the AI development agent for this project, you must strictly adhere to the following workflows and standards to ensure safe, consistent, and maintainable modifications in this large-scale codebase.

Whenever you complete a task, analyze if there's any repetitive pattern you've noticed and suggest an update to Section 4 of CLAUDE.md to automate this knowledge for future sessions.

Read MEM.md before you start.

---

## 1. Interaction Protocol: The EPC Cycle
Before executing any non-trivial code modifications, you must follow the **Explore-Plan-Code-Verify** loop:

1.  **Explore**:
    * Use `grep`, `ls`, or `find` to locate relevant logic and definitions.
    * Analyze cross-file dependencies (e.g., how an API change affects consumers).
    * **Do not** guess logic; verify the file structure and existing patterns first.
2.  **Plan**:
    * Output a concise summary of your proposed changes in the terminal.
    * Include: Files to be modified, core logic shifts, and potential side effects.
    * **Wait for user confirmation** before proceeding to code generation.
3.  **Code**:
    * Implement changes based on the approved plan.
    * Maintain the project's existing coding style (naming conventions, indentation, patterns).
4.  **Verify**:
    * Proactively run relevant tests.
    * Check for Lint errors. If any are introduced, fix them immediately.

---

## 2. Context Management
* **Source of Truth**: Always prioritize information in `README.md`, `ARCHITECTURE.md`, and this `CLAUDE.md`.
* **Pattern Discovery**: If you identify "hidden" project conventions (e.g., specific error handling patterns), point them out and ask if they should be added to this guide.
* **Precise Referencing**: Use `@filename` or specific line ranges when discussing code to ensure clarity.

---

## 3. Git Workflow & Commit Standards
We prioritize **Atomic Commits** to keep the project history clean and revertable.

* **Branching Strategy**:
    * Always work on a feature branch. Never modify `main` or `master` directly.
* **Commit Frequency**:
    * Commit after every "atomic" unit of work (e.g., refactoring a single function, adding a type definition, or fixing one bug).
* **Commit Messages**:
    * Follow the [Conventional Commits](https://www.conventionalcommits.org/) specification (e.g., `feat:`, `fix:`, `refactor:`, `chore:`).
    * *Example:* `feat(auth): add role-based access control to user service`
* **Automation**:
    * After verification, ask: "The changes are verified. Should I create a git commit for you?"

---

## 4. Technical Standards (Project Specific)
* **Core Stack**: ESP32-S3, Arduino Framework, SimpleFOC, FreeRTOS, PlatformIO
* **Coding Style**:
    * C++17 with embedded constraints (no STL containers, no exceptions)
    * Static allocation only - no `new`/`malloc` in runtime hot paths
    * Namespaces: `wheelsbot::hardware`, `wheelsbot::control`
    * Class naming: `PascalCase` for classes, `snake_case` for files
    * Member variables: trailing underscore for private members (`foo_`)
* **Error Handling**:
    * Fault flags pattern (`CASCADE_FAULT_*`, `BC_FAULT_*`)
    * State machine approach (`STATE_DISABLED`, `STATE_RUNNING`, `STATE_FAULT`)
    * Sensor timeout detection with configurable thresholds
    * All control outputs include `valid` boolean flag
* **Documentation**: All control interfaces must have class-level comments describing input/output units (rad, rad/s, V, etc.)

---

## 5. Safety Guardrails
* **No Mass Deletions**: Never delete large directories or multiple files without explicit, double-confirmed user authorization.
* **Scope Locking**: Do not rewrite files unrelated to the current task (preventing "context drift").
* **Uncertainty**: If the impact of a change is unclear, run analysis tools first and ask for clarification.

## 6. Advanced Search & Retrieval (Precision First)
* **Avoid "Cat All"**: Do not read large files entirely if only a specific function is needed. Use `sed` or `grep` to extract relevant lines first.
* **Ripgrep Usage**: When searching for patterns, use `rg` with flags like `--type` (e.g., `rg --type ts "interface User"`) to narrow down results.
* **Dependency Mapping**: If modifying a shared component/utility, always search for all import references to assess the "blast radius" of your change.

## 7. Minimalist Editing & Diff Hygiene
* **No Stealth Reformatting**: Do not change whitespace, quotes, or reorder imports unless specifically asked or if it violates the project's Prettier/Lint config. Keep the Git diff focused strictly on the logic change.
* **Partial Writes**: For massive files (>1000 lines), prefer editing specific blocks rather than rewriting the whole file to prevent truncation or context loss.
* **Comment Preservation**: Do not remove existing JSDoc or TODO comments unless they are explicitly rendered obsolete by your changes.

## 8. Environment Awareness & Verification
* **Build Before Test**: In compiled languages (TS, Go, Rust), always run the build command (e.g., `npm run build`) before running tests to catch type-level errors that tests might miss.
* **Log Cleanup**: If you added `console.log` or debug prints during the **Code** phase, you must remove them during the **Verify** phase before committing.
* **Continuous Linting**: Run the linter (e.g., `eslint --fix`) as the final step of the **Code** phase to ensure the PR will pass CI.

## 9. Decision Making & Ambiguity
* **The "Choose Two" Rule**: If a task can be implemented in multiple ways (e.g., performance vs. readability), stop and ask the user for a preference instead of assuming.
* **Edge Case Probing**: When planning, proactively suggest 1-2 edge cases you intend to handle (e.g., "I will also handle the null case for the API response").

## 10. Documentation Debt
* **Auto-Update Docs**: If a logic change alters the behavior of an API or a CLI tool, you are responsible for updating the corresponding `.md` documentation or help text in the same commit.


# About the Project

## 1. Project Goal

This project aims to gradually evolve a two‑wheel self‑balancing robot into a modular, extensible robotics control framework. The framework must support multiple hardware configurations, interchangeable sensors, different motor drivers, and layered control architectures while remaining suitable for real‑time embedded deployment.

Claude Code will be used iteratively to:

* Improve the control architecture
* Refactor code into reusable modules
* Introduce plugin‑based hardware abstraction
* Maintain stability and real‑time performance
* Enable experimentation with advanced controllers and AI integration

The system must always remain runnable after each iteration.

---

## 2. Core Architectural Principles

### 2.1 Layered Control Stack

The software must follow a strict layered structure:

1. Hardware Abstraction Layer (HAL)
2. Sensor Fusion / State Estimation
3. Cascade Control Framework
4. Safety and State Machine
5. High‑level behaviors / external command interface

Each layer must only depend on the layer directly below it.

---

### 2.2 Cascade Control Philosophy

The control system must be implemented as a cascade of control loops where each loop regulates a physical state variable.

Typical cascade chain:

Position Loop → Velocity Loop → Angle Loop → Torque / Current Loop

Rules:

* Each loop outputs the reference input of the next loop
* Loop bandwidth must be separated (inner loops significantly faster)
* Loops must support saturation and anti‑windup
* Loops must be independently testable

---

### 2.3 Plugin‑Based Hardware Support

The framework must allow runtime or compile‑time selection of:

* IMU types (e.g., MPU6050, ICM42688, BNO085)
* Encoder types (magnetic, quadrature, SPI absolute)
* Motor drivers (FOC driver, voltage driver, smart servo)

All hardware must implement standardized interfaces.

Example interface categories:

* IMUSensor
* WheelEncoder
* MotorDriver
* PowerMonitor

New hardware must be addable without modifying control logic.

---

## 3. Control Framework Requirements

### 3.1 Generic ControlLoop Interface

All controllers must inherit from a unified control loop interface:

Required capabilities:

* step(reference, measurement, dt)
* reset()
* setLimits(min, max)
* runtime parameter update

Controller implementations may include:

* PID
* LQR (future)
* MPC (future)
* Learned controllers (future)

---

### 3.2 Cascade Controller Composition

The framework must allow multiple ControlLoop objects to be composed into a cascade controller pipeline.

Requirements:

* Loops are connected in sequence
* Each loop operates at its own frequency
* Scheduler must guarantee deterministic execution order
* Debug output for each stage must be available

---

## 4. State Estimation Requirements

The system must maintain a centralized robot state structure containing:

* orientation (pitch, roll, yaw)
* angular velocity
* wheel velocity
* robot velocity
* position estimate (optional)

Sensor fusion must be replaceable without affecting the controller API.

---

## 5. Safety and Reliability

The framework must include a safety state machine capable of handling:

* fall detection
* motor disable conditions
* sensor failure
* voltage protection
* watchdog recovery

Control loops must never directly bypass safety constraints.

---

## 6. Real‑Time Execution Model

The framework must support deterministic scheduling of multiple control tasks:

Typical task frequency targets:

* Motor / torque loop: highest frequency
* Balance / angle loop: high frequency
* Velocity loop: medium frequency
* Position loop: low frequency
* Telemetry / logging: asynchronous

The scheduling mechanism must allow adding new control loops without redesigning the runtime.

---

## 7. Extensibility Goals

Future framework extensions must be supported without redesigning the core:

* Reinforcement learning policy output as high‑level command
* Vision‑based control modules
* Multi‑actuator robots (wheel‑leg, humanoid joints)
* Distributed multi‑controller systems

The cascade control layer must remain the stable execution core.

---

## 8. Development Strategy with Claude Code

Claude Code should evolve the system gradually using the following approach:

1. Preserve working functionality at every step
2. Introduce abstraction layers incrementally
3. Avoid premature generalization
4. Refactor toward modular plugin‑based hardware interfaces
5. Add control loop abstraction before expanding algorithms
6. Ensure safety constraints remain active during all refactors

Each iteration should:

* keep the robot operational
* reduce coupling
* increase modularity
* improve observability and debugging support

---

## 9. Long‑Term Vision

The final system should resemble a miniature robotics control platform featuring:

* hardware abstraction
* state estimation pipeline
* modular cascade control architecture
* real‑time scheduling
* safety management
* high‑level AI or planning integration

The balance robot will serve as the initial reference platform, but the framework should be reusable for future robotic systems.
