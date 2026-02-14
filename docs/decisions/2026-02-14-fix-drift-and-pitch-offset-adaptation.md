## Decision: Fix Robot Drift and Pitch Offset Self-Adaptation

Date: 2026-02-14
Author: Claude Code

### Context

Two critical issues were discovered in the BalanceController after the cascade-to-balance refactor:

1. **Robot drifting (溜车)**: The robot would continuously drift forward or backward even when commanded to stay still.

2. **Pitch offset self-adaptation working incorrectly**: The CoG (center of gravity) self-adaptation logic had inverted sign, causing it to amplify drift instead of correcting it.

**Root Cause Analysis:**

**Issue 1 - Drift:**
- `speed_kp = 0.02f` was 35× smaller than the wheel-leg robot's value (0.7)
- Speed tracking was too weak to counteract drift
- Example: 10 rad/s velocity error → only 0.2V output (insufficient)

**Issue 2 - Pitch Offset Sign:**
```cpp
// Old (WRONG):
params_.pitch_offset -= pid_zeropoint_(lpf_zeropoint_(distance_ctrl));

// Logic error:
// Forward drift → distance_ctrl > 0
// pitch_offset decreased → robot leans forward MORE
// Amplifies drift instead of correcting it!
```

**Issue 3 - Overly Aggressive Zero-Point Reset:**
- `if (fabsf(speed) > 15.0f)` (0.525 m/s) triggered too easily
- Distance control disabled during normal movement
- Robot would stop far from original position

**Issue 4 - Adaptation Condition Too Strict:**
- `fabsf(distance_ctrl) < 4.0f` prevented adaptation when drift was significant
- Should activate when there's persistent small drift, not when error is tiny

### Decision

Applied four fixes to `src/control/balance_controller.h`:

**Fix 1: Increase speed loop gain (primary fix for drift)**
```cpp
// Line 128
float speed_kp = 0.02f;  // OLD
float speed_kp = 0.5f;   // NEW (25× increase)
```

**Fix 2: Raise fast-reset velocity threshold**
```cpp
// Line 255
if (fabsf(speed) > 15.0f) {  // OLD: 0.525 m/s
if (fabsf(speed) > 30.0f) {  // NEW: 1.05 m/s
```

**Fix 3: Correct pitch_offset adaptation sign**
```cpp
// Line 291
params_.pitch_offset -= pid_zeropoint_(...);  // OLD (WRONG)
params_.pitch_offset += pid_zeropoint_(...);  // NEW (CORRECT)

// Correct logic:
// Forward drift (distance_ctrl > 0) → pitch_offset increases → robot leans back → corrects drift ✓
```

**Fix 4: Relax adaptation activation condition**
```cpp
// Line 287 (OLD)
if (fabsf(lqr_u) < 5.0f && !has_joystick_input
    && fabsf(distance_ctrl) < 4.0f && !wheel_lifted)

// Line 287 (NEW)
if (fabsf(lqr_u) < 5.0f && !has_joystick_input
    && fabsf(speed) < 1.0f && !wheel_lifted)

// Changed condition from "distance error small" to "robot nearly stationary"
// Allows adaptation when there's persistent drift at low speed
```

### Alternatives Considered

**Alternative 1: Only fix speed_kp, keep rest unchanged**
- Rejected: Would fix drift but leave pitch offset adaptation broken

**Alternative 2: Disable pitch_offset adaptation entirely**
- Rejected: Adaptation is useful for compensating battery position changes, payload variations, etc.

**Alternative 3: Use different speed_kp value (0.3 or 0.7)**
- Deferred: Start with 0.5 (middle ground), can tune on hardware

**Alternative 4: Remove fast-reset logic entirely**
- Rejected: Still useful for detecting hard manual pushes / external disturbances

### Impact

**Files Modified:**
- `src/control/balance_controller.h` (4 changes)

**Behavioral Changes:**
1. **Drift correction**: 25× stronger velocity tracking should eliminate drift
2. **CoG adaptation**: Now works correctly (sign fixed), will slowly adjust to static weight imbalance
3. **Position hold**: More stable during normal motion (higher reset threshold)
4. **Adaptation trigger**: Activates when robot is nearly still with persistent drift (more logical)

**Parameter Changes:**
- Default `speed_kp`: 0.02 → 0.5 (users with saved params need to update manually or delete `/params/balance.json`)

**Breaking Changes:**
- None (parameter structure unchanged)

**Testing Required:**
- Verify drift is eliminated when robot commanded to stay still
- Check that velocity tracking responds appropriately to joystick commands
- Observe pitch_offset adaptation over ~30s of stationary operation
- Confirm fast-reset only triggers during hard pushes (>1 m/s)

### Follow-ups

- [ ] Test on hardware and fine-tune `speed_kp` (range 0.3-0.7)
- [ ] Monitor `pitch_offset` value in telemetry during operation
- [ ] Add telemetry field for `speed_ctrl` contribution to debug velocity loop
- [ ] Consider adding hysteresis to fast-reset logic (e.g., reset at 30 rad/s, re-enable at <25 rad/s)
- [ ] Document recommended parameter update procedure in user guide

---

**Related ADRs:**
- `2026-02-14-replace-cascade-with-balance-controller.md` - Initial refactor that introduced these issues

**References:**
- Original code review: `CODE_REVIEW_ISSUES.md` (issues #1, #2, #6, #7)
- Wheel-leg robot reference: Micro-Wheeled_leg-Robot `wl_pro_robot.ino`
