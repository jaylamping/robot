/// Tests for sweep_pass / sweep_home coordinate-frame math.
///
/// These tests simulate the step loop that `sweep_pass` and `sweep_home` run,
/// replacing the actual CAN motor with a fake that echoes the commanded position
/// as its feedback.  This validates:
///
///   1. Convergence — the loop reaches each target within tolerance.
///   2. Correct-frame math — even when the raw encoder starts on a different 2π
///      branch from the joint config limits.
///   3. Step size — each commanded position changes by at most `step_rad`.
///   4. Arrival detection — the loop exits when within 0.02 rad of the target.
///   5. Cancellation — the CancellationToken stops the loop mid-pass.
///   6. `sweep_home` — same convergence guarantee for the return-to-home segment.

use std::f32::consts::TAU;
use tokio_util::sync::CancellationToken;
use cortex::safety::{canonical_joint_angle, motor_cmd_for_joint_target};

const ARRIVAL_TOL: f32 = 0.02;

/// Simulate one leg of the sweep loop (current raw position → joint-frame target).
/// Returns the sequence of raw positions the "motor" reports after each command.
/// Mirrors the loop body in `sweep_pass` / `sweep_home`.
fn simulate_sweep_leg(
    mut raw: f32,
    target: f32,
    min: f32,
    max: f32,
    step_rad: f32,
    cancel: &CancellationToken,
) -> (Vec<f32>, bool) {
    let limits = (min, max);
    let mut positions = vec![raw];

    loop {
        if cancel.is_cancelled() {
            return (positions, true);
        }
        let canonical = canonical_joint_angle(raw, target, min, max);
        let err = target - canonical;
        if err.abs() < ARRIVAL_TOL {
            break;
        }
        let step = err.clamp(-step_rad, step_rad);
        let cmd = motor_cmd_for_joint_target(raw, canonical + step, limits);
        // Fake motor: feedback raw position equals the commanded raw position.
        raw = cmd;
        positions.push(raw);
    }

    (positions, false)
}

// ---------------------------------------------------------------------------
// Basic convergence — normal branch, sweep to min then max
// ---------------------------------------------------------------------------

#[test]
fn sweep_converges_to_min_from_mid() {
    let (min, max, home) = (-1.57_f32, 1.57_f32, 0.0_f32);
    let start_raw = home; // raw == joint frame (no branch offset)
    let step_rad = 0.1_f32;
    let cancel = CancellationToken::new();

    let (positions, cancelled) = simulate_sweep_leg(start_raw, min, min, max, step_rad, &cancel);

    assert!(!cancelled, "should not be cancelled");
    let final_canonical = canonical_joint_angle(*positions.last().unwrap(), min, min, max);
    assert!(
        (final_canonical - min).abs() < ARRIVAL_TOL,
        "expected to arrive at min={min:.3}, final canonical={final_canonical:.3}"
    );
}

#[test]
fn sweep_converges_to_max_from_min() {
    let (min, max) = (-1.57_f32, 1.57_f32);
    let start_raw = min;
    let step_rad = 0.1_f32;
    let cancel = CancellationToken::new();

    let (positions, cancelled) = simulate_sweep_leg(start_raw, max, min, max, step_rad, &cancel);

    assert!(!cancelled);
    let final_canonical = canonical_joint_angle(*positions.last().unwrap(), max, min, max);
    assert!(
        (final_canonical - max).abs() < ARRIVAL_TOL,
        "expected max={max:.3}, got canonical={final_canonical:.3}"
    );
}

// ---------------------------------------------------------------------------
// Branch-mismatch: raw encoder starts on a different 2π branch
// ---------------------------------------------------------------------------

#[test]
fn sweep_converges_when_raw_on_wrong_branch_positive() {
    // Physical position is near min (-1.57), but raw encoder reports it as
    // one full turn higher: -1.57 + 2π ≈ 4.71.
    let (min, max) = (-1.57_f32, 1.57_f32);
    let physical_at_home = 0.0_f32;
    let raw_start = physical_at_home + TAU; // encoder on +1 turn branch
    let step_rad = 0.1_f32;
    let cancel = CancellationToken::new();

    let (positions, cancelled) = simulate_sweep_leg(raw_start, min, min, max, step_rad, &cancel);

    assert!(!cancelled);
    let final_raw = *positions.last().unwrap();
    let final_canonical = canonical_joint_angle(final_raw, min, min, max);
    assert!(
        (final_canonical - min).abs() < ARRIVAL_TOL,
        "branch-offset: expected canonical ~{min:.3}, got {final_canonical:.3} (raw={final_raw:.3})"
    );
}

#[test]
fn sweep_converges_when_raw_on_wrong_branch_negative() {
    // Raw encoder starts one full turn below the joint frame.
    let (min, max) = (-1.57_f32, 1.57_f32);
    let physical_at_home = 0.0_f32;
    let raw_start = physical_at_home - TAU; // encoder on −1 turn branch
    let step_rad = 0.1_f32;
    let cancel = CancellationToken::new();

    let (positions, cancelled) = simulate_sweep_leg(raw_start, max, min, max, step_rad, &cancel);

    assert!(!cancelled);
    let final_raw = *positions.last().unwrap();
    let final_canonical = canonical_joint_angle(final_raw, max, min, max);
    assert!(
        (final_canonical - max).abs() < ARRIVAL_TOL,
        "branch-offset: expected canonical ~{max:.3}, got {final_canonical:.3} (raw={final_raw:.3})"
    );
}

// ---------------------------------------------------------------------------
// Step size never exceeds step_rad
// ---------------------------------------------------------------------------

#[test]
fn sweep_steps_never_exceed_step_rad() {
    let (min, max) = (-1.57_f32, 1.57_f32);
    let step_rad = 0.087_f32; // ~5°
    let cancel = CancellationToken::new();

    let (positions, _) = simulate_sweep_leg(0.0, min, min, max, step_rad, &cancel);

    for window in positions.windows(2) {
        let delta = (window[1] - window[0]).abs();
        assert!(
            delta <= step_rad + 1e-4,
            "step delta {delta:.4} exceeded step_rad {step_rad:.4}"
        );
    }
}

// ---------------------------------------------------------------------------
// Already at target → exits immediately (no steps taken)
// ---------------------------------------------------------------------------

#[test]
fn sweep_already_at_target_no_steps() {
    let (min, max) = (-1.57_f32, 1.57_f32);
    // Start exactly at min — should exit without any motor commands.
    let cancel = CancellationToken::new();
    let (positions, cancelled) = simulate_sweep_leg(min, min, min, max, 0.1, &cancel);

    assert!(!cancelled);
    // Only the initial position, no additional steps.
    assert_eq!(positions.len(), 1, "should not step when already at target");
}

// ---------------------------------------------------------------------------
// Cancellation stops the loop
// ---------------------------------------------------------------------------

#[test]
fn sweep_cancellation_stops_loop() {
    let (min, max) = (-1.57_f32, 1.57_f32);
    let cancel = CancellationToken::new();
    // Cancel immediately before the first iteration.
    cancel.cancel();

    let (positions, cancelled) = simulate_sweep_leg(0.0, min, min, max, 0.1, &cancel);

    assert!(cancelled, "should report cancellation");
    // No steps should have been taken.
    assert_eq!(positions.len(), 1, "no steps expected after immediate cancel");
}

#[test]
fn sweep_cancellation_mid_pass() {
    // Cancel after a fixed number of steps by counting iterations manually.
    let (min, max) = (-1.57_f32, 1.57_f32);
    let step_rad = 0.1_f32;
    let limits = (min, max);
    let cancel = CancellationToken::new();
    let cancel_after = 3usize;

    let mut raw = 0.0_f32;
    let target = min;
    let mut steps = 0;
    let mut was_cancelled = false;

    loop {
        if steps >= cancel_after {
            cancel.cancel();
        }
        if cancel.is_cancelled() {
            was_cancelled = true;
            break;
        }
        let canonical = canonical_joint_angle(raw, target, min, max);
        let err = target - canonical;
        if err.abs() < ARRIVAL_TOL {
            break;
        }
        let step = err.clamp(-step_rad, step_rad);
        let cmd = motor_cmd_for_joint_target(raw, canonical + step, limits);
        raw = cmd;
        steps += 1;
    }

    assert!(was_cancelled, "loop should have been cancelled");
    assert!(steps <= cancel_after, "should stop at or before cancel_after steps");
}

// ---------------------------------------------------------------------------
// Full pass: min → max both converge (simulating a complete sweep_pass call)
// ---------------------------------------------------------------------------

#[test]
fn full_sweep_pass_min_then_max() {
    let (min, max) = (-1.2_f32, 1.2_f32);
    let step_rad = 0.15_f32;
    let cancel = CancellationToken::new();

    // Leg 1: home (0.0) → min
    let (_, cancelled1) = simulate_sweep_leg(0.0, min, min, max, step_rad, &cancel);
    assert!(!cancelled1);

    // Leg 2: min → max (starting from raw ≈ min after leg 1)
    let (_, cancelled2) = simulate_sweep_leg(min, max, min, max, step_rad, &cancel);
    assert!(!cancelled2);
}

// ---------------------------------------------------------------------------
// sweep_home math: converges from max back to home
// ---------------------------------------------------------------------------

#[test]
fn sweep_home_converges_from_max() {
    let (min, max, home) = (-1.57_f32, 1.57_f32, 0.0_f32);
    let step_rad = 0.1_f32;
    let cancel = CancellationToken::new();

    let (positions, cancelled) = simulate_sweep_leg(max, home, min, max, step_rad, &cancel);

    assert!(!cancelled);
    let final_canonical = canonical_joint_angle(*positions.last().unwrap(), home, min, max);
    assert!(
        (final_canonical - home).abs() < ARRIVAL_TOL,
        "sweep_home: expected ~{home:.3}, got {final_canonical:.3}"
    );
}

#[test]
fn sweep_home_converges_from_wrong_branch() {
    // Motor at home physically but raw encoder is one turn off.
    let (min, max, home) = (-1.57_f32, 1.57_f32, 0.0_f32);
    let raw_start = max + TAU; // one full turn above max
    let step_rad = 0.1_f32;
    let cancel = CancellationToken::new();

    let (positions, cancelled) = simulate_sweep_leg(raw_start, home, min, max, step_rad, &cancel);

    assert!(!cancelled);
    let final_raw = *positions.last().unwrap();
    let final_canonical = canonical_joint_angle(final_raw, home, min, max);
    assert!(
        (final_canonical - home).abs() < ARRIVAL_TOL,
        "branch-offset home: expected ~{home:.3}, got {final_canonical:.3}"
    );
}

// ---------------------------------------------------------------------------
// Asymmetric limits (realistic shoulder pitch: -0.5 to 2.7 rad, home at 0.3)
// ---------------------------------------------------------------------------

#[test]
fn sweep_asymmetric_limits() {
    let (min, max, home) = (-0.5_f32, 2.7_f32, 0.3_f32);
    let step_rad = 0.1_f32;
    let cancel = CancellationToken::new();

    // Start at home, sweep to min.
    let (p1, c1) = simulate_sweep_leg(home, min, min, max, step_rad, &cancel);
    assert!(!c1);
    let at_min = canonical_joint_angle(*p1.last().unwrap(), min, min, max);
    assert!((at_min - min).abs() < ARRIVAL_TOL, "at_min={at_min:.3}");

    // Sweep min → max.
    let (p2, c2) = simulate_sweep_leg(*p1.last().unwrap(), max, min, max, step_rad, &cancel);
    assert!(!c2);
    let at_max = canonical_joint_angle(*p2.last().unwrap(), max, min, max);
    assert!((at_max - max).abs() < ARRIVAL_TOL, "at_max={at_max:.3}");

    // Return home.
    let (p3, c3) = simulate_sweep_leg(*p2.last().unwrap(), home, min, max, step_rad, &cancel);
    assert!(!c3);
    let at_home = canonical_joint_angle(*p3.last().unwrap(), home, min, max);
    assert!((at_home - home).abs() < ARRIVAL_TOL, "at_home={at_home:.3}");
}
