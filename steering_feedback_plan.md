# Steering feedback + road/obstacle arbitration plan

Competition in 2 days. Goal: closed-loop steering control (lidar sees the front
wheels/steering linkage) layered under the existing obstacle-avoidance state
machine, made road-aware via `road_follower`. Full state-machine removal is an
optional stretch phase, not required.

Steering signal convention throughout: normalized **-1..1** (matches existing
`/steering_motor_cmd`/`/traction_motor_cmd` convention), `+1.0` = full left.

## Status

- [x] Phase A — steering angle sensor (`steering_sensor` package, estimator node) — **calibrated and verified on hardware**
- [x] Phase B — closed-loop steering controller (same package, controller node) — **verified on hardware: converges to +1/-1 targets and holds, recenters on watchdog timeout**
- [x] Phase C — road+obstacle arbitration (`obstacle_avoidance` diff) — implemented, **not yet field tested**
- [x] Wiring — `robot_bringup` launch updates — implemented, **not yet run end-to-end**
- [ ] Phase D — stretch: remove obstacle_avoidance state machine (optional, only if time remains)
- [x] Start/Pause GPIO control (`run_state_node`, GPIO17/27) — implemented and **verified on hardware**, see dedicated section below

## TODO next session

- **Add an error deadzone to `steering_controller_node.py`.** Observed on
  hardware: lidar noise on `/steering_angle` (~1-1.5deg raw, amplified by the
  narrow ~13deg calibration span) causes the P controller to constantly
  hunt/dither the steering motor back and forth trying to null out a tiny
  error, even when the wheel is already "straight enough". Risk of motor
  wear/overheating from constant direction reversals. Fix: add a
  `error_deadzone` param (e.g. ~0.03-0.05 normalized) — if
  `abs(target - measured) < error_deadzone`, output 0 instead of a small
  correction. Tune the deadzone value on hardware (small enough to still
  reach the road-following/avoidance targets, large enough to stop the
  jitter).

## Phase A — Steering angle sensor

New package `src/steering_sensor/`, node `steering_estimator_node.py`:
- Subscribes `/scan_c1` (LaserScan, best-effort QoS).
- Applies the same angle correction as `obstacle_avoidance/avoider_node.py`
  (`corrected = atan2(sin(-raw+pi), cos(-raw+pi))`).
- Isolates a near-range angular/distance window where the wheel/steering
  linkage blob appears (params: `blob_angle_min_deg`, `blob_angle_max_deg`,
  `blob_range_min_m`, `blob_range_max_m`).
- Computes the mean angle of points in that window each scan.
- Maps that raw angle to normalized -1..1 via 3-point calibration
  (`calib_left_angle_deg`, `calib_center_angle_deg`, `calib_right_angle_deg`).
- Publishes `/steering_angle` (`std_msgs/Float32`, -1..1) and a debug raw angle
  topic to speed up calibration.

**Calibration procedure:** traction motor off, use `motors.md` commands to
drive `/steering_motor_cmd` to full left / center / full right, echo the debug
raw-angle topic at each position, fill the three `calib_*_angle_deg` params in
`config/params.yaml`.

## Phase B — Closed-loop steering controller

Same package, node `steering_controller_node.py`:
- Subscribes `/steering_angle` (measured) and `/steering_cmd` (target, -1..1).
- P controller: `output = clip(kp * (target - measured), -1, 1)`.
- Publishes `/steering_motor_cmd`.
- Watchdog: if `/steering_cmd` stale → target defaults to 0 (center). If
  `/steering_angle` (measured) stale → output forced to 0 (no open-loop guess).

## Phase C — Minimal road+obstacle arbitration

Edit `obstacle_avoidance/obstacle_avoidance/avoider_node.py`:
- Subscribe `/road_follower/error` (Float32, -1..1), with staleness guard
  (treat as 0 if not received within ~1s).
- `DRIVING` state: steer = `clip(road_error * road_steer_gain, -1, 1)` instead
  of hardcoded 0.0.
- `AVOIDING`/`STRAIGHTENING`: unchanged full-lock override (safety first).
- Steering output now publishes to `/steering_cmd` (consumed by Phase B
  controller) instead of directly to `/steering_motor_cmd`. Traction still
  publishes directly to `/traction_motor_cmd`, unchanged.
- Add `road_steer_gain` param to `config/params.yaml`.

## Wiring

- `robot_bringup/launch/robot_launch.py`: add `enable_steering_sensor`,
  `enable_road_follower`, `enable_obstacle_avoidance` launch args + includes.
- Repoint gamepad's `topic_steering` launch arg to `/steering_cmd` so manual
  driving also goes through the closed-loop controller.

## Phase D — Stretch (optional)

Replace obstacle_avoidance's discrete states with a continuous steering bias
+ traction scale from left/right sector clearance, blended with road error.
Keep the hard emergency-stop cutoff as-is. Only attempt if A–C are field
tested and working with time to spare.

## Verification checklist

1. Phase A: echo `/steering_angle` while commanding `/steering_motor_cmd` to
   full-left/center/full-right; confirm tracking, ~±1.0 at locks.
2. Phase B: traction off, publish test `/steering_cmd` values, confirm wheel
   converges and holds.
3. Phase C: road_follower + modified avoider on a road with grass edge, no
   obstacles, confirm centering; then introduce an obstacle.
4. Full stack via `robot_bringup` launch; check `ros2 topic list`/
   `ros2 node list` for no duplicate publishers on `/steering_motor_cmd`.
5. Reduced-speed field test in the park, gamepad deadman override ready as
   kill-switch.

## Decisions

- Traction stays open-loop; only steering gets closed-loop control.
- Normalized -1..1 convention, not radians.
- Phase D deprioritized/optional.
- Gamepad manual override remains available as fallback throughout.

## Notes / open risks

- If Phase A calibration proves noisy/unstable, fall back to open-loop
  steering (skip B/C) rather than risk an unreliable closed loop.
- Budget explicit on-site gain-tuning time on day 2, not just code completion.
- `road_follower` is now launched by default from `robot_bringup` — it calls
  `cv2.imshow`, which needs a display (`DISPLAY` env var set), same as the
  standalone `road_follower.sh` script. If running headless, set
  `enable_road_follower:=false` or fix the node to make the imshow optional.
- Next step: finish hardware calibration (see below), then run through the
  verification checklist above in order (A → B → C → full stack → field test).

## Final calibration values (2026-09-17, verified working)

`config/params.yaml`:
- `blob_angle_min_deg: -155`, `blob_angle_max_deg: -105`, `blob_range_min_m: 0.15`, `blob_range_max_m: 0.40`
- `calib_left_angle_deg: -122.0`, `calib_center_angle_deg: -129.5`, `calib_right_angle_deg: -135.0`
- `steering_controller.motor_sign: -1.0` (cmd polarity is inverted relative to `/steering_angle` sign on this robot)
- Confirmed on hardware: commanding `/steering_cmd` to +1.0 / -1.0 converges and holds at the correct physical lock; watchdog correctly recenters when `/steering_cmd` stops.

## Hardware calibration pitfalls learned (2026-09-17)

- `/steering_motor_cmd = 0.0` means **zero PWM / stop**, not "return to
  center" — the motor has no position feedback, it just halts wherever it is.
  Center must be found by visually nudging the wheel straight, not by
  publishing 0.0.
- A short (~1.5s) drive pulse was NOT enough to reach the true mechanical
  lock on the real rack — only produced ~13deg of raw angle swing (too
  narrow a span; amplifies lidar noise a lot once normalized to -1..1). Must
  drive continuously until the wheel visibly stops turning (stalls at the
  mechanical stop) before reading the calibration angle. Calibration is now
  a manual/interactive process — see updated procedure below.
- Blob angle window: ruler measurement (base_link frame, wheels straight)
  put the wheel/linkage blob at roughly -105..-155deg (a rear-side lobe, not
  a forward cone as originally assumed) and 0.15-0.40m range. Currently set
  in `config/params.yaml` as a **signed** window
  (`blob_angle_min_deg: -155`, `blob_angle_max_deg: -105`) — using
  `abs(angle)` was tried and is WRONG: both sides have close-range returns,
  and averaging across both cancels the signal to ~0.
  If your robot's geometry differs, re-derive from a ruler measurement (see
  git history of this file / ask for the calc) or inspect `/scan_c1` in
  Foxglove directly.
- `_map_to_normalized` in `steering_estimator_node.py` must not assume
  `calib_left > calib_center > calib_right` — on this robot the order came
  out reversed (right=-122 > center=-129.5 > left=-135.3, i.e. angle
  *decreases* going left). Fixed to a sign-agnostic generic interpolation
  that works regardless of ordering.
- This ROS2/rclpy build has **removed `Logger.warn()`** — must use
  `.warning()`. Fixed in `steering_estimator_node.py`. Pre-existing files
  `motor_driver/motor_node.py` (lines ~135, ~155) and
  `gamepad_teleop/gamepad_node.py` (line ~113) still use `.warn()` and will
  crash the node if that code path is ever hit — not fixed yet (out of scope
  of this task, flagged to user, consider patching before competition).

## Start/Pause GPIO control (added + verified on hardware, 2026-09-18)

New feature, separate from the steering-feedback phases above but built on
top of the same `robot_bringup`/`obstacle_avoidance` stack. Two momentary
buttons wired to the RPi5: GPIO17 = START, GPIO27 = PAUSE (active-low,
internal pull-up). Robot boots **paused**; gamepad drives freely; START
enables autonomous driving (obstacle_avoidance + road_follower via its
existing `/road_follower/error` input).

- New node `src/robot_bringup/robot_bringup/run_state_node.py`:
  - Polls GPIO17/27 via `lgpio` (debounced), publishes `/autonomy_enabled`
    (`std_msgs/Bool`) every tick, defaults to `false` (paused) at boot.
  - Bonus (now the default): `run_control_manage_gamepad` launch arg spawns
    `gamepad.sh` on PAUSE (kill-then-respawn fresh on *every* PAUSE press —
    fixes stale `/dev/input/js0` reads when the physical gamepad is powered
    on late) and kills it on START, via `subprocess.Popen(...,
    preexec_fn=os.setsid)` / `os.killpg`.
- `obstacle_avoidance/avoider_node.py` gated on `/autonomy_enabled`:
  - New param `autonomy_enabled_timeout_s` (default 1.0s) — staleness
    watchdog, fails safe (treated as paused) if `run_state_node` dies.
  - When paused/stale: publishes a single zero-command on the
    enabled→disabled transition, then goes fully silent — skips the state
    machine *and* the emergency-stop check entirely (human/gamepad has full
    responsibility while paused, to avoid two publishers racing on
    `/traction_motor_cmd` / `/steering_cmd`).
- `robot_bringup/launch/robot_launch.py`: new args `enable_run_control`
  (default `true`), `run_control_start_pin`/`run_control_pause_pin`
  (17/27), `run_control_manage_gamepad` (default `true`); includes
  `run_state_node`.
- Gamepad steering direction fix (found while testing this feature):
  `gamepad_teleop`'s `gamepad_launch.py` `invert_steering` arg default
  flipped to `true` — gamepad steering was reversed relative to
  `/steering_cmd`'s `+1=left` convention on this robot.
- Packaging: `robot_bringup` gained a console-script entry point
  (`run_state_node`) and `lgpio`/`std_msgs` exec_depends;
  `obstacle_avoidance/config/params.yaml` gained
  `autonomy_enabled_timeout_s: 1.00`.
- **Verified end-to-end on hardware**: paused-at-boot, gamepad free-drive,
  START → autonomous driving, PAUSE → instant stop, gamepad
  auto-respawn/kill all confirmed working; steering direction confirmed
  correct after the `invert_steering` fix.
- Not yet revisited: the `error_deadzone` TODO for
  `steering_controller_node.py` (see top of this file) — still open.

## Updated calibration procedure (manual/interactive, do this on hardware)

Bring up lidar + steering motor only (traction/camera/road_follower/
avoidance/steering_sensor's controller off), run
`steering_estimator_node` standalone with the params file, then in a
separate terminal:

1. `ros2 topic pub -r 20 /steering_motor_cmd std_msgs/msg/Float32 "{data: 1.0}"`
   — watch the wheel physically; once it visibly stops turning (hit the
   mechanical stop), wait ~1s more, then Ctrl+C.
2. Immediately read a few samples of
   `ros2 topic echo /steering_sensor/debug_raw_angle_deg` and note the
   stabilized value → `calib_left_angle_deg`.
3. Repeat with `{data: -1.0}` → `calib_right_angle_deg`.
4. Nudge with brief opposite-sign pulses until the wheel looks visually
   straight, read the value → `calib_center_angle_deg`.
5. Fill all three into `config/params.yaml`, rebuild
   (`colcon build --packages-select steering_sensor`), restart, and confirm
   `/steering_angle` reads ~+1/0/-1 at each position with `ros2 topic echo`.

