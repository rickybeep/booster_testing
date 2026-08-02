# Arena, UWB, and Robot Calibration

> Calibration may prepare configuration, but it does not authorize powered
> movement. Keep `live_motion_enabled` false and every robot disabled while
> establishing the coordinate contract.

The tracker joins three frames: UWB arena coordinates, the robot's IMU heading,
and the offset from the UWB tag to the desired robot-center point. A wrong
origin, anchor order, sign, or offset can make correct navigation logic command
the wrong physical direction.

## 1. Establish the arena frame

Record a physical origin, positive X and Y directions, and dimensions in
meters. Configure:

- `arena.bounds_m` as `[min_x, min_y, max_x, max_y]`;
- `arena.safe_margin_m` as the inward margin on every edge; and
- each `arena.static_keepouts` entry as a labeled circular center and radius.

The loader rejects a margin that removes the entire safe interior. Every
waypoint must lie inside that interior and must not overlap a static keepout
after robot radius and planning margin are included; see
[pc/tracker_config.py](pc/tracker_config.py#L279).

For every enabled robot, the live safety relationships also require:

```text
safe_margin_m >= keepout_radius_m
                 + max_forward_mps * PC_deadman_seconds
                 + maximum_residual_m
```

This is a conservative configuration invariant, not a measured stopping model.
The recovery hysteresis must also leave a nonempty interior
([pc/tracker_config.py](pc/tracker_config.py#L134)).

The software keepout is a geometric planning input, not a measured stopping
guarantee or a safety-rated boundary. Retain additional physical clearance.

## 2. Record anchors in wire order

`uwb.anchors_m` is an ordered list of `[x, y]` coordinates. For an `mc` range
packet, range slot zero is paired with anchor-list index zero, and so on
([pc/track_uwb.py](pc/track_uwb.py#L119)). Do not infer this order from the order
in which serial messages happen to arrive.

Also record:

- `uwb.port`: an explicit serial device or `AUTO`;
- `uwb.baud`;
- `uwb.allow_unverified_lo`: keep false unless the reduced quality evidence of
  a device-solved `LO` fix has been explicitly reviewed;
- `uwb.anchor_to_tag_height_m`: the vertical separation used to project
  slant ranges into the horizontal plane;
- `uwb.position_stale_s`;
- `uwb.minimum_anchors`; and
- `uwb.maximum_residual_m`.

The reader can accept a device-computed `LO` position or solve an `mc` packet.
For `LO`, `n_anchors == 0` means contributing-anchor count and residual quality
are not observable from the wire; it does not mean that zero anchors were
proven healthy ([pc/track_uwb.py](pc/track_uwb.py#L412)).

When quality policy is supplied to `UwbTracker`, rejected source, anchor-count,
or residual metadata is discarded before the median/Kalman filter is updated
([pc/track_uwb.py](pc/track_uwb.py#L280)). The runner checks the atomic sample
again before trusting it.

## 3. Map tags and variants

For each physical robot, record its stable name, hardware variant, UWB tag ID,
panel addresses, and tag mounting location. The accepted variant labels are
`education_jetson` and `geek_qualcomm_qrb5165`; these labels select no vendor
binary and prove no compatibility.

Confirm tag identity by observing read-only UWB data while locomotion is
disabled. Never identify a tag by commanding a robot to move.

## 4. Calibrate heading

The PC converts raw yaw radians to arena degrees as:

```text
heading_deg = (imu_yaw_sign * degrees(raw_yaw) + imu_yaw_offset_deg) mod 360
```

`imu_yaw_sign` must be `-1` or `1`. The implementation is in
[pc/booster_motion.py](pc/booster_motion.py#L1143).

With the command path disabled, face the robot along marked arena axes and
collect multiple advancing yaw samples. Determine the sign and offset from
those observations. Repeat at +X, +Y, and -X to catch a sign or wrap error.
Do not tune the sign by trial-and-error powered turning.

The separate `K1_TURN_CMD_SIGN` converts the navigator's turn convention to
the Booster command convention. Do not compensate for a bad IMU calibration
by changing the command sign.

## 5. Measure the tag-to-center vector

In this schema, `tag_offset_m` is the vector from the reported tag point to the
desired robot-center point, expressed in robot-forward and robot-left axes.
The runner applies:

```text
center_x = tag_x + forward*cos(heading) - left*sin(heading)
center_y = tag_y + forward*sin(heading) + left*cos(heading)
```

See [pc/arena_runner.py](pc/arena_runner.py#L65). If a measurement instead
describes the tag relative to robot center, reverse that vector before entering
it. A correct center estimate should remain approximately fixed when the
stationary robot is manually reoriented safely about its body center.

## 6. Set waypoints and conservative limits

`robots[].waypoints_m` is an ordered fixed patrol loop. The runner turns,
walks, dwells, scans its head, and advances to the next point. Static keepouts
and peer robots are circular obstacles; the effective planning clearance starts
with the sum of the relevant radii.

Begin configuration review with conservative values. The loader caps
`max_forward_mps` at 0.4 m/s and `max_turn_rad_s` at 0.8 rad/s, matching the
robot guard's absolute bounds. Passing validation proves only that values are
well formed and within software limits; it does not prove they are physically
safe.

The runner holds at zero if an observed robot center is outside the configured
safe interior. Although the vendored Navigator retains recovery helpers, this
fixed-waypoint runner does not command recovery motion across that boundary.

Head scan yaw values are radians and must remain within +/-1.1. A head command
is a physical command and belongs only to a separately approved live test.

## 7. Validate without hardware

From the package root:

```powershell
py -3.10 pc\arena_runner.py --config config\tracker.example.json
```

For a site file, copy the example to ignored `config/tracker.local.json`, keep
`live_motion_enabled` false, keep all robots disabled, enter measured values,
and run the same command against that local path. Without `--live`, the runner
only parses and validates configuration
([pc/arena_runner.py](pc/arena_runner.py#L468)).

Record who measured each value, when, with what instrument, and which offline
checks passed. Any robot-connected validation is a separate step governed by
[TESTING_CHECKLIST.md](TESTING_CHECKLIST.md).
