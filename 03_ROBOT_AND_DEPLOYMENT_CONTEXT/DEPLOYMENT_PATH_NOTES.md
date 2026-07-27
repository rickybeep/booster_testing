# Upstream pins and isolated changes

## Upstream snapshots

| Component | Commit |
|---|---|
| Booster Assets | `508cbee6ca9ae6fbc8c0b38dd58785a6f3fc61a2` |
| Booster Train | `651b7a53f2ffaf2d5629d0604d065cc385e29c6b` |
| Booster Deploy | `563e7c37e7468b3b96ad9f84f9e61becea3b7fb8` |
| Copied Booster SDK baseline | `324946e7c885de6cd6d37220eca64ed4e0f8d418` |
| Audited current SDK 1.7 reference | `d5d8f7ae76d3e9f8cc224e682216f4681003ca46` |

## Isolated deployment prototype

The included Booster Deploy source is not a pristine upstream checkout. The
isolated prototype:

- Keeps the ROS publisher in its creating parent process.
- Transfers inference actions through shared memory.
- Continuously republishes the current safe hold through prefill, mode
  transition, policy loading, and first-action readiness.
- Requires a directly matched subscription.
- Requires fresh LowState with exactly 22 entries.
- Checks the mode RPC and confirms CUSTOM.
- Rejects stale, wrong-length, non-finite, out-of-range, and predicted
  over-effort targets.
- Keeps both squat tasks hardware locked.
- Adds a separate training-gain A/B configuration for offline comparison.

These changes were source- and unit-tested only. They were not deployed.

## Why raw DDS was deprioritized

The raw Python path used an older B1-oriented ABI, default/BEST_EFFORT
publication, no exposed match-count method, and PARALLEL ankle coordinates.
The installed official ROS route provides the robot's message definitions,
RELIABLE depth-1 publication, direct subscription-count inspection, and
explicit SERIAL K1 commands.

The raw route remains useful as a diagnostic only after SDK/runtime, QoS, type,
profile, and direct match are proven.
