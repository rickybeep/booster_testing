# Safety and acceptance

## Current authorization boundary

This package is for review and offline work. It does not authorize:

- Changing robot mode
- Publishing LowCmd or MotorCmd
- Running a policy on a physical robot
- Moving any joint
- Installing or deploying modified code

Some source files are intentionally capable of actuation in their original
use. Treat every script as review-only unless a specific test is separately
approved.

## What does not count as physical success

- Receiving LowState
- A local publisher call returning successfully
- Seeing the topic in a ROS graph
- A clean process exit
- Entering CUSTOM
- The robot not immediately falling

## Minimum evidence sequence

1. While remaining in WALK, the exact command writer reports at least one
   compatible matched subscription without constructing or publishing a
   LowCmd.
2. With separate approval and physical support/hoist, a complete 22-entry
   current-position hold is continuously published before, during, and after
   CUSTOM entry.
3. Firmware logs show a valid 22-entry command snapshot, with no repeated
   snapshot failure or size mismatch.
4. Gain and effort units are calibrated with fixed-position tests, not with a
   motion policy.
5. End-to-end command latency is measured.
6. Only then is a bounded squat attempted with a spot/hoist, freshness
   watchdogs, limit enforcement, a verified return path, and explicit approval.

## Required squat success

A successful squat must complete the uninterrupted reference cycle without a
reset, fall, stale command, target/effort violation, or firmware rejection. It
must return to a stable standing state under policy control and exit through
the agreed safe handoff.
