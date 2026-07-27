# Robot and deployment context

## Physical robots

- K1 Education — NVIDIA Jetson compute
- K1 Geek — Qualcomm compute

The detailed read-only firmware evidence currently applies to the
Education/Jetson robot. The Geek/Qualcomm robot still needs the same
environment inventory before assuming deployment compatibility.

## Known Education evidence

- Edition: K1 Education
- Model version: 1.2.1
- Firmware:
  `1.7.0.7-release-00040-2026-07-02-global`
- K1 feedback and the CUSTOM consumer expect 22 joints.

See:

- `ROBOT_VARIANTS.md`
- `CURRENT_STATUS.md`
- `FIRMWARE_CANARY_EXCERPT.md`
- `SAFETY_AND_ACCEPTANCE.md`
- [`WHAT_WE_TRIED.md`](WHAT_WE_TRIED.md)
- [`DEPLOYMENT_PATH_NOTES.md`](DEPLOYMENT_PATH_NOTES.md)

## Deployment problem encountered

Our raw Python DDS hold canary received LowState and completed local writer
calls, but those observations did not prove that firmware had accepted a
command. Firmware repeatedly reported:

```text
Failed to get custom command snapshot
User motor command size mismatch, expected 22, got 0
```

This should be treated as a transport/interface handoff failure preceding
policy interpretation. We are particularly interested in adapting your known
K1 deployment route instead of assuming our raw DDS prototype was correct.

## Robot assets

`stock_k1_22dof_model/` contains the stock K1 URDF, MuJoCo XML, locomotion
URDF, meshes, ZED-head variant, and Booster Assets license used by our work.

No included code or asset is authorization to actuate a physical robot.
