# Robot and deployment context

## Physical robots

- K1 Education with NVIDIA Jetson compute
- K1 Geek with Qualcomm compute

Our prior squat work reached simulation playback and policy export, but we did
not complete a reliable physical deployment. We would like to adapt your
existing K1 deployment workflow rather than carry forward our experimental
path.

See:

- [`ROBOT_VARIANTS.md`](ROBOT_VARIANTS.md)
- [`PHYSICAL_TESTING_NOTES.md`](PHYSICAL_TESTING_NOTES.md)

## Robot assets

`stock_k1_22dof_model/` contains the stock K1 URDF, MuJoCo XML, locomotion
URDF, meshes, ZED-head variant, and Booster Assets license used by our work.

No custom payload or attachment is part of either requested behavior.

## Baseline SDK arena-control example

[`arena_patrol_example/`](arena_patrol_example/) is a focused, standalone
example of our current K1 control path: UWB tracking, arena-heading
calibration, bounded waypoint navigation, stop-and-look head movement, and
the onboard Booster SDK bridge. It excludes the camera, person-detection,
streaming, audio, language-model, and dashboard layers used by the larger
installation.
