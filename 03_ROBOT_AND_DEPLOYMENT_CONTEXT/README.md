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
