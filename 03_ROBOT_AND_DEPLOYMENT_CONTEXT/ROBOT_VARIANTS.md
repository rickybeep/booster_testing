# Robot variants

## Physical inventory

| Unit | Compute | What is confirmed | What remains unknown |
|---|---|---|---|
| K1 Education | NVIDIA Jetson | Edition from firmware log; model version 1.2.1; firmware 1.7.0.7; 22-joint contract; installed BoosterRos2Interface; active single-board UDP-only DDS profile | Physical gains, effort units, latency, and successful command acceptance |
| K1 Geek | Qualcomm | User-provided edition/compute identity; Booster platform documentation exposes a `real_qcom` target | Firmware, model version, installed SDK/ROS packages, DDS profile/interfaces, command-reader lifecycle, ABI, and deployment compatibility |

The available platform documentation distinguishes `real_jetson` and
`real_qcom` deployment targets. Therefore, the compute difference is not just
a performance detail; packaging, dependencies, architecture, and service
layout may differ even when the mechanical 22-DOF model is shared.

## Available mechanical assets

The package includes the stock K1 22-DOF URDF, MuJoCo XML, locomotion URDF, and
STL meshes. No edition-specific mechanical difference was found in the
available local assets, but that does not prove the files exactly match both
delivered robots.

No custom payload, attachment, or added 3D asset is part of the squat task.

## Compatibility question for the collaborators

Their existing deployment code targets the K1 Pro Edition. Before any robot
test, compare the following across K1 Pro, Education/Jetson, and Geek/Qualcomm:

- Joint count, order, serial/parallel ankle semantics, and limits
- Feedback message and observation ordering
- Installed ROS message definitions
- DDS domain, profile, QoS, topic names, and allowed interfaces
- Policy runtime architecture and supported model format
- Controller mode-transition API
- Control rate, watchdog, and command freshness behavior
- Gain and effort conventions

The recommended first target is Education/Jetson because its firmware and
failure evidence are already captured. The Geek should receive a separate
read-only inventory before reusing the deployment package.
