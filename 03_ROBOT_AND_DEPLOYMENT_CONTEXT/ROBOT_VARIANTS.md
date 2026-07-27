# Robot variants

## Physical inventory

| Unit | Compute | Current context |
|---|---|---|
| K1 Education | NVIDIA Jetson | Firmware 1.7.0.7 observed; 22-joint model; likely first integration target |
| K1 Geek | Qualcomm | Separate runtime target; exact onboard software inventory still needs confirmation |

The compute difference may affect packaging, dependencies, runtime
architecture, and deployment setup even though both robots use the same
general mechanical model.

## Available mechanical assets

The package includes the stock K1 22-DOF URDF, MuJoCo XML, locomotion URDF, and
STL meshes. No edition-specific mechanical difference was found in the
available local assets, but that does not prove the files exactly match both
delivered robots.

No custom payload, attachment, or added 3D asset is part of either task.

## Compatibility items to settle together

Your existing deployment code targets the K1 Pro Edition. Before the first
robot test, we should compare:

- Joint count, order, coordinate conventions, and limits
- Feedback message and observation ordering
- Runtime dependencies, architecture, and supported model format
- Controller mode-transition API
- Control rate, watchdog, and command freshness behavior
- Gain and effort conventions

The Education/Jetson robot is the default first target because we know more
about its current setup. We are open to selecting the Geek/Qualcomm robot if
it is a better match for your existing deployment workflow.
