# Firmware CUSTOM canary excerpt

Source: read-only copy of the K1 Education firmware motion log.

Only the relevant lines are reproduced here so the collaboration package does
not expose the complete robot log or device identity.

```text
[07-14 00:56:58.260573] [info] [mode.hpp:165] OnEnter PrepareMode
[07-14 00:56:58.260575] [info] [body_control.hpp:50] OnEnter PrepareBodyControl
[07-14 00:56:58.260577] [info] [mode_manager.cpp:696] exec switch to prepare

[07-14 00:57:10.887949] [info] [mode_manager.cpp:748] exec switch to CustomMode
[07-14 00:57:10.890260] [info] [body_control.hpp:53] OnExit PrepareBodyControl
[07-14 00:57:10.892268] [info] [mode.hpp:173] OnExit PrepareMode
[07-14 00:57:10.892279] [info] [mode.hpp:165] OnEnter CustomMode
[07-14 00:57:10.892282] [info] [booster_body_controls.hpp:535] OnEnter CustomBodyControl
[07-14 00:57:10.892287] [info] [mode_manager.cpp:284] mode: CustomMode switched
[07-14 00:57:10.892326] [warning] [custom_body_control_module.cpp:112] Failed to get custom command snapshot
[07-14 00:57:10.894278] [warning] [custom_body_control_module.cpp:112] Failed to get custom command snapshot
[07-14 00:57:10.896287] [warning] [custom_body_control_module.cpp:112] Failed to get custom command snapshot

[07-14 00:57:11.894002] [info] [booster_body_controls.hpp:601] GetCmdSnapshot Update Freq: 500.167 Hz
[07-14 00:57:11.894007] [warning] [custom_body_control_module.cpp:112] Failed to get custom command snapshot
[07-14 00:57:11.895992] [warning] [custom_body_control_module.cpp:112] Failed to get custom command snapshot
[07-14 00:57:11.895999] [error] [custom_body_control_module.cpp:58] User motor command size mismatch, expected 22, got 0

[07-14 00:57:12.107068] [info] [booster_body_controls.hpp:577] Reset Custom Command----
[07-14 00:57:12.107072] [info] [booster_body_controls.hpp:543] OnExit CustomBodyControl
[07-14 00:57:12.107073] [info] [mode.hpp:173] OnExit CustomMode
[07-14 00:57:12.107077] [info] [mode.hpp:165] OnEnter PrepareMode
[07-14 00:57:12.107079] [info] [body_control.hpp:50] OnEnter PrepareBodyControl
[07-14 00:57:12.107081] [info] [mode_manager.cpp:696] exec switch to prepare
```

The complete copied log contained 607 snapshot failures during this event.
The actual squat policy was not used.
