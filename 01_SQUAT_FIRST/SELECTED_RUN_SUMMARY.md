# Training run inventory

The complete frozen source log tree contained nine squat runs and 106 files.
To keep this package practical, it includes:

- All five checkpoints from the selected final run
- The selected TorchScript and ONNX exports
- Final-run environment and agent parameters
- TensorBoard event data
- The Booster Train local diff recorded by the run
- Two representative replay videos from earlier resumed runs

The final run is:

`2026-07-10_17-16-29_cliff_10_30ms_resume_799_to_1200`

Its five checkpoints were all evaluated under the corrected offline test:

| Checkpoint | Randomized pass | 30 ms | 35 ms | 40 ms |
|---|---:|---|---|---|
| model_800 | 16/20 | pass | fail | fail |
| model_900 | 16/20 | pass | fail | fail |
| model_1000 | 15/20 | pass | fail | fail |
| model_1100 | 13/20 | fail | fail | fail |
| model_1198 | 16/20 | pass | fail | fail |

`model_1198` was retained because it tied for the best randomized result and
had the smallest maximum target-limit overrun among the three 16/20
checkpoints. It is not a physical deployment candidate yet.

The two MP4s in `representative_videos/` are earlier policy replays and must
not be represented as model 1198 validation.
