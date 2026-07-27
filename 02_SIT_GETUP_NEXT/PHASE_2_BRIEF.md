# Phase 2: sit down and stand back up

## Desired outcome

After the squat workflow is working, create a learned whole-body behavior in
which the robot:

1. Begins standing.
2. Crouches under control.
3. Reaches its arms behind it for ground support.
4. Places its hands and then its butt on the ground without tipping forward.
5. Extends or settles its legs into a comfortable floor position.
6. Remains seated or resting briefly.
7. Reverses the motion and returns to a stable stand.

The human source video communicates the intended overall motion. It is not
expected to map literally to K1 proportions, limits, contacts, or strength.

## Included reference material

### Source video

- `source_video/lying_down_standing_up.mov` — human reference clip for the
  desired sequence.
- `source_video/lying_down_standing_up_720x1280.mp4` — normalized processing
  input.

### GVHMR example

- `gvhmr_example/hmr4d_results.pt` — captured GVHMR result.
- `gvhmr_example/gvhmr_preview.mp4` — visualized reconstruction.

### K1 retarget example

- `retarget_example/k1_retarget_side_preview.mp4` — side-view K1 retarget.
- `retarget_example/k1_sit_getup_reference_50fps.csv` — initial retarget CSV.
- `retarget_example/k1_sit_getup_reference_50fps.npz` — converted 50 Hz K1
  reference.
- `retarget_example/k1_retarget_contact_sheet.jpg` — quick visual summary.
- `retarget_example/studio_sit_keyframe_concepts.png` — supported-sit
  keyframe concepts explored after the first retarget tipped forward.

These files show our direction and prior work. We expect the motion,
contacts, rewards, and policy to be revised substantially.

## Main issue observed so far

Our first retarget/training attempts crouched but did not reliably transition
onto the butt. The motion tended to tip forward rather than establish hand
support behind the body. This is why the desired contact sequence is stated
explicitly above.

## Phase 2 collaboration target

After the squat workflow is complete, reuse its task structure, evaluation,
export, and deployment path where practical. The sit/get-up task will need a
new contact strategy, reference treatment, rewards, and likely a staged
curriculum.
