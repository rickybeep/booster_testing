# Physical testing notes

Physical tests should be coordinated in advance with both teams present.

Before attempting a learned motion:

1. Agree on the exact robot, software environment, controller mode sequence,
   and stop procedure.
2. Review the selected policy in simulation and confirm joint and observation
   conventions.
3. Validate the deployment path with a stationary supported test.
4. Use physical support or a spotter, keep immediate stop access available,
   and begin with conservative motion limits.
5. Record the result so training and deployment changes can be reviewed
   together.

A successful squat should descend smoothly, maintain balance and controlled
feet and knees, return to a stable stand, and exit through the agreed control
handoff without a fall or limit violation.
