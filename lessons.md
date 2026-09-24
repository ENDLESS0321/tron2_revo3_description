# Project Lessons

## 2026-09-24 — Wrist-camera 180-degree rotation

- Context: Repositioning the paired wrist cameras to the opposite side of each
  TRON2 arm.
- Mistake: Rotated each camera assembly's orientation around the wrist-roll X
  axis but left its offset position unchanged, so the camera did not orbit to
  the axially opposite side of the arm.
- Rule: When a mounted component must rotate *around* an arm axis, apply the
  rotation to the complete rigid transform: rotate both the orientation and
  the radial position vector about that axis, then verify the resulting world
  position visually and numerically.

## 2026-09-24 — Keep wrist cameras at their original pose

- Context: Separating the requested Revo3 hand rotation from the wrist-camera
  mounting pose.
- Mistake: Continued applying a 180-degree wrist-roll transform to both wrist
  cameras after the user clarified that they must return to their original
  positions and orientations.
- Rule: Keep both wrist-camera root transforms at their pre-change values;
  rotate only the left and right hands unless the user explicitly requests a
  later camera-pose change.

## 2026-09-24 — Separate wrist-camera orientation from orbital position

- Supersedes the immediately preceding “Keep wrist cameras at their original
  pose” rule after the user's fuller clarification.
- Context: Final clarification of the wrist-camera placement request.
- Mistake: Treated “return the wrist-roll X rotation” as restoring the entire
  camera transform, including its position.
- Rule: Use the original wrist-camera orientations, but place both camera roots
  on the arm-axis opposite side (`z=+0.0753 m`). Do not pre-multiply the camera
  orientation by the arm-axis orbit rotation; orientation and orbital position
  are intentionally specified independently here.

## 2026-09-24 — Restore the complete original wrist-camera transforms

- Supersedes the earlier wrist-camera placement rules while the user evaluates
  the model from a known baseline.
- Context: The user wants both wrist cameras returned to their original state
  before deciding on any further change.
- Mistake: Kept the camera positions on the arm-axis opposite side after their
  attitudes were restored.
- Rule: Restore both wrist-camera root positions and orientations exactly to
  the pre-1.2.0 values. Keep the hand-root rotations and head D455 unchanged.
