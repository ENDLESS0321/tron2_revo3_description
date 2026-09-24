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
