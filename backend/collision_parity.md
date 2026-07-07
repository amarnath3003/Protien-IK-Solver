# Collision-Proxy vs Real-Mesh Parity (Phase 3)

Per-config comparison of our capsule `self_collision_min_distance` against PyBullet real-mesh `getClosestPoints`, over random configs.

| Robot | n | real col% | proxy col% | sign-agree% | **false-clear%** | false-alarm% | gap mean (m) | corr (near) | calibrated δ (m) |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| ur5 | 3000 | 36.5 | 16.9 | 79.2 | **20.2** | 0.6 | +0.0139 | 0.837 | none≤0.15 |
| franka_panda | 3000 | 9.9 | 0.5 | 90.5 | **9.5** | 0.0 | +0.0179 | 0.412 | none≤0.15 |

- **false-clear%** = proxy reports clearance ≥ 0 while the real meshes interpenetrate. This is the quadrant a solver optimizing the proxy cannot see — it accepts these as collision-free. Driving it down is the point of recalibration.
- **gap** = proxy − real; positive means the proxy over-claims clearance (optimistic).
- **calibrated δ** = smallest proxy-clearance threshold that makes real collision ≤1% likely; a usable safety margin for the collision energy if the correlation is strong enough.

## Where the proxy fails — per-link-pair attribution

For each config the driving pair is the one whose real meshes are closest. Below: the pairs that drive real collision overall, and the pairs behind the proxy's dangerous false-clears.

### ur5

| driver of… | top link pairs (share of that quadrant) |
|:--|:--|
| real collision (n=1094) | `forearm_link|wrist_2_link` 41%, `forearm_link|wrist_3_link` 28%, `upper_arm_link|wrist_1_link` 15%, `upper_arm_link|wrist_2_link` 11%, `upper_arm_link|wrist_3_link` 3% |
| **false-clear** (n=606) | `forearm_link|wrist_2_link` 73%, `forearm_link|wrist_3_link` 16%, `upper_arm_link|wrist_2_link` 5%, `upper_arm_link|wrist_1_link` 3%, `upper_arm_link|wrist_3_link` 3% |

### franka_panda

| driver of… | top link pairs (share of that quadrant) |
|:--|:--|
| real collision (n=298) | `panda_link5|panda_link7` 69%, `panda_link0|panda_link5` 7%, `panda_link0|panda_link6` 5%, `panda_link1|panda_link5` 5%, `panda_link2|panda_link5` 4% |
| **false-clear** (n=284) | `panda_link5|panda_link7` 73%, `panda_link0|panda_link5` 6%, `panda_link1|panda_link5` 5%, `panda_link0|panda_link6` 5%, `panda_link2|panda_link5` 4% |
