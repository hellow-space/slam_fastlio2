# FAST-LIVO2 offline A/B report

**RETEST**

- Outdoor high-load mean ≤80 ms **PASS**
- Outdoor high-load p95 ≤80 ms **FAIL**
- Repeatability **unverified**
- Bag input hashes **recorded metadata only; bag files not verified by report**
- Absolute trajectory accuracy **missing** (independent ground truth unavailable).
- At 10 Hz the expected input interval is 100 ms. Mean processing time does not predict output rate.
- The 197 ms historical measurement and the 145 ms observation use different conditions.
- State age uses the successful scan end. The odometry header and the corresponding camera_init → aft_mapped TF retain the node clock and matching poses.

## Evidence and decision

- bag file hashes were recorded but not verified for this analysis
- outdoor pair 1: initialization IMU acceptance fingerprints differ
- indoor: 1 successful paired runs; 2 required
- outdoor: 1 successful paired runs; 2 required
- indoor pair 1: paired_pose_max_m (paired) exceeds the frozen regression limit
- indoor pair 1: paired_quaternion_component_max (paired) exceeds the frozen regression limit
- outdoor pair 1: paired_pose_max_m (paired) exceeds the frozen regression limit
- outdoor pair 1: paired_quaternion_component_max (paired) exceeds the frozen regression limit

## indoor pair 1

| Window | Variant | Mean ms | p50 ms | p95 ms | Max ms | >80 ms | >100 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| all_after_5s | baseline | 39.952535739 | 39.822247000 | 50.884116000 | 73.025646000 | 0/211 (0.00%) | 0/211 (0.00%) |
| all_after_5s | candidate | 31.619765787 | 31.261880000 | 41.525233000 | 66.922969000 | 0/211 (0.00%) | 0/211 (0.00%) |

- baseline startup IMU status -2 count 0, first accepted raw stamp 1794488403.5395384, first 16 accepted stamps SHA-256 `5006f91d6645ce24767f21ea97fd99f6fe77ab7cae94388c91d158ccd32d27a5`.
- candidate startup IMU status -2 count 0, first accepted raw stamp 1794488403.5395384, first 16 accepted stamps SHA-256 `5006f91d6645ce24767f21ea97fd99f6fe77ab7cae94388c91d158ccd32d27a5`.

Per-window state age, steady-mapped age, output interval, queues and paired pose difference are in metrics.csv and comparison.json. The comparison also records TF parity, raw scan audits and evaluator hashes.

## outdoor pair 1

| Window | Variant | Mean ms | p50 ms | p95 ms | Max ms | >80 ms | >100 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| all_after_5s | baseline | 62.174239345 | 48.284596000 | 104.025758000 | 134.917388000 | 440/1188 (37.04%) | 119/1188 (10.02%) |
| all_after_5s | candidate | 49.560105697 | 38.298501000 | 83.645673000 | 145.700583000 | 121/1188 (10.19%) | 5/1188 (0.42%) |
| high_load_40_100s | baseline | 81.037273615 | 91.228167000 | 107.676407000 | 134.917388000 | 417/600 (69.50%) | 114/600 (19.00%) |
| high_load_40_100s | candidate | 65.903088440 | 73.782086000 | 87.086002000 | 145.700583000 | 121/600 (20.17%) | 5/600 (0.83%) |

- baseline startup IMU status -2 count 7, first accepted raw stamp 1794745573.3904438, first 16 accepted stamps SHA-256 `24f3bb72b815e95d144801043a04b150dd2b6fab81a40a9f7606efcf1b332a63`.
- candidate startup IMU status -2 count 15, first accepted raw stamp 1794745573.4272366, first 16 accepted stamps SHA-256 `9105748515fc16368951d3845ecbb24d2962c9e1585dc79dd50a0e73a57ced1b`.

Per-window state age, steady-mapped age, output interval, queues and paired pose difference are in metrics.csv and comparison.json. The comparison also records TF parity, raw scan audits and evaluator hashes.

