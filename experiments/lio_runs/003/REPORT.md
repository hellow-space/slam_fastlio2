# FAST-LIVO2 offline A/B report

**RETEST**

- Outdoor high-load mean ≤80 ms **PASS**
- Outdoor high-load p95 ≤80 ms **PASS**
- Repeatability **unverified**
- Bag input hashes **verified by pipeline**
- Absolute trajectory accuracy **missing** (independent ground truth unavailable).
- At 10 Hz the expected input interval is 100 ms. Mean processing time does not predict output rate.
- The 197 ms historical measurement and the 145 ms observation use different conditions.
- State age uses the successful scan end. The odometry header and the corresponding camera_init → aft_mapped TF retain the node clock and matching poses.

## Evidence and decision

- outdoor pair 1: initialization IMU acceptance fingerprints differ
- outdoor: 1 successful paired runs; 2 required
- outdoor pair 1: paired_pose_max_m (paired) exceeds the frozen regression limit
- outdoor pair 1: paired_quaternion_component_max (paired) exceeds the frozen regression limit

## outdoor pair 1

| Window | Variant | Mean ms | p50 ms | p95 ms | Max ms | >80 ms | >100 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| all_after_5s | baseline | 49.758457211 | 37.435244000 | 83.148074000 | 147.534034000 | 115/1188 (9.68%) | 5/1188 (0.42%) |
| all_after_5s | candidate | 47.316166886 | 37.113355000 | 75.335066000 | 114.822610000 | 30/1188 (2.53%) | 6/1188 (0.51%) |
| high_load_40_100s | baseline | 66.133695277 | 74.844570000 | 85.706368000 | 147.534034000 | 115/600 (19.17%) | 5/600 (0.83%) |
| high_load_40_100s | candidate | 61.661802833 | 67.862786000 | 79.427681000 | 114.822610000 | 30/600 (5.00%) | 6/600 (1.00%) |

- baseline startup IMU status -2 count 7, first accepted raw stamp 1794745573.3904438, first 16 accepted stamps SHA-256 `24f3bb72b815e95d144801043a04b150dd2b6fab81a40a9f7606efcf1b332a63`.
- candidate startup IMU status -2 count 9, first accepted raw stamp 1794745573.397214, first 16 accepted stamps SHA-256 `76443bdd404094491fe6621ef273a3fec2b975b032058985e72c5f1a711f231a`.

Per-window state age, steady-mapped age, output interval, queues and paired pose difference are in metrics.csv and comparison.json. The comparison also records TF parity, raw scan audits and evaluator hashes.

