# FAST-LIVO2 offline A/B report

**RETEST**

- Outdoor high-load mean ≤80 ms **PASS**
- Outdoor high-load p95 ≤80 ms **FAIL**
- Repeatability **unverified**
- Bag input hashes **verified by pipeline**
- Absolute trajectory accuracy **missing** (independent ground truth unavailable).
- At 10 Hz the expected input interval is 100 ms. Mean processing time does not predict output rate.
- The 197 ms historical measurement and the 145 ms observation use different conditions.
- State age uses the successful scan end. The odometry header and the corresponding camera_init → aft_mapped TF retain the node clock and matching poses.

## Evidence and decision

- outdoor pair 1: initialization IMU acceptance fingerprints differ
- indoor: 1 successful paired runs; 2 required
- outdoor: 1 successful paired runs; 2 required
- outdoor pair 1: paired_pose_max_m (paired) exceeds the frozen regression limit
- outdoor pair 1: paired_quaternion_component_max (paired) exceeds the frozen regression limit

## indoor pair 1

| Window | Variant | Mean ms | p50 ms | p95 ms | Max ms | >80 ms | >100 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| all_after_5s | baseline | 30.314815711 | 30.784491000 | 39.386052000 | 58.763735000 | 0/211 (0.00%) | 0/211 (0.00%) |
| all_after_5s | candidate | 30.753049687 | 30.665022000 | 39.294731000 | 56.661005000 | 0/211 (0.00%) | 0/211 (0.00%) |

- baseline startup IMU status -2 count 0, first accepted raw stamp 1794488403.5395384, first 16 accepted stamps SHA-256 `5006f91d6645ce24767f21ea97fd99f6fe77ab7cae94388c91d158ccd32d27a5`.
- candidate startup IMU status -2 count 0, first accepted raw stamp 1794488403.5395384, first 16 accepted stamps SHA-256 `5006f91d6645ce24767f21ea97fd99f6fe77ab7cae94388c91d158ccd32d27a5`.

Per-window state age, steady-mapped age, output interval, queues and paired pose difference are in metrics.csv and comparison.json. The comparison also records TF parity, raw scan audits and evaluator hashes.

## outdoor pair 1

| Window | Variant | Mean ms | p50 ms | p95 ms | Max ms | >80 ms | >100 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| all_after_5s | baseline | 50.837002680 | 38.410994000 | 84.454615000 | 147.832923000 | 161/1188 (13.55%) | 8/1188 (0.67%) |
| all_after_5s | candidate | 49.394261630 | 37.394274000 | 82.741363000 | 120.399288000 | 113/1188 (9.51%) | 5/1188 (0.42%) |
| high_load_40_100s | baseline | 67.150606278 | 75.872227000 | 87.131793000 | 147.832923000 | 157/600 (26.17%) | 8/600 (1.33%) |
| high_load_40_100s | candidate | 65.653088325 | 74.508700000 | 85.475690000 | 120.399288000 | 112/600 (18.67%) | 5/600 (0.83%) |

- baseline startup IMU status -2 count 6, first accepted raw stamp 1794745573.3851538, first 16 accepted stamps SHA-256 `788b1d27d5850a33da1a99859df26cfbe418e9f5235f2c106061ce1aecdac99c`.
- candidate startup IMU status -2 count 7, first accepted raw stamp 1794745573.3904438, first 16 accepted stamps SHA-256 `24f3bb72b815e95d144801043a04b150dd2b6fab81a40a9f7606efcf1b332a63`.

Per-window state age, steady-mapped age, output interval, queues and paired pose difference are in metrics.csv and comparison.json. The comparison also records TF parity, raw scan audits and evaluator hashes.

