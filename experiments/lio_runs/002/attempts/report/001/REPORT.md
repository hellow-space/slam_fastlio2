# FAST-LIVO2 offline A/B report

**RETEST**

- Outdoor high-load mean ≤80 ms **FAIL**
- Outdoor high-load p95 ≤80 ms **FAIL**
- Repeatability **unverified**
- Bag input hashes **verified by pipeline**
- Absolute trajectory accuracy **missing** (independent ground truth unavailable).
- At 10 Hz the expected input interval is 100 ms. Mean processing time does not predict output rate.
- The 197 ms historical measurement and the 145 ms observation use different conditions.
- State age uses the successful scan end. The odometry header and the corresponding camera_init → aft_mapped TF retain the node clock and matching poses.

## Evidence and decision

- indoor: 1 successful paired runs; 2 required
- outdoor: 1 successful paired runs; 2 required
- outdoor pair 1: candidate full_cycle mean exceeds 80 ms
- outdoor pair 1: outdoor_high_load_wall_mapped_age_p95_candidate_max (candidate) exceeds the frozen regression limit
- outdoor pair 1: outdoor_high_load_lidar_q_max_candidate_max (candidate) exceeds the frozen regression limit

## indoor pair 1

| Window | Variant | Mean ms | p50 ms | p95 ms | Max ms | >80 ms | >100 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| all_after_5s | baseline | 39.610479981 | 39.477956000 | 49.789280000 | 53.586040000 | 0/211 (0.00%) | 0/211 (0.00%) |
| all_after_5s | candidate | 40.584066242 | 40.103518000 | 53.359355000 | 74.080960000 | 0/211 (0.00%) | 0/211 (0.00%) |

- baseline startup IMU status -2 count 0, first accepted raw stamp 1794488403.5395384, first 16 accepted stamps SHA-256 `5006f91d6645ce24767f21ea97fd99f6fe77ab7cae94388c91d158ccd32d27a5`.
- candidate startup IMU status -2 count 0, first accepted raw stamp 1794488403.5395384, first 16 accepted stamps SHA-256 `5006f91d6645ce24767f21ea97fd99f6fe77ab7cae94388c91d158ccd32d27a5`.

Per-window state age, steady-mapped age, output interval, queues and paired pose difference are in metrics.csv and comparison.json. The comparison also records TF parity, raw scan audits and evaluator hashes.

## outdoor pair 1

| Window | Variant | Mean ms | p50 ms | p95 ms | Max ms | >80 ms | >100 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| all_after_5s | baseline | 63.939547136 | 49.407514000 | 105.207869000 | 133.056881000 | 453/1188 (38.13%) | 184/1188 (15.49%) |
| all_after_5s | candidate | 64.015163785 | 49.903500000 | 106.699832000 | 128.394848000 | 443/1188 (37.29%) | 182/1188 (15.32%) |
| high_load_40_100s | baseline | 83.658236847 | 94.876165000 | 108.627282000 | 133.056881000 | 433/600 (72.17%) | 180/600 (30.00%) |
| high_load_40_100s | candidate | 83.651363075 | 95.468169000 | 109.615722000 | 128.394848000 | 424/600 (70.67%) | 177/600 (29.50%) |

- baseline startup IMU status -2 count 7, first accepted raw stamp 1794745573.3904438, first 16 accepted stamps SHA-256 `24f3bb72b815e95d144801043a04b150dd2b6fab81a40a9f7606efcf1b332a63`.
- candidate startup IMU status -2 count 7, first accepted raw stamp 1794745573.3904438, first 16 accepted stamps SHA-256 `24f3bb72b815e95d144801043a04b150dd2b6fab81a40a9f7606efcf1b332a63`.

Per-window state age, steady-mapped age, output interval, queues and paired pose difference are in metrics.csv and comparison.json. The comparison also records TF parity, raw scan audits and evaluator hashes.

