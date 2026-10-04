# Intelligent Object Monitoring System for Autonomous Vehicles: Safety Enhancing

An end-to-end MATLAB pipeline that classifies the behavior of surrounding road users as **safe** or **risky** from their motion alone, measures honestly how well that classifier survives a realistic **radar + multi-object tracker**, and finally places it inside a **live closed-loop demo** where its predictions change how the ego vehicle drives.

> Final-year project, Department of Computer Science and Engineering, Shyamoli Engineering College.

**Keywords:** autonomous vehicles · behavior classification · radar · multi-object tracking · extended Kalman filter · decision tree · data leakage · sensor robustness · MATLAB Automated Driving Toolbox

---

## Table of Contents

- [Overview](#overview)
- [Key Results](#key-results)
- [Pipeline](#pipeline)
- [Dataset](#dataset)
- [Motion Features](#motion-features)
- [Behavior Module Library](#behavior-module-library)
- [Sensing and Tracking Setup](#sensing-and-tracking-setup)
- [Live Closed-Loop Demonstration](#live-closed-loop-demonstration)
- [Repository Structure](#repository-structure)
- [Requirements](#requirements)
- [Getting Started](#getting-started)
- [Reproducibility](#reproducibility)
- [Limitations and Future Work](#limitations-and-future-work)
- [License](#license)

---

## Overview

An autonomous vehicle must do more than detect and track objects. It must judge whether each object is behaving safely or dangerously. A car that changes lanes smoothly and a car that swerves aggressively can occupy the same point at the same speed, yet they demand very different responses.

This project:

1. **Generates** 450 driving scenarios programmatically in MATLAB across three complexity tiers.
2. **Labels** every actor using a library of named behavior modules, each with a ground-truth safe/risky label.
3. **Extracts** ten interpretable motion features per actor trajectory.
4. **Trains** a fine decision tree classifier (with a data-leakage audit and a scenario-level train/test split).
5. **Stress-tests** the classifier against radar detections passed through a multi-object tracker (constant-velocity EKF).
6. **Closes the loop** by letting the classifier adjust the ego vehicle's following distance and lateral behavior in real time.

The central contribution is not one accuracy number but an honest account of the **gap between idealized and realistic performance**.

---

## Key Results

| Experiment | Result |
|---|---|
| Early (leaky) model | ~100% accuracy, caused by metadata leakage (behavior names in a description column) |
| Leaky model with metadata shuffled | ~62.8% |
| **Leakage-free, cross-validated accuracy** | **95.4%** |
| **Leakage-free, held-out test accuracy** | **96.0%** |
| Ground-truth features (robustness sample) | 100.0% |
| **Sensor-tracked features (radar + tracker)** | **53.1%** (chance = 50%) |
| Mean observed duration, ground truth vs tracked | 125.02 s vs 25.18 s |
| Live demo | 5,832 predictions across 18 actors |

**Why the drop under tracking?** The classifier's logic is sound (100% on clean data). The failure comes from **partial observation and track fragmentation**: the radar's shallow field of view and the tracker's broken tracks mean an actor is seen for only about one-fifth of its true lifetime. Extremes such as hard braking or sharp swerving are often never observed, so risky actors tend to be read as safe (a safety-critical false-negative bias).

---

## Pipeline

```
Scenario generation ──► Trajectory logging ──► Feature extraction ──► Leakage removal
        │                                                                   │
        │                                                      Scenario-level 80/20 split
        │                                                                   │
        │                                                    Fine decision tree (fitctree)
        │                                                                   │
        └──► Radar + EKF tracker ──► Track-to-actor matching ──► Robustness test
                                                                            │
                                                          Live classification-driven control
```

Each stage reads named files and writes named files (idempotent hand-off), so any stage can be re-run in isolation.

| Stage | Principal script(s) | Output |
|---|---|---|
| Generate scenarios | `main_generate_100_scenarios.m` (+ batch drivers) | `scenario_NNN.mat`, `scenario_NNN_labels.csv` |
| Log trajectories | `runAndRecordScenario.m`, `getGroundTruthActorLog.m` | per-actor time-stamped logs |
| Extract features | `build_feature_dataset.m` → `extractTrackFeatures.m` | `features_dataset.csv` |
| Remove leakage | `build_clean_feature_dataset.m` | clean table (10 features + label) |
| Split by scenario | `split_train_test_by_scenario.m` | `features_train.csv`, `features_test.csv` |
| Train and evaluate | `fitctree` (Classification Learner session included) | `trainedRiskClassifier_clean.mat` |
| Sense and track | `demoSensorTracking.m`, `buildTrackedFeatureDataset.m` | tracked feature rows |
| Match and re-label | `matchTracksToActors.m` | `features_tracked_noisy.csv` |
| Robustness test | `main_step4_tracked_robustness_test.m` | ground-truth vs tracked accuracy |
| Live demo | `main_run_live_demo.m` → `runLiveClassifiedDemo.m` | `live_demo_predictions_log.csv` |

---

## Dataset

The dataset is **entirely synthetic**, generated in MATLAB, with exact ground-truth labels by construction.

| Quantity | Value |
|---|---|
| Scenarios | 450 |
| Road-network types | 7 (highway, rural, village, crossroad, multi-turn, rail crossing, mixed) |
| Complexity tiers | 3 (basic, intermediate, complex) |
| Labelled actors | 3,615 |
| Actor types | car, truck, bus, bicycle, pedestrian, child, train |
| Valid feature rows (after 4-sample guard) | 3,464 |
| Clean rows after removing leaky columns | 3,360 × 11 columns |
| Train/test split | scenario-level, 80/20, seed 42 |
| Classes | Safe (0), Risky (1) |

**Generation batches**

| Scenario IDs | Batch | Situations introduced |
|---|---|---|
| 1 – 100 | Baseline | Highway, rural, village, mixed roads; scripted safe/risky maneuvers |
| 101 – 200 | Baseline extended | Signalized crossroads, railway level crossings |
| 201 – 250 | Reactive | Closed-loop car following (IDM), live traffic signals, pedestrian crossings |
| 251 – 350 | Complex reactive | 2–3 crossroads, multi-lane traffic, lane changes, turns, U-turns |
| 351 – 450 | Complex routed | Long multi-intersection routes with turns, obstacles, lane changes |

Per scenario the repository stores a `.mat` scenario file, a `_labels.csv` (ground-truth labels) and a `_trajectories.csv` (logged actor motion).

---

## Motion Features

Every actor trajectory is reduced to the same ten features. No feature encodes actor type, scenario, or behavior name.

| # | Feature | Description | Unit |
|---|---|---|---|
| 1 | `avg_speed` | Mean speed | m/s |
| 2 | `max_speed` | Maximum speed | m/s |
| 3 | `speed_std` | Speed variability | m/s |
| 4 | `max_accel` | Largest positive acceleration | m/s² |
| 5 | `max_decel` | Hardest braking | m/s² |
| 6 | `max_lateral_rate` | Peak lateral velocity | m/s |
| 7 | `yaw_rate_std` | Variability of heading change | rad/s |
| 8 | `min_dist_to_ego` | Closest approach to ego | m |
| 9 | `avg_dist_to_ego` | Mean distance to ego | m |
| 10 | `duration` | Observed trajectory length | s |

The **same extractor** (`extractTrackFeatures.m`) is used for ground-truth and sensor-tracked paths, so any accuracy gap comes from the data, not the code. Tracks with fewer than four samples are rejected (abstention).

---

## Behavior Module Library

23 modules (15 risky, 8 safe), each with a fixed ground-truth label.

**Risky:** `aggressive_lane_change`, `sudden_braking`, `tailgating`, `unsafe_overtake`, `speeding_zigzag`, `child_chasing_ball`, `jaywalking_pedestrian`, `bus_sudden_stop`, `truck_merging`, `animal_or_obstacle_crossing`, `running_red_light`, `failure_to_yield`, `pedestrian_jaywalk_intersection`, `railway_dash_crossing`, `pedestrian_railway_dash`

**Safe:** `safe_lane_change`, `safe_following`, `safe_crosswalk_pedestrian`, `safe_cyclist`, `safe_bus_stop`, `safe_intersection_stop`, `safe_pedestrian_crosswalk_intersection`, `safe_railway_wait`

---

## Sensing and Tracking Setup

| Component | Parameter | Value |
|---|---|---|
| Radar | Field of view (azimuth × elevation) | 140° × 5° |
| Radar | Range | 0 – 150 m |
| Radar | Update rate | 10 Hz |
| Radar | Mounting | Front of ego |
| Radar | Measurement noise | Enabled |
| Tracker | Filter | Constant-velocity EKF (`initcvekf`) |
| Tracker | Assignment threshold | 35 |
| Tracker | Confirmation | 2 of 3 detections |
| Tracker | Deletion | 5 consecutive misses |
| Tracker | State | `[x vx y vy z vz]` |

---

## Live Closed-Loop Demonstration

`runLiveClassifiedDemo.m` runs perception, inference and control on separate clocks:

- **Classification:** every 2.0 s (20 steps), each actor with at least 4 samples of history is classified.
- **Display:** redrawn every 0.5 s (5 steps). Green = safe, red = risky, yellow = not yet classified.
- **Longitudinal response:** safe time gap is 2.0 s; a risky verdict on the lead actor multiplies the desired gap by **1.8×**. Car-following uses the Intelligent Driver Model.
- **Lateral response:** the ego reacts earlier to risky actors and applies a bounded lateral offset away from them.

The demo loads the clean, leakage-free classifier (`trainedRiskClassifier_clean.mat`).

---

## Repository Structure

```
.
├── Project_Book/
│   ├── Full_Project_Book.docx
│   └── Full_Project_Book.pdf
│
├── dataset_100_scenarios/                    # generated scenarios and datasets
│   ├── dataset_summary.csv
│   ├── dataset_summary_complexroute_351_450.csv
│   ├── features_clean_no_leakage.csv
│   ├── features_clean_train.csv
│   ├── features_clean_test.csv
│   ├── features_groundtruth_recomputed.csv
│   ├── features_tracked_noisy.csv
│   └── scenario_NNN.mat / _labels.csv / _trajectories.csv   (NNN = 001 … 450)
│
├── Scenario generation
│   ├── generateScenario.m
│   ├── generateReactiveScenario.m
│   ├── generateComplexReactiveScenario.m
│   ├── generateComplexRouteScenario.m
│   ├── generateLiveDemoScenario.m
│   ├── createRoadNetwork.m
│   ├── createMultiCrossroadNetwork.m
│   ├── complexScenarioLib.m
│   ├── complexityLibrary.m
│   ├── reactiveCarFollowingLib.m
│   ├── buildEgoRoute.m / buildChainedEgoRoute.m
│   ├── bakeEgoTrajectoryFromLog.m / bakeReactiveTrajectoryForDesigner.m
│   ├── findPathCrossingsAtX.m
│   ├── pathCumulativeDistance.m / pathPointAtDistance.m
│   └── runAndRecord*.m, runComplexRouteScenario.m, runEgoFollowingScenario.m
│
├── Batch drivers
│   ├── main_generate_100_scenarios.m
│   ├── main_generate_next_100_scenarios.m
│   ├── main_generate_50_reactive_scenarios.m
│   ├── main_generate_100_complex_reactive_scenarios.m
│   ├── main_generate_100_complex_route_scenarios.m
│   └── export_batch2_trajectories.m
│
├── Feature extraction and dataset building
│   ├── extractTrackFeatures.m
│   ├── build_feature_dataset.m
│   ├── build_clean_feature_dataset.m
│   ├── buildTrackedFeatureDataset.m
│   ├── getGroundTruthActorLog.m
│   └── split_train_test_by_scenario.m
│
├── Classifier
│   ├── ClassificationLearnerSession.mat
│   ├── trainedRiskClassifier.mat             # early model (leaky)
│   └── trainedRiskClassifier_clean.mat       # leakage-free model
│
├── Sensing, tracking and robustness
│   ├── demoSensorTracking.m
│   ├── matchTracksToActors.m
│   ├── diagnose_radar_detections.m
│   └── main_step4_tracked_robustness_test.m
│
├── Live demonstration
│   ├── main_run_live_demo.m
│   ├── runLiveClassifiedDemo.m
│   ├── live_risk_visualization.m
│   ├── live_demo_predictions_log.csv
│   └── live_demo_final_summary.csv
│
├── Visualization and diagnostics
│   ├── visualizeScenario.m
│   ├── visualizeReactiveScenario.m
│   ├── visualizeComplexReactiveScenario.m
│   └── diagnose_csv_issues.m
│
├── features_dataset.csv / features_train.csv / features_test.csv
├── LICENSE
└── README.md
```

---

## Requirements

- **MATLAB** (with Simulink)
- **Automated Driving Toolbox:** `drivingScenario`, `drivingRadarDataGenerator`
- **Sensor Fusion and Tracking Toolbox:** `multiObjectTracker`, `initcvekf`
- **Statistics and Machine Learning Toolbox:** `fitctree`, Classification Learner

---

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/Latif1218/Intelligent-Object-Monitoring-System-for-Autonomous-Vehicles-Safety-Enhancing.git
cd Intelligent-Object-Monitoring-System-for-Autonomous-Vehicles-Safety-Enhancing
```

### 2. Open MATLAB in the project folder

```matlab
addpath(genpath(pwd))
```

### 3. Run the pipeline

The scripts follow the build chain above. A typical order:

```matlab
% 1) Generate scenarios (re-run the other batch drivers for IDs 101-450)
main_generate_100_scenarios

% 2) Build the feature table and remove leaky metadata columns
build_feature_dataset
build_clean_feature_dataset

% 3) Scenario-level train/test split (seed 42)
split_train_test_by_scenario

% 4) Train the fine decision tree on the 10 motion features
%    (use Classification Learner, or fitctree directly), then save the model
%    as trainedRiskClassifier_clean.mat

% 5) Robustness test: radar + tracker vs ground truth
main_step4_tracked_robustness_test

% 6) Live closed-loop demonstration
main_run_live_demo
```

> The pre-generated scenarios, feature CSVs and trained models are already included, so you can start from any stage (for example, jump straight to step 5 or 6).

---

## Reproducibility

- Every random step (actor placement, behavior assignment, train/test split) is **seeded** (`seed = 42`), so re-running reproduces the same dataset, split and accuracy.
- The train/test split is done over unique `ScenarioID`s, so actors from one scenario never straddle the train/test boundary.
- Stages exchange data through files and overwrite rather than append, so re-runs never accumulate stale rows.

---

## Limitations and Future Work

**Limitations**

- Single forward-looking radar: actors behind or beside the ego (for example during turns) are invisible.
- Duration-dependent features (extremes and variances) are fragile under truncated observation.
- The data is synthetic, and the robustness test uses a small mixed sample of ten scenarios (32 tracked rows).

**Future work**

- **Multi-sensor fusion** (radar + camera/lidar, wider coverage)
- **Duration-independent feature design**
- **Sequence models** over partial tracks
- **Track stitching and re-identification**
- **Real-vehicle validation**

---

## License

See the [LICENSE](LICENSE) file for details.
