## SORT (Simple Online and Realtime Tracking)

This repo contains:
- **A modern websocket “bridge”** that assigns stable track IDs to per-frame CV detections and re-broadcasts the streams.
- **Image-space and world-space** tracking modes built around the SORT idea.
- A **legacy MOTChallenge demo** (kept for reference, not used by the bridge).

## How SORT works (high level)

SORT is a minimal multi-object tracker that combines:
- **A motion model** (Kalman Filter per track) and
- **A data association step** (Hungarian assignment) to match detections ↔ tracks each frame.

### Per-frame loop

For each video frame:
- **1) Predict**: each track’s Kalman Filter predicts the next state (where the object should be now).
- **2) Associate**: compute a cost between each detection and each predicted track.
  - Classic SORT uses **IOU** between 2D boxes as the matching signal.
  - Solve the assignment with the **Hungarian algorithm** (min-cost matching).
  - Apply **gating** (e.g., minimum IOU) so bad pairs don’t match.
- **3) Update**:
  - matched tracks run a **Kalman update** with the matched detection
  - unmatched detections spawn **new tracks**
  - unmatched tracks age; tracks exceeding `max_age` are removed
- **4) Confirm tracks**: only output IDs after a track has enough consecutive hits (`min_hits`).

## What this repo is used for (CV detections over websockets)

We are **not** using MOTChallenge evaluation in day-to-day work here.
Instead, we use this repository to:
- consume a video stream with detection metadata (per frame),
- run a tracker to turn “frame detections” into “stable object IDs”, and
- re-broadcast the same stream with stable `track_id` on each bbox and `obj_id` preserved from upstream (original detection ID).

The entrypoint is `sort-3d.py` which runs `sort_ws/bridge.py`.

## Installation

### Python + venv

This repo expects you to use a local virtual environment in `venv/`.

```bash
cd /path/to/sort
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Notes on dependencies
- **`lap`** is used for fast Hungarian assignment; if it can’t import, the code falls back to SciPy’s `linear_sum_assignment`.
- Core runtime deps for the bridge/tracker are already captured in `requirements.txt` (notably `websockets`, `numpy`, `scipy`, `filterpy`).

## Running the websocket bridge

### Upstream / downstream ports

The bridge connects to upstream websocket servers (typically a replay process) and re-broadcasts downstream:
- **Upstream video**: `ws://127.0.0.1:5001` (binary payload: `[4-byte len][json][jpeg]`)
- **Upstream NMEA**: `ws://127.0.0.1:3636` (JSON text)
- **Upstream control**: `ws://127.0.0.1:6001` (JSON/text)

Downstream defaults:
- **video**: `ws://0.0.0.0:5002`
- **nmea**: `ws://0.0.0.0:3637`
- **control**: `ws://0.0.0.0:6002`

### Enter venv

You can either:
- activate the venv: `source venv/bin/activate`, then run `python ...`, or
- call the interpreter directly: `venv/bin/python ...`

### Image-space mode

Image-space mode tracks directly in pixel coordinates using IOU (plus optional extra terms).

```bash
venv/bin/python sort-3d.py \
  --mode image_space \
  --image-space-max-age 40 \
  --image-space-min-hits 300 \
  --image-space-iou-threshold 0.10
```

**Detection input (per bbox)**: expects dicts like:
- **required**: `x`, `y`, `width`, `height` (pixels)
- **optional**: `confidence`, `distance`, `heading`, `category`

**Output behavior**:
- both matched and unmatched confirmed tracks are output every frame (unconfirmed detections are dropped)
- each bbox has `track_id` (stable tracker ID) and, for matched bboxes, `obj_id` (original upstream detection ID)
- outgoing metadata includes `tracked_space="image_space"` and `tracked_status` (`"matched"` or `"unmatched"`)

### World-space mode

World-space mode uses ego NMEA (lat/lon/heading) + camera parameters to project each detection into **local ENU meters**, then tracks in that world space.

#### Why local ENU meters (not absolute lat/lon) for KF + Hungarian?

We track in a local tangent plane (ENU = East/North in meters) instead of directly in latitude/longitude because:
- **Linear units**: Kalman Filters assume a (locally) linear state and measurement model. ENU meters are approximately linear over small areas; lat/lon are angular degrees with non-uniform scaling (longitude depends on cos(latitude)).
- **Meaningful distances/costs**: Hungarian association needs a cost with consistent units. Euclidean distance in ENU meters is meaningful; “distance” in degrees is not (and varies with latitude).
- **Numerical conditioning**: meter-scale values (tens/hundreds) are better conditioned than tiny degree deltas (~1e-5) and avoid mixing lat/lon scales.

#### Current limitations:
- **Current local ENU `ref_latlon` approach loses accuracy as journey range increases**: our ENU frame is anchored to a single fixed `ref_latlon` (the first ego GPS fix). This “flat plane” (equirectangular) approximation is typically accurate over **small/local regions (rule of thumb: up to a few km)**; we do **not** currently re-anchor it as the boat travels, so accuracy can degrade over longer distances.
- **Why we need the first ego fix before outputting world-space detections**: projecting a detection into world space requires a reference pose (ego **lat/lon** + **heading**) to convert pixel bearing + range into ENU meters. Until we receive the first valid ego state, the bridge cannot compute `world_east_m/world_north_m`, so in `--mode world_space` it drops bboxes rather than emitting incorrect world coordinates/IDs.

#### Where the ENU conversion happens

- **Detection → ENU (relative meters)**: `sort_ws/image_to_world.py`:
  - `pixel_x_to_bearing_rad()` and `camera_relative_xy_m()` convert pixel x + range to (right_m, forward_m)
  - `right_fwd_to_enu_m()` converts (right/forward) into ENU using ego heading
  - `detection_to_world_latlon()` returns:
    - `enu_rel_ego` (meters relative to ego), and
    - `enu_from_ref` (meters relative to a fixed local reference point)
- **Bridge populates ENU fields used by the world tracker**: `sort_ws/bridge.py` in `_track_world_space()`:
  - writes `world_east_m` / `world_north_m` into each detection dict
  - after association, enriches every output track with `enu_ref_lat` / `enu_ref_lon` and full KF kinematic state (`vel_east_mps`, `vel_north_mps`, `speed_mps`, `course_deg`, `accel_east_mps2`, `accel_north_mps2`)
- **World tracker consumes ENU meters and outputs full kinematic state**: `sort_ws/world_space_tracker.py`:
  - `WorldSpaceSort` reads `world_east_m` / `world_north_m` and associates tracks using meter distances
  - `assign()` returns a 3-tuple: `(assigned_ids, unmatched_tracks, matched_track_states)` where `matched_track_states` maps each matched track ID to its full KF state (position, velocity, acceleration, speed, course)
  - Each KF tracker (`KalmanCVPointTracker`, `KalmanCAPointTracker`) exposes `get_full_state()` returning the complete kinematic state as a dict

```bash
venv/bin/python sort-3d.py \
  --mode world_space \
  --world-space-cv-width-px 1920 \
  --world-space-fov-x-deg 55.3 \
  --world-space-camera-yaw-offset-deg 0.0 \
  --world-space-max-distance-m 50.0
```

**Requirements**:
- upstream NMEA stream must provide ego **Latitude/Longitude/Heading**
- video metadata bboxes must include `distance` (meters) and pixel `x/width` so the bridge can compute bearing

**Output behavior**:
- both matched and unmatched confirmed tracks are output every frame
- each bbox has `track_id` (stable tracker ID) and, for matched bboxes, `obj_id` (original upstream detection ID)
- world fields are added (e.g. `world_latitude/world_longitude`, `world_east_m/world_north_m`, etc.); unmatched tracks get back-projected bbox fields for AR view
- outgoing metadata includes `tracked_space="world_space"` and `tracked_status` (`"matched"` or `"unmatched"`)
- every track includes the **ENU reference point** (`enu_ref_lat`, `enu_ref_lon`) — the fixed lat/lon origin of the ENU plane (first ego GPS fix), enabling downstream consumers to convert ENU positions/velocities without a separate handshake
- every track includes **full Kalman Filter kinematic state**: velocity (`vel_east_mps`, `vel_north_mps`), derived speed and course (`speed_mps`, `course_deg`), and acceleration (`accel_east_mps2`, `accel_north_mps2`). Acceleration is `null` for the CV (constant-velocity) model and populated for the CA (constant-acceleration) model

### Tracker output structure

Both trackers now output **matched** and **unmatched** confirmed tracks every frame. Unmatched tracks are confirmed tracks that weren't associated with a detection this frame but are still within `max_age`. This allows downstream consumers (e.g., lookout-V2 AR view) to continue displaying tracks even when detections are temporarily missing.

#### Output mode selection logic

The `--mode` parameter controls which tracker's output is used:

- **`image_space`**: Only outputs image-space tracks (matched + unmatched). World-space tracker still runs internally but its output is discarded.
- **`world_space`**: Only outputs world-space tracks (matched + unmatched). May output empty list until first ego fix arrives.
- **`auto`** (default): If world-space tracks are non-empty, outputs only world-space tracks; otherwise outputs only image-space tracks. This provides a seamless fallback when ego data is unavailable.

#### Output fields by tracker and match status

Each output bbox includes a `tracked_status` field:
- `"matched"` — track was associated with a detection this frame
- `"unmatched"` — confirmed track with no detection this frame (using predicted/stored values)

#### Track output decision tree

The tracker uses two counters to decide whether to output a track:

- **`hits`**: total lifetime matches (cumulative, never resets)
- **`hit_streak`**: consecutive matches (resets to 0 on any missed frame)

```
                        Track exists
                             |
            +----------------+----------------+
            |                                 |
    Detection matched?                 No detection matched
            |                                 |
   hit_streak >= min_hits?            hits >= min_hits?
            |                                 |
      +-----+-----+                     +-----+-----+
      |           |                     |           |
     YES         NO                    YES         NO
      |           |                     |           |
   OUTPUT    hits > min_hits?        OUTPUT      NOT
   MATCHED        |                  UNMATCHED   OUTPUT
            +-----+-----+            (ghost)
            |           |
           YES         NO
            |           |
         OUTPUT       NOT
        UNMATCHED    OUTPUT
        (re-warm)   (warming)
```

**State definitions:**

| State | Condition | Meaning |
|-------|-----------|---------|
| **Matched** | `hit_streak >= min_hits` | Track has enough consecutive matches to be fully confirmed |
| **Re-warming** | `hit_streak < min_hits` but `hits > min_hits` | Track was previously confirmed but lost consecutive matches; still outputs as unmatched using detection position |
| **Unmatched (ghost)** | No detection this frame, but `hits >= min_hits` | Confirmed track with no detection; outputs using Kalman prediction |
| **Warming** | `hits < min_hits` | New track building confidence; not yet output |

**Position and kinematic source by state:**

| Track State | Position Source | Velocity / Acceleration Source |
|-------------|-----------------|-------------------------------|
| Matched | Detection position | KF state (post-update) |
| Re-warming | Detection position (not Kalman prediction) | KF state (post-update) |
| Unmatched (ghost) | Kalman filter prediction | KF state (post-predict) |

**Why re-warming uses detection position:**

When a previously-confirmed track matches a detection but hasn't yet rebuilt its `hit_streak` to `min_hits`, we use the detection position rather than the Kalman prediction. This is because:
1. We have a fresh detection — use it
2. The Kalman prediction may have drifted during the missed frames
3. Re-warming tracks are output as `"unmatched"` status so downstream knows they're not fully re-confirmed yet

#### Track lifecycle and deletion

A track is created when an unmatched detection spawns a new tracker. The track uses a third counter:

- **`time_since_update`**: frames since last matched detection (resets to 0 on match, increments on each predict)

**Track deletion rule:** A track is permanently deleted when `time_since_update > max_age`.

```
Track created (unmatched detection)
         |
         v
    [Warming phase: hits < min_hits]
         |
    hit_streak >= min_hits?
         |
    +----+----+
    |         |
   YES       NO (missed frames)
    |         |
    v         v
 [Confirmed] time_since_update > max_age?
    |              |
    |         +----+----+
    |         |         |
    |        NO        YES
    |         |         |
    |    [Continue]  [DELETED]
    |    (as ghost)
    v
 [Outputs as matched/unmatched/ghost]
```

**Summary: when is a track NOT output?**

| Condition | Result |
|-----------|--------|
| `hits < min_hits` (warming) | Not output — still building confidence |
| `time_since_update > max_age` | Track deleted — gone forever |

**Summary: when IS a track output?**

Once `hits >= min_hits` (i.e., the track was confirmed at some point), the track will be output every frame until it's deleted. The output status depends on the current frame:
- **Matched**: detection matched AND `hit_streak >= min_hits`
- **Unmatched (re-warming)**: detection matched BUT `hit_streak < min_hits`
- **Unmatched (ghost)**: no detection matched this frame

##### 1. Image-space tracker, matched

All original detection fields are preserved, plus tracker metadata:

| Field | Source |
|-------|--------|
| `x`, `y`, `width`, `height` | From detection (pixels) |
| `confidence`, `distance`, `heading`, `category` | From detection |
| `obj_id` | From detection (original upstream ID) |
| `track_id` | Stable tracker ID (1-based) |
| `tracked_space` | `"image_space"` |
| `tracked_status` | `"matched"` |

##### 2. Image-space tracker, unmatched

Uses Kalman-predicted bbox and last known metadata:

| Field | Source |
|-------|--------|
| `x`, `y`, `width`, `height` | Kalman-predicted bbox (pixels) |
| `confidence`, `distance`, `heading`, `category` | Last known values from most recent match |
| `track_id` | Stable tracker ID (1-based) |
| `tracked_space` | `"image_space"` |
| `tracked_status` | `"unmatched"` |

##### 3. World-space tracker, matched

All original detection fields plus computed world fields and KF kinematic state:

| Field | Source |
|-------|--------|
| `x`, `y`, `width`, `height` | From detection (pixels) |
| `confidence`, `distance`, `heading`, `category` | From detection |
| `obj_id` | From detection (original upstream ID) |
| `world_latitude`, `world_longitude` | Computed from ego + bearing + distance |
| `world_east_m`, `world_north_m` | ENU meters from local reference |
| `world_rel_ego_east_m`, `world_rel_ego_north_m` | ENU meters relative to current ego |
| `enu_ref_lat`, `enu_ref_lon` | Fixed ENU origin (first ego GPS fix, degrees) |
| `vel_east_mps`, `vel_north_mps` | KF velocity in ENU (m/s) |
| `speed_mps` | Derived speed: `hypot(vel_east, vel_north)` (m/s) |
| `course_deg` | Course over ground: `atan2(vel_east, vel_north)` (0=N, 90=E, degrees) |
| `accel_east_mps2`, `accel_north_mps2` | KF acceleration in ENU (m/s²); `null` for CV model |
| `track_id` | Stable tracker ID (1-based) |
| `tracked_space` | `"world_space"` |
| `tracked_status` | `"matched"` |

##### 4. World-space tracker, unmatched

Tracks with `tracked_status="unmatched"` include two sub-cases with different position sources:

**4a. Ghost tracks (no detection this frame)**

Uses Kalman-predicted world position + stored bbox geometry + full KF kinematic state:

| Field | Source |
|-------|--------|
| `world_east_m`, `world_north_m` | **Kalman-predicted** position (ENU meters) |
| `x` | Back-projected from predicted world position |
| `y`, `width`, `height` | **Stored** from most recent matched detection |
| `distance` | Back-projected from predicted world position |
| `world_latitude`, `world_longitude` | Converted from predicted ENU |
| `world_rel_ego_east_m`, `world_rel_ego_north_m` | Recomputed relative to current ego |
| `enu_ref_lat`, `enu_ref_lon` | Fixed ENU origin (first ego GPS fix, degrees) |
| `vel_east_mps`, `vel_north_mps` | KF velocity in ENU (m/s) |
| `speed_mps` | Derived speed (m/s) |
| `course_deg` | Course over ground (degrees) |
| `accel_east_mps2`, `accel_north_mps2` | KF acceleration in ENU (m/s²); `null` for CV model |
| `confidence`, `heading`, `category` | Last known values |
| `track_id` | Stable tracker ID (1-based) |
| `tracked_space` | `"world_space"` |
| `tracked_status` | `"unmatched"` |

**4b. Re-warming tracks (detection matched, but hit_streak < min_hits)**

**Passes through the full detection directly** (no back-projection needed) plus KF kinematic state:

| Field | Source |
|-------|--------|
| `x`, `y`, `width`, `height` | **Directly from current detection** (exact values) |
| `distance` | **Directly from current detection** |
| `confidence`, `heading`, `category` | **Directly from current detection** |
| `obj_id` | **Directly from current detection** (preserved!) |
| `world_east_m`, `world_north_m` | From current detection |
| `world_latitude`, `world_longitude` | From current detection |
| `world_rel_ego_east_m`, `world_rel_ego_north_m` | From current detection |
| `enu_ref_lat`, `enu_ref_lon` | Fixed ENU origin (first ego GPS fix, degrees) |
| `vel_east_mps`, `vel_north_mps` | KF velocity in ENU (m/s) |
| `speed_mps` | Derived speed (m/s) |
| `course_deg` | Course over ground (degrees) |
| `accel_east_mps2`, `accel_north_mps2` | KF acceleration in ENU (m/s²); `null` for CV model |
| `track_id` | Stable tracker ID (1-based) |
| `tracked_space` | `"world_space"` |
| `tracked_status` | `"unmatched"` |

**Why re-warming tracks pass through detection directly**: Re-warming tracks have a matched detection this frame — they're only "unmatched" in terms of not having enough consecutive hits yet. Since we have fresh detection data, we use it directly instead of back-projecting from world position. This preserves all original fields including `obj_id`, avoids round-trip precision loss, and uses current confidence/heading/category values.

#### Why compute artificial bbox for ghost tracks?

Ghost tracks (true unmatched — no detection this frame) need artificial bbox computation because the world tracker only maintains ENU position, not pixel coordinates. The lookout-V2 AR view needs bbox coordinates to display overlays synchronized with the Aerial View.

For **ghost tracks only**, we back-project world position to image space:
1. `world_to_image.py::world_to_image_space()` computes `x_center_px` and `distance` from the Kalman-predicted world position
2. Stored `y`, `width`, `height` from the last matched detection are reused

For **re-warming tracks**, no back-projection is needed — we pass through the detection data directly.

This allows the AR view to continue showing a bbox overlay for the track even when the detector completely misses the object (ghost tracks).

### Parameters (what they mean)

#### Image-space tracker parameters
- **`--image-space-max-age`**: frames a track can miss before being deleted
- **`--image-space-min-hits`**: consecutive matched frames required before emitting an ID
- **`--image-space-iou-threshold`**: minimum IOU to allow a detection↔track match
- **`--image-space-alpha-distance`**: weight for distance-difference cost term (if `distance` present)
- **`--image-space-beta-heading`**: weight for heading-difference cost term (if `heading` present)
- **`--image-space-gamma-confidence`**: weight for confidence penalty term (prefers high-confidence detections)
- **`--image-space-new-track-min-confidence`**: minimum confidence to spawn a new track

#### World-space camera parameters
- **`--world-space-cv-width-px`**: image width in pixels (used to convert x-pixel → bearing)
- **`--world-space-fov-x-deg`**: horizontal field-of-view in degrees
- **`--world-space-camera-yaw-offset-deg`**: yaw offset between camera forward and boat forward

#### World-space tracker parameters
- **`--world-space-max-age`**: frames a track can miss before being deleted
- **`--world-space-min-hits`**: consecutive matched frames required before emitting an ID
- **`--world-space-max-distance-m`**: max ENU distance allowed for association (gating)
- **`--world-space-beta-heading`**: weight for heading-difference cost term (if target heading exists)
- **`--world-space-gamma-confidence`**: weight for confidence penalty term
- **`--world-space-new-track-min-confidence`**: minimum confidence to spawn a new track

#### Per-category kinematic caps

These cap the KF state to physically reasonable values, preventing tracks from "flying away" during occlusions or noisy detections. Clamping is applied after every KF predict step (not after update), preserving direction while capping magnitude.

The clamp is always active. **0** = clamp to zero (e.g., force stationary), **positive** = cap at that value. Set a large value (e.g. `50` for speed, `20` for acceleration) if you effectively don't want a constraint.

- **`--world-space-max-speed-boat-mps`**: max speed for `boat` category tracks (m/s). Velocity vector magnitude is clamped to this value. Default: `50`.
- **`--world-space-max-speed-other-mps`**: max speed for non-boat category tracks (m/s). Default: `50`.
- **`--world-space-max-accel-boat-mps2`**: max acceleration for `boat` category tracks (m/s²). Only effective with the CA model. Default: `20`.
- **`--world-space-max-accel-other-mps2`**: max acceleration for non-boat category tracks (m/s²). Only effective with the CA model. Default: `20`.

## File guide (what each file does)

### Websocket bridge + pipeline
- **`sort-3d.py`**: entrypoint; calls `sort_ws.bridge.cli_main()`.
- **`sort_ws/bridge.py`**: connects to upstream websockets, runs tracking, re-broadcasts downstream.
- **`sort_ws/codec.py`**: encodes/decodes the binary video websocket payload: `[len][json][jpeg]`.

### Tracking + transforms
- **`sort_ws/image_space_tracker.py`**: image-space SORT-style tracker (`ImageSpaceSort`) + IOU association + Hungarian assignment (with optional extra cost terms).
- **`sort_ws/image_to_world.py`**: math utilities to project pixels + distance into ENU meters and back to lat/lon.
- **`sort_ws/ego.py`**: ego state smoothing and storage; consumes NMEA JSON, maintains a smoothed `EgoState` (lat/lon/heading and local ENU).
- **`sort_ws/world_space_tracker.py`**: world-space SORT-style tracker (`WorldSpaceSort`) that tracks points in ENU meters. Supports constant-velocity (CV) and constant-acceleration (CA) Kalman Filter models. Each tracker exposes `get_full_state()` returning ENU position, velocity, acceleration, speed, and course.

## Legacy / unused: MOTChallenge demo (not used by the bridge)

The original upstream SORT demo is preserved under `sort_mot/`:
- **`sort_mot/sort.py`**: the original demo that reads MOT-format detections from `data/` and can optionally display results.

Run the legacy demo (uses `data/train/*/det/det.txt`):

```bash
venv/bin/python sort_mot/sort.py
```

To display results, you must also have the MOT images available and symlinked to `mot_benchmark/` as expected by the script:

```bash
ln -s /path/to/MOT2015_challenge/data/2DMOT2015 mot_benchmark
venv/bin/python sort_mot/sort.py --display
```

## License

SORT is released under the GPL License (refer to `LICENSE`).

## Citing SORT

If you find SORT useful, please consider citing:

```
@inproceedings{Bewley2016_sort,
  author={Bewley, Alex and Ge, Zongyuan and Ott, Lionel and Ramos, Fabio and Upcroft, Ben},
  booktitle={2016 IEEE International Conference on Image Processing (ICIP)},
  title={Simple online and realtime tracking},
  year={2016},
  pages={3464-3468},
  keywords={Benchmark testing;Complexity theory;Detectors;Kalman filters;Target tracking;Visualization;Computer Vision;Data Association;Detection;Multiple Object Tracking},
  doi={10.1109/ICIP.2016.7533003}
}
```

## See also

- [DeepSORT](https://github.com/nwojke/deep_sort)
- [Local Metrics for Multi-Object Tracking](https://github.com/google-research/localmot)
