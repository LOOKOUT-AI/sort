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
- **World tracker consumes ENU meters**: `sort_ws/world_space_tracker.py`:
  - `WorldSpaceSort` reads `world_east_m` / `world_north_m` and associates tracks using meter distances

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

All original detection fields plus computed world fields:

| Field | Source |
|-------|--------|
| `x`, `y`, `width`, `height` | From detection (pixels) |
| `confidence`, `distance`, `heading`, `category` | From detection |
| `obj_id` | From detection (original upstream ID) |
| `world_latitude`, `world_longitude` | Computed from ego + bearing + distance |
| `world_east_m`, `world_north_m` | ENU meters from local reference |
| `world_rel_ego_east_m`, `world_rel_ego_north_m` | ENU meters relative to current ego |
| `track_id` | Stable tracker ID (1-based) |
| `tracked_space` | `"world_space"` |
| `tracked_status` | `"matched"` |

##### 4. World-space tracker, unmatched

Uses Kalman-predicted world position, with **artificially computed bbox fields** for AR view synchronization:

| Field | Source |
|-------|--------|
| `x` | Back-projected from world position (`x_center - width/2`) |
| `y` | Stored last known value from most recent match |
| `width`, `height` | Stored last known values from most recent match |
| `distance` | Back-projected from world position |
| `confidence`, `heading`, `category` | Last known values from most recent match |
| `world_latitude`, `world_longitude` | Converted from predicted ENU |
| `world_east_m`, `world_north_m` | Kalman-predicted position (ENU meters) |
| `world_rel_ego_east_m`, `world_rel_ego_north_m` | Recomputed relative to current ego |
| `track_id` | Stable tracker ID (1-based) |
| `tracked_space` | `"world_space"` |
| `tracked_status` | `"unmatched"` |

#### Why compute artificial bbox for unmatched world-space tracks?

The lookout-V2 frontend AR view needs bbox coordinates to display overlays synchronized with the Aerial View. When a world-space track goes unmatched (no detection this frame), we back-project the Kalman-predicted world position to image space using:

1. `world_to_image.py::world_to_image_space()` computes `x_center_px` and `distance` from the predicted world position
2. Stored `y`, `width`, `height` from the last matched detection are reused (vertical position and size don't change much frame-to-frame)

This allows the AR view to continue showing a bbox overlay for the track even when the detector misses the object.

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

## File guide (what each file does)

### Websocket bridge + pipeline
- **`sort-3d.py`**: entrypoint; calls `sort_ws.bridge.cli_main()`.
- **`sort_ws/bridge.py`**: connects to upstream websockets, runs tracking, re-broadcasts downstream.
- **`sort_ws/codec.py`**: encodes/decodes the binary video websocket payload: `[len][json][jpeg]`.

### Tracking + transforms
- **`sort_ws/image_space_tracker.py`**: image-space SORT-style tracker (`ImageSpaceSort`) + IOU association + Hungarian assignment (with optional extra cost terms).
- **`sort_ws/image_to_world.py`**: math utilities to project pixels + distance into ENU meters and back to lat/lon.
- **`sort_ws/ego.py`**: ego state smoothing and storage; consumes NMEA JSON, maintains a smoothed `EgoState` (lat/lon/heading and local ENU).
- **`sort_ws/world_space_tracker.py`**: world-space SORT-style tracker (`WorldSpaceSort`) that tracks points in ENU meters.

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
