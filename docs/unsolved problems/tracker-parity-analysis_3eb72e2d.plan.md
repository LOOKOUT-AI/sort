---
name: tracker-parity-analysis
overview: Analyze whether image-space and world-space trackers share the same lifecycle logic, enumerate every meaningful difference beyond IOU vs distance gating, and outline the lowest-risk path to increase parity if desired.
todos:
  - id: map-lifecycle-parity
    content: "Document the shared lifecycle rules and call out the only true transition-code mismatch: image space lacks an explicit `update_rewarming()` path."
    status: pending
  - id: separate-mode-specific-differences
    content: Split differences into association/gating, KF dynamics, bridge preprocessing, runtime defaults, and emitted payload schema so parity work stays focused.
    status: pending
  - id: define-parity-refactor
    content: If implementation is desired later, unify lifecycle decisions behind shared helper logic while keeping IOU vs metric-distance association mode-specific.
    status: pending
isProject: false
---

# Tracker Parity Analysis

## Bottom Line

The two trackers already share the same high-level lifecycle skeleton for `no track -> warming -> matched -> ghost -> rewarming -> matched/deleted`:

- New unmatched detections can spawn trackers if they survive the bridge-level `input_min_confidence` filter.
- A track becomes externally `matched` only when `hit_streak >= min_hits`.
- A confirmed track becomes `ghost` after one missed frame.
- A confirmed ghost becomes `rewarming` after a recovery match when the rebuilt streak is still `< min_hits`.
- Any track is deleted when `time_since_update > max_age`.

The biggest differences are not only `IOU` vs `max_distance_m`. They also differ in motion model richness, association inputs, rewarming implementation, upstream preprocessing, emitted state, and runtime tunability.

## Key Files To Compare

- `[/Users/stevekx/Desktop/github-projects/sort/sort_ws/image_space_tracker.py](/Users/stevekx/Desktop/github-projects/sort/sort_ws/image_space_tracker.py)`
- `[/Users/stevekx/Desktop/github-projects/sort/sort_ws/world_space_tracker.py](/Users/stevekx/Desktop/github-projects/sort/sort_ws/world_space_tracker.py)`
- `[/Users/stevekx/Desktop/github-projects/sort/sort_ws/bridge.py](/Users/stevekx/Desktop/github-projects/sort/sort_ws/bridge.py)`

## Differences To Analyze

### 1. State-transition logic

These are nearly the same today.

- `image_space_tracker.py` always calls `update()` for matched pairs, then decides `matched` vs `rewarming` from `hit_streak` and `hits`.
- `world_space_tracker.py` explicitly branches between `update()` and `update_rewarming()` before deciding outputs.
- Important nuance: `world_space_tracker.py` currently makes `update_rewarming()` behave the same as `update()` in practice, so this is mostly a structural difference, not a behavioral one.
- Both trackers share the same silent edge case: if `hits == min_hits` but `hit_streak < min_hits`, the track is neither emitted as `matched` nor as `rewarming` that frame.

### 2. Association logic

This is the largest intentional algorithmic difference.

- Image space gates on bbox overlap: `iou_matrix >= iou_threshold`.
- World space gates on spatial proximity: `dist <= max_distance_m`.
- Image space cost = `(1 - IOU) + alpha_distance * range_diff + beta_heading * heading_diff + gamma_confidence * conf_penalty`.
- World space cost = `distance + beta_heading * heading_diff + gamma_confidence * conf_penalty`.
- Image space optionally uses detection-vs-track `distance` as an extra cost term, but not as the hard gate.
- World space does not have an `alpha_distance` term because distance is already the primary cost and gate.

### 3. Motion / filter model

World space is materially richer.

- Image space uses one fixed `KalmanBoxTracker` bbox CV model with empirical pixel-space `Q`.
- World space supports `cv` and `ca` point trackers and rebuilds physics-based `Q` from real `dt`.
- World space updates measurement covariance from relative ENU position and clamps speed/acceleration by category.
- Image space has no category-based kinematic clamping and no adaptive measurement covariance.

### 4. Input preprocessing before tracking

World space has extra filtering before a detection even reaches the tracker.

- In `bridge.py`, world-space tracking drops detections if ego pose is unavailable.
- It also drops invalid detections lacking usable `x`, `width`, or positive `distance`.
- Image-space tracking sends incoming bboxes directly to `ImageSpaceSort.assign()`.
So some observed behavior differences come from bridge-side admission rules, not tracker-state logic.

### 5. Rewarming payload semantics

The status concept is aligned, but the payload source differs.

- Image-space rewarming emits detection bbox geometry directly.
- World-space rewarming keeps detection image fields but overrides world position with the tracker prediction from `trk_xy` and also emits full KF state.
- World-space ghost and matched outputs carry far more KF metadata.

### 6. Runtime parameter surface

World space is more operationally configurable.

- `WorldSpaceSort` exposes `get_tunable_params()` and `set_tunable_params()`.
- Image space does not expose a comparable runtime tuning API.

### 7. Default runtime configuration

The shipped defaults differ a lot in `bridge.py`.

- Image space: `min_hits=300`, `max_age=40`, `iou_threshold=0.1`, `alpha_distance=0.15`.
- World space: `min_hits=5`, `max_age=50`, `max_distance_m=20.0` plus KF model options.
Even if transition code were identical, these defaults make the observed behavior very different.

## Practical Conclusions

If the question is strictly "is the lifecycle logic the same?", the answer is: mostly yes.
If the question is "will they behave the same in practice?", the answer is no, for these reasons:

- Different admission filters before tracking.
- Different association/gating geometry.
- Different motion models and filter dynamics.
- Different defaults.
- Different rewarming/matched/ghost payload construction.

## Best Path To Maximize Similarity

If you want image space to be as similar as possible to world space without inventing fake world geometry:

1. Keep the unavoidable hard difference: image uses `IOU` gate, world uses `max_distance_m` gate.
2. Refactor both trackers to share one lifecycle/output-state helper for:
   - birth gating
   - matched confirmation
   - ghost eligibility
   - rewarming eligibility
   - deletion threshold
3. Add an explicit `update_rewarming()` method to image space for structural parity, even if it initially aliases `update()`.
4. Normalize rewarming/ghost payload schemas so both trackers emit the same lifecycle metadata fields whenever possible.
5. Add an image-space runtime tuning API mirroring `WorldSpaceSort` where it makes sense.
6. Decide whether image space should gain more world-like motion constraints, such as category-based velocity caps or adaptive measurement noise in pixel space.

## Recommendation

Treat parity in three layers rather than one:

- Lifecycle parity: already close; can be unified cleanly.
- Association parity: should remain mode-specific because IOU and metric distance solve different problems.
- Filter/output parity: this is where most remaining cleanup value is.

## Useful Code References

```481:554:/Users/stevekx/Desktop/github-projects/sort/sort_ws/image_space_tracker.py
        # Update matched trackers with assigned detections.
        for det_idx, trk_idx in matches:
            matched_trk_indices.add(int(trk_idx))
            self.trackers[int(trk_idx)].update(det_xyxy[int(det_idx), :], det_extras[int(det_idx)])
            trk = self.trackers[int(trk_idx)]
            track_id = int(trk.id) + 1
            
            if trk.hit_streak >= self.min_hits:
                assigned_ids[int(det_idx)] = track_id
            elif trk.hits > self.min_hits:
                rewarming_tracks.append({
                    ...
                    "unmatched_frames": 0,
                    "tracker_max_age": int(self.max_age),
                    "_is_rewarming": True,
                })
```

```828:925:/Users/stevekx/Desktop/github-projects/sort/sort_ws/world_space_tracker.py
        for det_idx, trk_idx in matches:
            ...
            was_confirmed = trk.hits >= self.min_hits
            will_recover_this_frame = (trk.hit_streak + 1) >= self.min_hits

            if was_confirmed and not will_recover_this_frame:
                trk.update_rewarming(det_xy[det_idx_i, :], det_extras[det_idx_i])
            else:
                trk.update(det_xy[det_idx_i, :], det_extras[det_idx_i])

            if trk.hit_streak >= self.min_hits:
                assigned[det_idx_i] = track_id
                matched_track_states[track_id] = trk.get_full_state()
            elif trk.hits > self.min_hits:
                rewarming_entry = dict(det)
                rewarming_entry["_is_rewarming"] = True
                rewarming_entry["unmatched_frames"] = 0
```

```520:596:/Users/stevekx/Desktop/github-projects/sort/sort_ws/world_space_tracker.py
def associate_world_detections_to_trackers(
    ...
    max_distance_m: float = 30.0,
    beta_heading: float = 0.0,
    gamma_confidence: float = 0.0,
):
    ...
    dist = _euclid_dist_matrix(det_xy, trk_xy)
    cost = dist.copy()
    ...
    gated = dist <= float(max_distance_m)
```

```103:207:/Users/stevekx/Desktop/github-projects/sort/sort_ws/image_space_tracker.py
def associate_detections_to_trackers(
    ...
    iou_threshold: float = 0.3,
    alpha_distance: float = 0.15,
    beta_heading: float = 0.10,
    gamma_confidence: float = 0.05,
):
    ...
    iou_matrix = iou_batch(dets_xyxy, trks_xyxy)
    cost = 1.0 - iou_matrix
    ...
    gated = iou_matrix >= float(iou_threshold)
```
