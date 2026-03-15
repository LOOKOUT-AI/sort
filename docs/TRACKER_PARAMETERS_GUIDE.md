# World-Space Tracker Parameters Guide

This document explains the tunable parameters for the world-space SORT tracker, how they interact, and how they affect the three types of output tracks. **Use case**: marine boat/other fusion with CV detections as the only source, where distance prediction has poor accuracy and stability.

**Spreadsheet:** The same content is available as CSV files (`TRACKER_PARAMETERS_01_parameter_reference.csv`, etc.) and as a **single Excel workbook** with one sheet per table: **`TRACKER_PARAMETERS.xlsx`**. To regenerate the workbook from the CSVs, run from the repo root: `python docs/build_tracker_params_xlsx.py` (no extra dependencies).

---

## Part 1: Parameter Reference

| Parameter | Meaning | How it affects tracker behavior | Performs well when … | Performs poorly when … | Suggested values (marine CV, poor distance) |
|-----------|--------|----------------------------------|----------------------|------------------------|---------------------------------------------|
| **max_age** | Max number of consecutive frames a track can go **without** a detection match before the track is **deleted**. | Higher → tracks survive longer occlusions / missed detections; more ghost tracks. Lower → tracks die quickly when detector misses; fewer ghosts, but more ID switches after brief dropouts. | You want to bridge short detection gaps (e.g. 1–3 s) and tolerate detector flicker. | Detector has long systematic dropouts; you get long-lived ghosts that “fly away” on predict-only. | **30–90** (1–3 s at 1–3 fps). Start ~60; increase if you see good tracks dying in brief occlusions; decrease if ghosts linger too long. |
| **min_hits** | Number of **consecutive** matches required before a track is “confirmed” and gets a stable ID. Also: a track stays “confirmed” if its **total** hits ever exceeded this (for rewarming). | Higher → fewer false tracks, slower to confirm real objects; more frames before ID appears. Lower → faster confirmation, but more spurious IDs from noisy detections. | Detections are noisy or fragmentary; you prefer stability over speed of ID. | You need instant IDs and detections are very clean. | **3–5**. With poor distance stability, **4–5** reduces flicker; 3 if you need faster confirmation. |
| **max_distance_m** | Max ENU distance (meters) between a track’s **predicted** position and a detection for them to be **allowed** to match (association gating). | Higher → more matches across frames when position jumps (e.g. bad distance); more ID switches / wrong associations. Lower → stricter matching; fewer wrong associations but more unmatched tracks → more ghosts, more fragmentations. | Distance is accurate; you want to avoid wrong associations. | Distance is noisy; you need to tolerate large frame-to-frame position jumps. | **15–40 m**. With poor distance: **25–35 m** to tolerate jumps; tighten to 15–20 m if you see wrong merges. |
| **max_speed_boat_mps** | Cap on velocity **magnitude** for **boat**-category tracks (m/s). Applied after both predict and update. 0 = force stationary. | Lower → boats move less in KF state; less “fly-away” during occlusions; velocity arrows shrink. Higher/large → effectively no cap. | You want to limit unrealistic boat speed from bad distance or KF drift. | Real boats are fast and you don’t want to cap them. | **10–25 m/s** (~20–50 kn). Use **0** for “stationary boats” if that’s your prior; otherwise 15–20 m/s to allow real motion but cap noise. |
| **max_speed_other_mps** | Same as above for **non-boat** (other) tracks. | Same as boat; applies to buoys, AIS, etc. | You want to limit speed of non-boat targets (e.g. buoys ~0, small craft moderate). | N/A | **5–15 m/s** for buoys/slow craft; **0** if you assume stationary (e.g. buoys). |
| **max_accel_boat_mps2** | Cap on acceleration **magnitude** for **boat**-category tracks (m/s²). Only used in **CA** KF model. | Lower → smoother, less responsive to sudden position jumps. Higher → more responsive, but noisier. | You use CA model and want to limit unrealistic acceleration from bad distance. | You use CV model (ignored) or want maximum responsiveness. | **2–10 m/s²** if using CA. With poor distance, **3–5** keeps motion plausible. |
| **max_accel_other_mps2** | Same as above for **non-boat** tracks. | Same as boat. | Non-boats (e.g. buoys) should not accelerate much. | N/A | **1–5 m/s²** or **0** for stationary others. |

**Notes:**

- **CV vs CA**: With `--world-space-kf-model cv`, only speed caps apply; acceleration caps are ignored. With `ca`, both apply.
- **Category**: Each track has a `category` (e.g. `"boat"` vs other). Boat caps apply only when `category == "boat"`; otherwise “other” caps apply.
- **Clamping**: Caps are applied after **both** predict and update, so even when a detection is matched, velocity/acceleration cannot exceed these values.

---

## Part 2: How Parameters Affect Each Other and Tuning as a Whole

### Interactions

| Pair / group | Interaction | Tuning tip |
|--------------|-------------|------------|
| **max_age** × **min_hits** | High max_age + high min_hits → many “unconfirmed” tracks that never get a stable ID before they age out, or that survive long as ghosts once confirmed. Low max_age + low min_hits → fast confirmation but tracks die quickly on dropout. | Keep max_age > min_hits (e.g. max_age ≥ 3× min_hits) so tracks have time to confirm; then adjust max_age for desired ghost lifetime. |
| **max_distance_m** × **max_speed_*** | Large max_distance allows matches even when predicted position has drifted far (e.g. from high uncapped speed). Capping speed reduces drift, so you can sometimes use a **tighter** max_distance_m without fragmenting. | If you tighten speed caps, you can often tighten max_distance_m a bit for cleaner association. |
| **max_distance_m** × distance accuracy | Poor distance → large frame-to-frame position jumps. Too small max_distance_m → no match → new track or ghost. Too large → wrong detections get associated to the same track. | With poor distance: raise max_distance_m enough to allow correct matches; use speed/accel caps to limit KF drift so predictions don’t run away. |
| **min_hits** × detection rate | Low framerate or many missed detections → hit_streak grows slowly. High min_hits → long delay before confirmation. | If detections are sparse, consider min_hits=3; if detector is noisy, 4–5. |
| **max_speed_*** × **max_accel_*** (CA) | Speed cap limits velocity; accel cap limits how fast velocity can change. Together they bound how far the track can “jump” in one frame from KF alone. | For stable marine use with poor distance: moderate speed cap (e.g. 15–20 m/s boat) + moderate accel cap (e.g. 5 m/s²) keeps motion plausible. |

### Suggested tuning order (marine CV, poor distance)

1. **Set kinematic caps first** (max_speed_boat_mps, max_speed_other_mps, and if CA: max_accel_*).  
   Prevents KF from flying away; makes the rest of tuning meaningful.

2. **Set max_distance_m**.  
   Large enough that correct detections still associate (e.g. 25–35 m with bad distance); not so large that clearly wrong detections match.

3. **Set min_hits**.  
   4–5 for stability with noisy/fragmentary detections.

4. **Set max_age**.  
   Long enough to bridge typical occlusions (e.g. 60 frames at 1–2 fps); shorten if you get too many long-lived ghosts.

5. **Refine**: If you see wrong associations, reduce max_distance_m or increase min_hits. If you see good tracks dying in short gaps, increase max_age. If ghosts move unrealistically, tighten speed/accel caps.

---

## Part 3: Three Output Track Types and How Parameters Affect Them

The tracker outputs three **conceptual** track types. The exact condition in code is noted for each.

---

### Type 1: Matched, confirmed tracks (“actual” tracks)

**Condition:**  
Matched a detection this frame **and** `trk.hit_streak >= min_hits`.

**Meaning:**  
Stable ID; track is “in good standing” with enough consecutive matches. Output is the **updated** KF state (position + velocity + acceleration from the **update** step). These are the primary tracks you use for fusion/display.

| Parameter | Effect on matched confirmed tracks |
|-----------|------------------------------------|
| **min_hits** | **Increase** → fewer tracks qualify; need more consecutive matches before a track becomes “confirmed”. New tracks take longer to get a stable ID. **Decrease** → more tracks qualify sooner; faster IDs, more risk of confirming noisy tracks. |
| **max_distance_m** | **Increase** → more detections can match existing tracks (including when position jumps); fewer fragmentations but more risk of wrong association. **Decrease** → stricter matching; more unmatched detections → more new tracks or rewarming; possible fragmentation. |
| **max_speed_*** / **max_accel_*** | After update, KF state is clamped. **Tighter caps** → lower velocity/acceleration in output; arrows and motion look more conservative. **Looser caps** → KF can follow noisy measurements more; motion can look jumpy. |
| **max_age** | No direct effect on *whether* a track is matched confirmed this frame. Indirect: higher max_age keeps more tracks alive, so more candidates to match → can reduce new-track creation. |
| **min_hits** | Only affects *when* a track first becomes confirmed; once confirmed, it stays confirmed until it misses too many frames (then rewarming). |

---

### Type 2: Matched, rewarming tracks

**Condition:**  
Matched a detection this frame **and** `trk.hits > min_hits` **and** `trk.hit_streak < min_hits`.

**Meaning:**  
Track was confirmed in the past (total hits > min_hits) but then missed one or more frames, so `hit_streak` reset. It matched again this frame and is “re-warming” toward confirmed. Output uses the **detection** for position/bbox and **KF state** for velocity/acceleration; often sent in the “unmatched” list with a rewarming marker.

| Parameter | Effect on rewarming tracks |
|-----------|----------------------------|
| **min_hits** | **Increase** → need more consecutive matches again to go from rewarming → confirmed; tracks stay in rewarming longer after a dropout. **Decrease** → faster return to confirmed. |
| **max_age** | **Increase** → confirmed tracks are kept longer when they don’t match; when they match again they appear as rewarming. So more rewarming tracks after occlusions. **Decrease** → tracks deleted sooner when unmatched; fewer rewarming tracks (they’re either confirmed again quickly or removed). |
| **max_distance_m** | **Increase** → easier for a returning detection to match the predicted track → more rewarming (and less “new track” creation). **Decrease** → returning detection might not match → track may die (if max_age exceeded) or stay ghost; or a new track spawns. |
| **max_speed_*** / **max_accel_*** | Same as type 1: clamp the KF state (including velocity/accel in the output). Tighter caps keep rewarming motion conservative. |

---

### Type 3: Unmatched, ghost tracks

**Condition:**  
Did **not** match a detection this frame **and** `trk.hits >= min_hits`.

**Meaning:**  
Track is confirmed but has no detection this frame. Output is **predicted** position and KF state (predict-only this frame; no update). Used to maintain ID and position/velocity during brief dropouts. “Ghost” = no current measurement.

| Parameter | Effect on ghost tracks |
|-----------|-------------------------|
| **max_age** | **Increase** → ghosts live longer (more frames without a match before deletion). You see more and longer-lived ghosts. **Decrease** → ghosts deleted sooner; fewer/longer-lived ghosts. |
| **max_distance_m** | No direct effect on *whether* a track is unmatched (it’s about association cost/gating). Indirect: **larger** max_distance can allow a marginal detection to match → fewer ghosts; **smaller** can leave more tracks unmatched → more ghosts. |
| **min_hits** | **Increase** → only tracks with more total hits count as confirmed, so fewer tracks qualify to be output as ghosts. **Decrease** → more tracks qualify as confirmed → more can appear as ghosts when unmatched. |
| **max_speed_*** / **max_accel_*** | **Critical for ghosts.** Ghosts are predict-only; no measurement to pull state back. **Tighter caps** → velocity/acceleration clamped after predict → ghost position and velocity stay bounded. **Looser caps** → ghosts can “fly away” during occlusions. With poor distance prediction, tight caps (e.g. boat 15–20 m/s, other 5–10 m/s, accel 3–5 m/s²) keep ghosts plausible. |

---

### Tracks that are never output

**Condition:**  
`trk.hits < min_hits` (never confirmed).

**Meaning:**  
These are internal only. When they match a detection, they don’t get a stable ID yet; when they don’t match, they’re not output as ghosts. They either reach `hit_streak >= min_hits` and become type 1, or they exceed `max_age` and are deleted.

| Parameter | Effect |
|-----------|--------|
| **min_hits** | **Increase** → more tracks stay in this “unconfirmed” pool longer; fewer new IDs. **Decrease** → tracks confirm sooner and start appearing as type 1 or 3. |
| **max_age** | **Increase** → unconfirmed tracks can survive longer without a match, so they have more time to reach min_hits. **Decrease** → unconfirmed tracks die sooner; more fragmentations. |

---

## Summary table: parameter → track type impact

| Parameter        | Matched confirmed (type 1)     | Matched rewarming (type 2)      | Unmatched ghost (type 3)        |
|------------------|---------------------------------|----------------------------------|---------------------------------|
| **max_age**      | Indirect (more candidates)     | More rewarming after occlusions | **Longer ghost lifetime**       |
| **min_hits**     | **Later confirmation**         | Longer rewarming phase          | Fewer ghosts (stricter confirm)|
| **max_distance_m** | More/looser matches           | More matches after dropout      | Fewer ghosts (more matches)    |
| **max_speed_***  | Lower velocity in output       | Same                            | **Prevents ghost fly-away**    |
| **max_accel_***  | Lower accel in output (CA)     | Same                            | **Prevents ghost fly-away** (CA)|

This guide is intended for the current world-space SORT implementation with CV/CA KF, ENU position, and marine boat/other fusion with CV detections and poor distance stability.
