# Lookout Perception API

**Unified Maritime Object Tracking for Enhanced Situational Awareness**

---

## What It Does

The Lookout Perception API provides real-time tracking of objects around your vessel by combining data from multiple sensors into a single, unified view. Instead of switching between radar, AIS displays, and camera feeds, operators receive one consolidated list of tracked objects with all relevant information.

**Key Benefits:**
- **Reduced Cognitive Load**: One unified track list instead of multiple disconnected displays
- **Improved Reliability**: Cross-validates detections across sensors to reduce false alarms
- **Enhanced Awareness**: Tracks objects even when individual sensors lose sight of them
- **Collision Prevention**: Automatic calculation of closest point of approach (CPA) and time to CPA

---

## Supported Sensor Inputs

| Sensor | What It Provides |
|--------|------------------|
| **Computer Vision** | Real-time camera-based detection of vessels, debris, buoys, and other objects |
| **AIS** | Vessel identification, reported position, speed, destination, and vessel details |
| **Radar** | All-weather position tracking with accurate range measurements |
| **Electronic Charts** | Known positions of buoys, markers, rocks, and other navigation aids |

The API intelligently combines these sources. For example, a cargo ship might be detected by the camera, identified by AIS (providing its name, destination, and cargo type), and tracked by radar through fog—all appearing as a single unified track.

---

## API Output Structure

Each message contains three main sections:

```
{
  "api_version": "0.1.0",
  "message_type": "track_update",
  "timestamp_utc": "2026-01-30T15:30:00.123Z",
  "frame_number": 12345,
  
  "ego": { ... },        // Your vessel's state
  "tracks": [ ... ],     // Array of tracked objects
  "diagnostics": { ... } // System health
}
```

---

## Track Fields Reference

Each object in the `tracks` array contains the following fields:

### Identity & Classification

| Field | Type | Description |
|-------|------|-------------|
| `track.track_id` | integer | Unique identifier for this track |
| `track.track_uuid` | string | UUID for cross-system reference |
| `track.sources` | array | Active sensors: `["cv", "ais", "radar", "chart"]` |
| `track.primary_source` | string | Most authoritative source for this track |
| `track.category` | string | `"vessel"`, `"aid_to_navigation"`, `"hazard"`, `"other"` |
| `track.subcategory` | string | e.g., `"cargo"`, `"fishing"`, `"buoy"`, `"rock"` |

### Position (Fused Estimate)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `track.position.fused.lat` | float | degrees | Latitude (WGS84) |
| `track.position.fused.lon` | float | degrees | Longitude (WGS84) |
| `track.position.fused.enu_e_m` | float | meters | East position in local frame |
| `track.position.fused.enu_n_m` | float | meters | North position in local frame |
| `track.position.fused.sigma_e_m` | float | meters | Position uncertainty (east) |
| `track.position.fused.sigma_n_m` | float | meters | Position uncertainty (north) |

### Position (Relative to Your Vessel)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `track.position.relative_to_ego.distance_m` | float | meters | Range to target |
| `track.position.relative_to_ego.bearing_deg` | float | degrees | True bearing (0-360°) |
| `track.position.relative_to_ego.bearing_relative_deg` | float | degrees | Relative to your heading |
| `track.position.relative_to_ego.x_m` | float | meters | Forward distance |
| `track.position.relative_to_ego.y_m` | float | meters | Lateral distance |

### Velocity (Absolute)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `track.velocity.absolute.speed_mps` | float | m/s | Speed over ground |
| `track.velocity.absolute.speed_knots` | float | knots | Speed over ground |
| `track.velocity.absolute.cog_deg` | float | degrees | Course over ground (0-360°) |
| `track.velocity.absolute.enu_e_mps` | float | m/s | East velocity component |
| `track.velocity.absolute.enu_n_mps` | float | m/s | North velocity component |
| `track.velocity.absolute.sigma_speed_mps` | float | m/s | Velocity uncertainty |

### Velocity (Relative to Your Vessel)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `track.velocity.relative_to_ego.closing_speed_mps` | float | m/s | Rate of approach (+ = closing) |
| `track.velocity.relative_to_ego.closing_speed_knots` | float | knots | Rate of approach |
| `track.velocity.relative_to_ego.lateral_speed_mps` | float | m/s | Cross-track velocity |
| `track.velocity.relative_to_ego.relative_cog_deg` | float | degrees | Relative motion direction |

### Heading & Dimensions

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `track.heading.deg` | float | degrees | Vessel heading (bow direction) |
| `track.heading.rate_dps` | float | deg/s | Turn rate |
| `track.heading.source` | string | - | `"ais"`, `"cv"`, or `"radar"` |
| `track.dimensions.length_m` | float | meters | Vessel length |
| `track.dimensions.width_m` | float | meters | Vessel beam |
| `track.dimensions.draught_m` | float | meters | Draft (from AIS) |

### Collision Assessment

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `track.collision_assessment.cpa_m` | float | meters | Closest point of approach |
| `track.collision_assessment.tcpa_s` | float | seconds | Time to CPA |
| `track.collision_assessment.cpa_position.lat` | float | degrees | CPA location latitude |
| `track.collision_assessment.cpa_position.lon` | float | degrees | CPA location longitude |
| `track.collision_assessment.threat_level` | string | - | `"none"`, `"watch"`, `"warning"`, `"danger"` |
| `track.collision_assessment.is_closing` | boolean | - | True if range is decreasing |

### Predicted Positions

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `track.predictions[].horizon_s` | float | seconds | Prediction time horizon |
| `track.predictions[].position.lat` | float | degrees | Predicted latitude |
| `track.predictions[].position.lon` | float | degrees | Predicted longitude |
| `track.predictions[].uncertainty_m` | float | meters | Prediction uncertainty |

### Track Quality & Lifecycle

| Field | Type | Description |
|-------|------|-------------|
| `track.quality.score` | float | Overall track quality (0.0 - 1.0) |
| `track.quality.confidence` | float | Detection confidence (0.0 - 1.0) |
| `track.quality.position_uncertainty_m` | float | Position accuracy in meters |
| `track.lifecycle.state` | string | `"tentative"`, `"confirmed"`, `"coasting"`, `"static"` |
| `track.lifecycle.age_seconds` | float | Time since track was created |
| `track.lifecycle.time_since_update_ms` | integer | Time since last sensor update |

### Anomaly Detection

| Field | Type | Description |
|-------|------|-------------|
| `track.anomalies.erratic_motion` | boolean | Unusual acceleration or turn rate |
| `track.anomalies.ais_position_jump` | boolean | Suspicious position discontinuity |
| `track.anomalies.ais_speed_mismatch` | boolean | Reported vs computed speed differs |
| `track.anomalies.stationary_prolonged` | boolean | Vessel stopped for extended period |

---

## Raw Sensor Data

Each track includes the original sensor readings when available:

### CV Raw (`track.cv_raw`)

| Field | Type | Description |
|-------|------|-------------|
| `track.cv_raw.detection_id` | integer | Detection ID from vision system |
| `track.cv_raw.bbox.x_px` | float | Bounding box X (pixels) |
| `track.cv_raw.bbox.y_px` | float | Bounding box Y (pixels) |
| `track.cv_raw.bbox.width_px` | float | Bounding box width |
| `track.cv_raw.bbox.height_px` | float | Bounding box height |
| `track.cv_raw.confidence` | float | Detection confidence (0.0 - 1.0) |
| `track.cv_raw.category` | string | CV classification |
| `track.cv_raw.distance_m` | float | Estimated range from camera |
| `track.cv_raw.heading_deg` | float | Estimated target heading |

### AIS Raw (`track.ais_raw`)

| Field | Type | Description |
|-------|------|-------------|
| `track.ais_raw.mmsi` | integer | Maritime Mobile Service Identity |
| `track.ais_raw.imo` | integer | IMO ship identification number |
| `track.ais_raw.name` | string | Vessel name |
| `track.ais_raw.callsign` | string | Radio call sign |
| `track.ais_raw.ship_type` | integer | AIS ship type code |
| `track.ais_raw.ship_type_name` | string | Human-readable ship type |
| `track.ais_raw.destination` | string | Reported destination |
| `track.ais_raw.eta` | string | Estimated time of arrival (ISO 8601) |
| `track.ais_raw.nav_status` | integer | Navigation status code |
| `track.ais_raw.nav_status_name` | string | e.g., "Under way using engine" |
| `track.ais_raw.position.lat` | float | AIS reported latitude |
| `track.ais_raw.position.lon` | float | AIS reported longitude |
| `track.ais_raw.cog_deg` | float | AIS course over ground |
| `track.ais_raw.sog_knots` | float | AIS speed over ground |
| `track.ais_raw.heading_deg` | float | AIS true heading |
| `track.ais_raw.rot_dpm` | float | Rate of turn (degrees/min) |
| `track.ais_raw.last_update_utc` | string | Last AIS message timestamp |

### Chart Raw (`track.chart_raw`)

| Field | Type | Description |
|-------|------|-------------|
| `track.chart_raw.feature_id` | string | Chart feature identifier |
| `track.chart_raw.name` | string | Feature name |
| `track.chart_raw.feature_type` | string | `"lateral_buoy"`, `"rock"`, `"light"`, etc. |
| `track.chart_raw.color` | string | Buoy/marker color |
| `track.chart_raw.light_characteristics` | string | e.g., "Fl G 4s" |
| `track.chart_raw.position.lat` | float | Charted latitude |
| `track.chart_raw.position.lon` | float | Charted longitude |

---

## Ego Vessel State (`ego`)

Your vessel's current state:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `ego.position.lat` | float | degrees | Current latitude |
| `ego.position.lon` | float | degrees | Current longitude |
| `ego.position.sigma_m` | float | meters | Position accuracy |
| `ego.heading_deg` | float | degrees | Compass heading |
| `ego.cog_deg` | float | degrees | Course over ground |
| `ego.sog_mps` | float | m/s | Speed over ground |
| `ego.sog_knots` | float | knots | Speed over ground |
| `ego.velocity.enu_e_mps` | float | m/s | East velocity |
| `ego.velocity.enu_n_mps` | float | m/s | North velocity |

---

## Configuration Options

### Enable/Disable Sensors

```json
{
  "sources": {
    "cv": { "enabled": true },
    "ais": { "enabled": true },
    "radar": { "enabled": false },
    "chart": { "enabled": true }
  }
}
```

### Safety Thresholds

```json
{
  "collision_assessment": {
    "cpa_warning_threshold_m": 500.0,
    "cpa_danger_threshold_m": 100.0,
    "tcpa_horizon_s": 600.0,
    "tcpa_warning_threshold_s": 300.0
  }
}
```

### Prediction Horizons

```json
{
  "prediction": {
    "enabled": true,
    "horizons_s": [5.0, 10.0, 30.0, 60.0]
  }
}
```

### Alert Zones

```json
{
  "zones": {
    "enabled": true,
    "definitions": [
      {
        "zone_id": "anchorage_1",
        "name": "Miami Anchorage A",
        "type": "polygon",
        "coordinates": [...],
        "alert_on_entry": true
      }
    ]
  }
}
```

---

## Connection Endpoints

| Endpoint | Protocol | Purpose |
|----------|----------|---------|
| `ws://host:5010/tracks` | WebSocket (JSON) | Real-time track stream at 30 fps |
| `ws://host:5011/tracks` | WebSocket (Binary) | Compact binary stream at 30 fps |
| `ws://host:5012/control` | WebSocket | Configuration and commands |
| `webrtc://host:5013` | WebRTC DataChannel | NAT-friendly streaming |
| `http://host:8080/api/v1/config` | HTTP REST | GET/PUT configuration |
| `http://host:8080/api/v1/tracks` | HTTP REST | Single snapshot query |
| `http://host:8080/api/v1/health` | HTTP REST | System health check |

---

## Example Track Output

```json
{
  "track_id": 42,
  "sources": ["cv", "ais"],
  "primary_source": "ais",
  "category": "vessel",
  "subcategory": "cargo",
  
  "position": {
    "fused": {
      "lat": 25.665634,
      "lon": -80.249833
    },
    "relative_to_ego": {
      "distance_m": 660.0,
      "bearing_deg": 26.3
    }
  },
  
  "velocity": {
    "absolute": {
      "speed_knots": 5.4,
      "cog_deg": 115.6
    },
    "relative_to_ego": {
      "closing_speed_knots": 3.5
    }
  },
  
  "collision_assessment": {
    "cpa_m": 125.0,
    "tcpa_s": 180.0,
    "threat_level": "none"
  },
  
  "quality": {
    "score": 0.85,
    "confidence": 0.92
  },
  
  "ais_raw": {
    "mmsi": 123456789,
    "name": "CARGO VESSEL",
    "destination": "MIAMI",
    "eta": "2026-01-30T18:00:00Z"
  }
}
```

---

## Technical Requirements

- Standard marine camera systems (analog or IP)
- AIS receiver with NMEA output
- Radar with ARPA target output (optional)
- Network connection to perception processor
- Compatible with NMEA 0183/2000 navigation data

---

## Integration Support

- Comprehensive API documentation
- Sample client implementations
- Testing and validation tools
- Technical integration assistance

---

*For full technical specifications including internal architecture, see [API_DESIGN.md](API_DESIGN.md).*
