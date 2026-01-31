# Lookout Multi-Source Tracking API Design

**Version:** 0.1.0 (Draft)  
**Last Updated:** 2026-01-30  
**Status:** Design Phase

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Protocol & Transport](#protocol--transport)
4. [Configuration API (SET)](#configuration-api-set)
5. [Track Output API (GET)](#track-output-api-get)
6. [Ego State Output](#ego-state-output)
7. [Diagnostics Output](#diagnostics-output)
8. [WebSocket Message Format](#websocket-message-format)
9. [Data Size Estimates](#data-size-estimates)
10. [Future Considerations](#future-considerations)

---

## Overview

The Lookout Tracking API provides real-time multi-source object tracking for maritime situational awareness. It fuses data from multiple sensor sources:

| Source | Update Rate | Strengths | Weaknesses |
|--------|-------------|-----------|------------|
| **CV (Computer Vision)** | 30 fps | High temporal resolution, good bearing accuracy | Noisy distance, weather-dependent |
| **AIS** | 2-10 sec | High position accuracy, vessel metadata (MMSI, name, destination) | Self-reported (can be spoofed), low update rate |
| **Radar** | 1-5 Hz | All-weather, range-accurate | No classification, clutter |
| **Chart Data** | Static | Known positions for aids to navigation | Static only, no moving targets |

The API outputs a unified track list with fused estimates, per-source raw data, and collision assessment.

---

## Architecture

### Single KF Per Track with Multi-Source Updates

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           INPUT SOURCES                                  │
├──────────────┬──────────────┬──────────────┬──────────────┬─────────────┤
│   CV Stream  │  AIS Stream  │ Radar Stream │ Chart Data   │  Ego NMEA   │
│   (30 fps)   │  (2-10 sec)  │  (1-5 Hz)    │  (static)    │  (1-10 Hz)  │
│   R = 100m²  │   R = 1m²    │   R = 25m²   │  (no KF)     │             │
└──────┬───────┴──────┬───────┴──────┬───────┴──────┬───────┴──────┬──────┘
       │              │              │              │              │
       │              │              │              │              ▼
       │              │              │              │      ┌──────────────┐
       │              │              │              │      │   Ego KF     │
       │              │              │              │      │  (smoother)  │
       │              │              │              │      └──────┬───────┘
       │              │              │              │              │
       └──────────────┴──────────────┴──────────────┘              │
                             │                                     │
                             ▼                                     │
              ┌─────────────────────────────┐                      │
              │     ASSOCIATION LAYER       │◄─────────────────────┘
              │  (Hungarian algorithm)      │
              │  Match detections → tracks  │
              └──────────────┬──────────────┘
                             │
                             ▼
              ┌─────────────────────────────┐
              │   SINGLE KF PER TRACK       │
              │                             │
              │  Sequential update with     │
              │  source-specific R values:  │
              │  - AIS: R = 1m² (trusted)   │
              │  - Radar: R = 25m²          │
              │  - CV: R = 100m² (noisy)    │
              └──────────────┬──────────────┘
                             │
                             ▼
              ┌─────────────────────────────┐
              │      Fused Track List       │
              │  (KF state + raw data)      │
              └──────────────┬──────────────┘
                             │
        ┌────────────┬───────┼───────┬────────────┐
        ▼            ▼       ▼       ▼            ▼
  ┌──────────┐ ┌──────────┐ ┌────┐ ┌──────────┐ ┌──────────┐
  │ WebSocket│ │ WebSocket│ │HTTP│ │  WebRTC  │ │  (future)│
  │  (JSON)  │ │(MsgPack) │ │REST│ │ DataChan │ │  ROS 2   │
  │  :5010   │ │  :5011   │ │8080│ │  :5013   │ │          │
  └──────────┘ └──────────┘ └────┘ └──────────┘ └──────────┘
```

**Why Single KF Per Track:**
- Simpler architecture, easier to maintain and debug
- Avoids "double filtering" - AIS data is already GPS-quality and pre-smoothed
- Source-specific measurement noise (R matrix) handles trust levels:
  - AIS with very low R (1m²) → strongly corrects position
  - CV with high R (100m²) → contributes but doesn't override AIS
  - Radar with medium R (25m²) → useful when AIS unavailable
- Sequential updates are mathematically equivalent regardless of order
- High-rate CV maintains track between low-rate AIS updates

**Update Flow (when multiple sources arrive):**
```
Frame k: CV detection + AIS message arrive

1. PREDICT track state to time k
2. ASSOCIATE: Match CV det → Track, Match AIS msg → Track (via MMSI or position)
3. UPDATE sequentially (order doesn't matter for linear KF):
   a. KF.update(cv_measurement, R=100m²)
   b. KF.update(ais_measurement, R=1m²)    ← AIS will dominate due to low R
4. Output fused state
```

---

## Protocol & Transport

### Protocol Comparison

| Criterion | WebSocket | WebRTC DataChannel | HTTP Polling | ROS 2 |
|-----------|-----------|-------------------|--------------|-------|
| Latency | ~1-5 ms | ~1-5 ms | ~50-100 ms | ~1-5 ms |
| 30 fps capable | Yes | Yes | Marginal | Yes |
| Browser support | Native | Native | Native | Requires bridge |
| Bidirectional | Yes | Yes | No | Yes |
| NAT traversal | No (needs port forward) | Yes (STUN/TURN) | Yes | No |
| P2P capable | No | Yes | No | No |
| Debug/Dev ease | Excellent | Moderate | Good | Steeper curve |
| Multi-client | Built-in broadcast | Per-peer channels | Server load | Native pub/sub |

### Recommended Strategy

- **WebSocket (JSON)**: Primary for development, debugging, browser clients on local network
- **WebSocket (MessagePack)**: Production with bandwidth constraints
- **WebRTC DataChannel**: Remote clients, NAT traversal, P2P scenarios
- **HTTP REST**: Configuration, health checks, one-off queries
- **ROS 2**: Future integration with robotics ecosystem

### Endpoints

| Endpoint | Protocol | Purpose |
|----------|----------|---------|
| `ws://host:5010/tracks` | WebSocket (JSON) | Real-time track stream |
| `ws://host:5011/tracks` | WebSocket (MessagePack) | Efficient binary stream |
| `ws://host:5012/control` | WebSocket (JSON) | Bidirectional config/commands |
| `webrtc://host:5013` | WebRTC DataChannel | NAT-friendly track stream (JSON or binary) |
| `http://host:8080/api/v1/config` | HTTP REST | GET/PUT configuration |
| `http://host:8080/api/v1/health` | HTTP REST | Health check |
| `http://host:8080/api/v1/tracks` | HTTP REST | Single snapshot (polling fallback) |
| `http://host:8080/api/v1/webrtc/offer` | HTTP REST | WebRTC signaling endpoint |

### WebRTC Integration

WebRTC DataChannels provide:
- **NAT traversal**: Works through firewalls without port forwarding
- **Low latency**: Similar to WebSocket, but peer-to-peer when possible
- **Reliable or unreliable**: Can choose ordered/reliable or unordered/unreliable delivery
- **Browser native**: No additional libraries needed in modern browsers

**Signaling flow:**
```
Client                          Server
   │                               │
   │──── HTTP POST /webrtc/offer ─►│  (SDP offer)
   │◄─── HTTP 200 (SDP answer) ────│
   │                               │
   │◄─── ICE candidates ──────────►│  (via WebSocket or HTTP)
   │                               │
   │◄════ DataChannel established ═►│
   │                               │
   │◄─── Track updates (30 fps) ───│  (JSON or MessagePack)
```

### Why JSON as Default

At 30 fps with ~50 tracks:
- JSON (compact): ~50 KB/frame → ~1.5 MB/s
- MessagePack: ~20 KB/frame → ~600 KB/s

For local network and development, JSON's debuggability outweighs the bandwidth cost. MessagePack is available for production or constrained environments.

---

## Configuration API (SET)

### Full Configuration Schema

```json
{
  "api_version": "0.1.0",
  
  "sources": {
    "cv": {
      "enabled": true,
      "input_port": 5001,
      "measurement_noise_r_m2": 100.0,
      "notes": "High R because CV distance estimates are noisy"
    },
    "ais": {
      "enabled": true,
      "input_port": 3636,
      "measurement_noise_r_m2": 1.0,
      "trust_reported_cog_sog": true,
      "notes": "Low R because AIS is GPS-quality and pre-smoothed"
    },
    "radar": {
      "enabled": false,
      "input_port": 4001,
      "measurement_noise_r_m2": 25.0,
      "notes": "Medium R, range-dependent accuracy"
    },
    "chart": {
      "enabled": true,
      "features": {
        "buoys": true,
        "rocks": true,
        "lights": true,
        "landmarks": false,
        "depth_contours": false,
        "anchorage_areas": false
      }
    }
  },

  "tracking": {
    "update_interval_ms": 33,
    "max_age_frames": 300,
    "max_age_seconds": 10.0,
    "min_hits_to_confirm": 40,
    "new_track_min_confidence": 0.5,
    "kalman_filter": {
      "process_noise_position_q": 1.0,
      "process_noise_velocity_q": 0.5,
      "initial_velocity_uncertainty": 1000.0,
      "notes": "Single KF per track; R values are per-source (see sources config)"
    }
  },

  "association": {
    "world_position": {
      "enabled": true,
      "max_distance_m": 50.0,
      "weight": 1.0
    },
    "polar_coords": {
      "enabled": true,
      "max_bearing_diff_deg": 15.0,
      "max_distance_ratio": 0.3,
      "weight": 0.5
    },
    "image_space_iou": {
      "enabled": true,
      "threshold": 0.1,
      "weight": 0.3
    },
    "heading_diff": {
      "enabled": false,
      "max_diff_deg": 45.0,
      "weight": 0.2
    },
    "mmsi_matching": {
      "enabled": true,
      "weight": 10.0
    }
  },

  "collision_assessment": {
    "enabled": true,
    "cpa_warning_threshold_m": 500.0,
    "cpa_danger_threshold_m": 100.0,
    "tcpa_horizon_s": 600.0,
    "tcpa_warning_threshold_s": 300.0,
    "min_closing_speed_mps": 0.5
  },

  "prediction": {
    "enabled": true,
    "horizons_s": [5.0, 10.0, 30.0, 60.0],
    "use_constant_velocity": true,
    "use_constant_turn_rate": false
  },

  "zones": {
    "enabled": false,
    "definitions": [
      {
        "zone_id": "anchorage_1",
        "name": "Miami Anchorage A",
        "type": "polygon",
        "coordinates": [
          {"lat": 25.76, "lon": -80.13},
          {"lat": 25.76, "lon": -80.11},
          {"lat": 25.74, "lon": -80.11},
          {"lat": 25.74, "lon": -80.13}
        ],
        "alert_on_entry": true,
        "alert_on_exit": false
      },
      {
        "zone_id": "danger_zone_1",
        "name": "Shallow Water",
        "type": "circle",
        "center": {"lat": 25.70, "lon": -80.20},
        "radius_m": 200.0,
        "alert_on_entry": true
      }
    ]
  },

  "anomaly_detection": {
    "enabled": true,
    "erratic_motion": {
      "enabled": true,
      "max_acceleration_mps2": 5.0,
      "max_turn_rate_dps": 30.0
    },
    "ais_spoofing": {
      "enabled": true,
      "position_jump_threshold_m": 1000.0,
      "speed_mismatch_threshold_mps": 10.0
    },
    "stationary_vessel": {
      "enabled": true,
      "speed_threshold_mps": 0.5,
      "duration_threshold_s": 300.0
    }
  },

  "output": {
    "include_raw_data": true,
    "include_kf_diagnostics": true,
    "include_predictions": true,
    "include_association_history": false,
    "association_history_max_entries": 10,
    "coordinate_precision_digits": 6,
    "compact_json": true
  }
}
```

### Configuration via WebSocket

Send to `ws://host:5012/control`:

```json
{
  "command": "set_config",
  "path": "sources.cv.enabled",
  "value": false
}
```

```json
{
  "command": "set_config",
  "path": "collision_assessment.cpa_warning_threshold_m",
  "value": 300.0
}
```

Response:

```json
{
  "ack": "set_config",
  "path": "sources.cv.enabled",
  "value": false,
  "success": true
}
```

### Configuration via HTTP REST

```http
GET /api/v1/config
```

```http
PUT /api/v1/config
Content-Type: application/json

{
  "sources": {
    "radar": {
      "enabled": true
    }
  }
}
```

---

## Track Output API (GET)

### Complete Track Message Schema

```json
{
  "api_version": "0.1.0",
  "message_type": "track_update",
  "timestamp_utc": "2026-01-30T15:30:00.123Z",
  "timestamp_unix_ms": 1769868600123,
  "frame_number": 12345,
  "processing_latency_ms": 8.2,

  "ego": { },

  "tracks": [
    {
      "track_id": 42,
      "track_uuid": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",

      "sources": ["cv", "ais"],
      "primary_source": "ais",
      "source_count": 2,

      "category": "vessel",
      "subcategory": "cargo",

      "quality": {
        "score": 0.85,
        "confidence": 0.92,
        "position_uncertainty_m": 5.2,
        "velocity_uncertainty_mps": 0.8,
        "factors": {
          "source_agreement": 0.95,
          "measurement_rate": 0.80,
          "filter_convergence": 0.85
        }
      },

      "lifecycle": {
        "state": "confirmed",
        "age_frames": 1200,
        "age_seconds": 40.0,
        "hits": 950,
        "misses": 250,
        "time_since_update_frames": 2,
        "time_since_update_ms": 66,
        "created_at_utc": "2026-01-30T15:29:20.000Z"
      },

      "position": {
        "fused": {
          "lat": 25.665634,
          "lon": -80.249833,
          "enu_e_m": -306.54,
          "enu_n_m": -747.87,
          "sigma_e_m": 5.2,
          "sigma_n_m": 5.2
        },
        "relative_to_ego": {
          "distance_m": 660.0,
          "bearing_deg": 26.3,
          "bearing_relative_deg": -18.9,
          "x_m": 591.7,
          "y_m": 292.4
        }
      },

      "velocity": {
        "absolute": {
          "enu_e_mps": 2.5,
          "enu_n_mps": -1.2,
          "speed_mps": 2.77,
          "speed_knots": 5.4,
          "cog_deg": 115.6,
          "sigma_speed_mps": 0.5
        },
        "relative_to_ego": {
          "closing_speed_mps": 1.8,
          "closing_speed_knots": 3.5,
          "lateral_speed_mps": 0.6,
          "relative_cog_deg": 142.3
        }
      },

      "heading": {
        "deg": 116.0,
        "sigma_deg": 5.0,
        "rate_dps": 0.0,
        "source": "ais"
      },

      "dimensions": {
        "length_m": 45.0,
        "width_m": 12.0,
        "height_m": null,
        "draught_m": 4.5,
        "source": "ais"
      },

      "collision_assessment": {
        "cpa_m": 125.0,
        "tcpa_s": 180.0,
        "cpa_position": {
          "lat": 25.667000,
          "lon": -80.248000
        },
        "threat_level": "none",
        "is_closing": true
      },

      "predictions": [
        {
          "horizon_s": 5.0,
          "position": {
            "lat": 25.665750,
            "lon": -80.249700,
            "enu_e_m": -294.2,
            "enu_n_m": -741.5
          },
          "uncertainty_m": 8.5
        },
        {
          "horizon_s": 30.0,
          "position": {
            "lat": 25.666500,
            "lon": -80.249000,
            "enu_e_m": -232.1,
            "enu_n_m": -710.2
          },
          "uncertainty_m": 25.0
        }
      ],

      "zones": [
        {
          "zone_id": "anchorage_1",
          "name": "Miami Anchorage A",
          "status": "inside",
          "entered_at_utc": "2026-01-30T15:28:00.000Z"
        }
      ],

      "anomalies": {
        "flags": [],
        "erratic_motion": false,
        "ais_position_jump": false,
        "ais_speed_mismatch": false,
        "stationary_prolonged": false,
        "last_anomaly_utc": null
      },

      "association_history": {
        "recent_sources": [
          {"source": "ais", "last_seen_utc": "2026-01-30T15:29:58.000Z"},
          {"source": "cv", "last_seen_utc": "2026-01-30T15:30:00.100Z"}
        ],
        "total_cv_hits": 800,
        "total_ais_hits": 20,
        "total_radar_hits": 0
      },

      "cv_raw": {
        "detection_id": 7,
        "bbox": {
          "x_px": 36.04,
          "y_px": 0.65,
          "width_px": 21.99,
          "height_px": 6.52,
          "image_width_px": 1920,
          "image_height_px": 1080
        },
        "confidence": 0.85,
        "category": "boat",
        "distance_m": 660.0,
        "heading_deg": 112.5,
        "position": {
          "lat": 25.665634,
          "lon": -80.249833,
          "enu_e_m": -306.54,
          "enu_n_m": -747.87
        },
        "kf_state": {
          "sigma_e_m": 147.1,
          "sigma_n_m": 147.1,
          "vel_e_mps": 2.4,
          "vel_n_mps": -1.1,
          "outlier": false,
          "gate_d2": 1.2,
          "reset_count": 0
        }
      },

      "ais_raw": {
        "mmsi": 123456789,
        "imo": 9876543,
        "name": "CARGO VESSEL",
        "callsign": "ABCD1",
        "ship_type": 70,
        "ship_type_name": "Cargo",
        "destination": "MIAMI",
        "eta": "2026-01-30T18:00:00Z",
        "nav_status": 0,
        "nav_status_name": "Under way using engine",
        "position": {
          "lat": 25.665640,
          "lon": -80.249830,
          "accuracy": "high"
        },
        "cog_deg": 115.5,
        "sog_knots": 5.5,
        "heading_deg": 116.0,
        "rot_dpm": 0.0,
        "dimensions": {
          "to_bow_m": 30.0,
          "to_stern_m": 15.0,
          "to_port_m": 6.0,
          "to_starboard_m": 6.0
        },
        "last_update_utc": "2026-01-30T15:29:58.000Z",
        "message_count": 45,
        "kf_state": {
          "sigma_e_m": 2.1,
          "sigma_n_m": 2.1,
          "vel_e_mps": 2.5,
          "vel_n_mps": -1.2
        }
      },

      "radar_raw": null,

      "chart_raw": null
    },

    {
      "track_id": 99,
      "sources": ["chart"],
      "primary_source": "chart",
      "category": "aid_to_navigation",
      "subcategory": "buoy",

      "quality": {
        "score": 1.0,
        "confidence": 1.0,
        "position_uncertainty_m": 1.0
      },

      "lifecycle": {
        "state": "static",
        "age_frames": null,
        "age_seconds": null
      },

      "position": {
        "fused": {
          "lat": 25.668000,
          "lon": -80.251000,
          "enu_e_m": -412.3,
          "enu_n_m": -485.6
        },
        "relative_to_ego": {
          "distance_m": 320.0,
          "bearing_deg": 310.5,
          "bearing_relative_deg": 265.3
        }
      },

      "velocity": null,
      "heading": null,
      "dimensions": null,
      "collision_assessment": null,
      "predictions": null,
      "anomalies": null,
      "association_history": null,

      "cv_raw": null,
      "ais_raw": null,
      "radar_raw": null,

      "chart_raw": {
        "feature_id": "US12345",
        "name": "Miami Channel Marker 7",
        "feature_type": "lateral_buoy",
        "color": "green",
        "shape": "can",
        "light_characteristics": "Fl G 4s",
        "radar_reflector": true,
        "position": {
          "lat": 25.668000,
          "lon": -80.251000
        },
        "chart_source": "NOAA ENC US5FL12M"
      }
    }
  ],

  "diagnostics": { }
}
```

### Track Quality Score Calculation

The `quality.score` is a weighted composite:

```
quality_score = w1 * source_agreement 
              + w2 * measurement_rate 
              + w3 * filter_convergence 
              + w4 * confidence_mean

where:
  source_agreement  = 1.0 if all sources agree within threshold, else proportional
  measurement_rate  = min(1.0, actual_rate / expected_rate)
  filter_convergence = 1.0 - normalized_covariance
  confidence_mean   = mean of per-source confidence values
```

Factors contributing to quality:

| Factor | Description | Weight (default) |
|--------|-------------|------------------|
| `source_agreement` | How well do multiple sources agree on position? | 0.3 |
| `measurement_rate` | Are we receiving updates at expected rate? | 0.2 |
| `filter_convergence` | Has the Kalman filter converged (low covariance)? | 0.3 |
| `confidence` | Mean confidence from detection sources | 0.2 |

### Anomaly Flags

| Flag | Trigger Condition |
|------|-------------------|
| `erratic_motion` | Acceleration > threshold OR turn rate > threshold |
| `ais_position_jump` | AIS position moved > 1000m between updates (impossible speed) |
| `ais_speed_mismatch` | Reported SOG differs from computed speed by > threshold |
| `stationary_prolonged` | Speed < 0.5 m/s for > 5 minutes |
| `cv_radar_mismatch` | CV and radar disagree on position by > threshold |
| `heading_speed_mismatch` | Heading differs significantly from COG while moving |

### Association History

To avoid unbounded data growth, association history is:
- **Limited to recent entries** (configurable, default 10)
- **Summarized as counts** (total hits per source)
- **Optional** (can be disabled via `output.include_association_history`)

---

## Ego State Output

```json
{
  "ego": {
    "timestamp_utc": "2026-01-30T15:30:00.100Z",
    "data_age_ms": 23,

    "position": {
      "lat": 25.672341,
      "lon": -80.243567,
      "altitude_m": null,
      "sigma_m": 2.5,
      "source": "gps"
    },

    "enu_reference": {
      "lat": 25.670000,
      "lon": -80.240000,
      "description": "First GPS fix of session"
    },

    "enu_position": {
      "e_m": 258.7,
      "n_m": 259.9
    },

    "heading_deg": 45.2,
    "heading_sigma_deg": 1.0,
    "heading_source": "compass",

    "cog_deg": 44.8,
    "sog_mps": 5.2,
    "sog_knots": 10.1,

    "velocity": {
      "enu_e_mps": 3.67,
      "enu_n_mps": 3.69
    },

    "attitude": {
      "roll_deg": null,
      "pitch_deg": null,
      "yaw_rate_dps": null
    },

    "nmea_sentences_received": {
      "GGA": 1500,
      "HDT": 1480,
      "VTG": 1490,
      "RMC": 1500
    }
  }
}
```

---

## Diagnostics Output

```json
{
  "diagnostics": {
    "sources": {
      "cv": {
        "connected": true,
        "last_message_utc": "2026-01-30T15:30:00.100Z",
        "messages_received": 45000,
        "messages_per_second": 30.1,
        "detections_this_frame": 12,
        "active_tracks": 8
      },
      "ais": {
        "connected": true,
        "last_message_utc": "2026-01-30T15:29:58.000Z",
        "messages_received": 5000,
        "messages_per_second": 2.3,
        "unique_mmsi_count": 45,
        "active_tracks": 42
      },
      "radar": {
        "connected": false,
        "last_message_utc": null,
        "messages_received": 0
      },
      "ego_nmea": {
        "connected": true,
        "last_message_utc": "2026-01-30T15:30:00.080Z",
        "messages_received": 15000,
        "has_valid_position": true,
        "has_valid_heading": true
      }
    },

    "fusion": {
      "total_tracks": 50,
      "confirmed_tracks": 42,
      "tentative_tracks": 8,
      "multi_source_tracks": 5,
      "associations_this_frame": 3,
      "dropped_tracks_this_frame": 1
    },

    "performance": {
      "frame_processing_time_ms": 8.2,
      "fusion_time_ms": 2.1,
      "serialization_time_ms": 1.5,
      "output_queue_depth": 0
    },

    "errors": {
      "recent": [],
      "total_count": 0
    }
  }
}
```

---

## WebSocket Message Format

### Track Stream (`ws://host:5010/tracks`)

Messages are sent at the configured update rate (default 30 fps).

**JSON format:**
```json
{"api_version":"0.1.0","message_type":"track_update","timestamp_utc":"2026-01-30T15:30:00.123Z",...}
```

**MessagePack format** (port 5011):
Same schema, binary-encoded with MessagePack.

### Control Channel (`ws://host:5012/control`)

**Client → Server Commands:**

| Command | Description |
|---------|-------------|
| `get_config` | Request full configuration |
| `set_config` | Update configuration value |
| `get_tracks` | Request single track snapshot |
| `pause_stream` | Pause track output |
| `resume_stream` | Resume track output |
| `reset_tracker` | Clear all tracks and reinitialize |

**Server → Client Messages:**

| Message Type | Description |
|--------------|-------------|
| `config` | Full or partial configuration |
| `ack` | Command acknowledgment |
| `error` | Error notification |
| `status` | Periodic status update |

---

## Data Size Estimates

### Per-Track Sizes

| Content Level | JSON (compact) | MessagePack |
|---------------|----------------|-------------|
| Minimal (position only) | ~200 bytes | ~100 bytes |
| Standard (no raw data) | ~500 bytes | ~250 bytes |
| Full (with raw data) | ~1.5 KB | ~700 bytes |
| Full + predictions + history | ~2.5 KB | ~1.2 KB |

### Frame Sizes (50 tracks)

| Content Level | JSON | MessagePack |
|---------------|------|-------------|
| Standard | ~25 KB | ~12 KB |
| Full | ~75 KB | ~35 KB |
| Full + extras | ~125 KB | ~60 KB |

### Bandwidth at 30 fps

| Content Level | JSON | MessagePack |
|---------------|------|-------------|
| Standard | ~750 KB/s | ~360 KB/s |
| Full | ~2.3 MB/s | ~1.0 MB/s |
| Full + extras | ~3.8 MB/s | ~1.8 MB/s |

---

## Future Considerations

### Potential Additions

1. **Historical Playback API**: Query past tracks by time range
2. **Event Stream**: Separate WebSocket for alerts/events only
3. **Video Overlay Integration**: Include track rendering instructions
4. **Multi-Ego Support**: Track relative to multiple reference vessels
5. **Weather Integration**: Include environmental data affecting tracking

### Protocol Extensions

1. **Protocol Buffers**: If bandwidth becomes critical
2. **ROS 2 Topics**: For integration with robotics systems
3. **NMEA 2000 Output**: For chart plotter integration
4. **SignalK Output**: For open marine data standard compatibility

### Versioning Strategy

- API version in every message (`api_version` field)
- Backward-compatible additions (new fields are optional)
- Breaking changes require major version bump
- Deprecation warnings for 2 minor versions before removal

---

## Appendix: Field Reference

### Track States

| State | Description |
|-------|-------------|
| `tentative` | New track, not yet confirmed (hits < min_hits) |
| `confirmed` | Active track with sufficient hits |
| `coasting` | No recent measurements, predicting only |
| `static` | Chart feature (no dynamics) |
| `dropped` | Track removed (final message only) |

### Category Values

| Category | Subcategories |
|----------|---------------|
| `vessel` | `unknown`, `cargo`, `tanker`, `passenger`, `fishing`, `sailing`, `pleasure`, `tug`, `pilot`, `military`, `sar` |
| `aid_to_navigation` | `buoy`, `beacon`, `light`, `daymark` |
| `hazard` | `rock`, `wreck`, `obstruction`, `shoal` |
| `landmark` | `tower`, `building`, `chimney` |
| `other` | `unknown`, `debris`, `whale`, `container` |

### Threat Levels

| Level | Criteria |
|-------|----------|
| `none` | CPA > warning threshold OR TCPA < 0 (diverging) |
| `watch` | CPA < warning threshold AND TCPA > 0 |
| `warning` | CPA < warning threshold AND TCPA < warning time |
| `danger` | CPA < danger threshold AND TCPA > 0 |

---

## References

- [AIS Message JSON Format](https://docs.aiscatcher.org/references/JSON-decoding/)
- [Maritime Sensor Fusion Benchmark Dataset](https://autoferry.github.io/sensor_fusion_dataset/)
- [CISE System Tracks Guidelines](https://emsa.europa.eu/cise-documentation/)
