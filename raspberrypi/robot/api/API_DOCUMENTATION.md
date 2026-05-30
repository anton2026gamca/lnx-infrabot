# Robot API Documentation

## Overview

This is a comprehensive WebSocket/HTTP API for controlling and monitoring a robot with vision capabilities. The API uses **SocketIO** for real-time bidirectional communication and **FastAPI** as the HTTP server.

**Server Details:**
- Protocol: SocketIO (WebSocket with fallback)
- Authentication: Token-based (base64 encoded) via query parameter
- Max HTTP Buffer Size: 5 MB
- CORS: Enabled for all origins

---

## Table of Contents

1. [Authentication](#authentication)
2. [Real-time Events (Subscriptions)](#real-time-events-subscriptions)
3. [Sensor & State Queries](#sensor--state-queries)
4. [Robot Control (Mutations)](#robot-control-mutations)
5. [Calibration Procedures](#calibration-procedures)
6. [Bluetooth Communication](#bluetooth-communication)
7. [Video Streaming](#video-streaming)
8. [Profiling & Performance](#profiling--performance)

---

## Authentication

All WebSocket connections require authentication via a token passed in the query string:

```
ws://host:port/socket.io/?token=<BASE64_ENCODED_TOKEN>
```

The token is base64-encoded from the configured `AUTH_TOKEN`. If authentication fails, the connection is rejected with a log warning.

---

## Real-time Events (Subscriptions)

### `subscribe_updates`

Subscribe to state change notifications.

**Request:**
```typescript
{
  event: "subscribe_updates",
  data: {
    updates: {
      mode_changed?: boolean,
      goal_color_changed?: boolean,
      important_sensor_data_change?: boolean,
      new_logs?: boolean
    }
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  message?: string,
  error?: string
}
```

**Emitted Events:**

**`mode_changed`** - Robot mode has changed
```typescript
{
  mode: "idle" | "manual" | "autonomous"
}
```

**`goal_color_changed`** - Target goal color changed
```typescript
{
  goal_color: "yellow" | "blue"
}
```

**`important_sensor_data_change`** - Critical sensor state changed (line, IR, or camera ball detection)
```typescript
{
  status: "ok",
  compass: {
    heading: number,
    pitch: number,
    roll: number
  },
  ir: {
    angle: number,
    distance: number,
    sensors: number[],
    status: string
  },
  camera_ball: {
    angle: number | null,
    distance: number | null,
    detected: boolean | null
  },
  line: {
    raw: number[],
    detected: boolean,
    thresholds: [[number, number], ...]
  },
  motors: {
    left: number,
    right: number
  },
  kicker: {
    charged: boolean,
    ready: boolean
  },
  running_state: {
    running: boolean,
    bt_module_enabled: boolean,
    bt_module_state: boolean,
    switch_state: boolean
  } | null,
  timestamp: number
}
```

**`new_logs`** - New log entries available
```typescript
{
  logs: Array<{
    id: number,
    level: string,
    message: string,
    timestamp: number
  }>
}
```

### `unsubscribe_updates`

Unsubscribe from all state change notifications.

**Request:**
```typescript
{
  event: "unsubscribe_updates",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok"
}
```

---

## Sensor & State Queries

### `get_sensor_data`

Get current sensor readings and hardware state.

**Request:**
```typescript
{
  event: "get_sensor_data",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  compass: {
    heading: number,
    pitch: number,
    roll: number
  },
  ir: {
    angle: number,
    distance: number,
    sensors: number[],
    status: string
  },
  camera_ball: {
    angle: number | null,
    distance: number | null,
    detected: boolean | null
  },
  line: {
    raw: number[],
    detected: boolean,
    thresholds: [[number, number], ...]
  },
  motors: {
    left: number,
    right: number
  },
  kicker: {
    charged: boolean,
    ready: boolean
  },
  running_state: {
    running: boolean,
    bt_module_enabled: boolean,
    bt_module_state: boolean,
    switch_state: boolean
  } | null,
  timestamp: number
}
```

### `get_logs`

Retrieve log entries since a specific ID.

**Request:**
```typescript
{
  event: "get_logs",
  data: {
    since?: number  // Log ID to retrieve logs after (default: 0)
  }
}
```

**Response:**
```typescript
{
  status: "ok",
  logs: Array<{
    id: number,
    level: string,
    message: string,
    timestamp: number
  }>,
  last_id: number
}
```

### `get_mode`

Get current robot operation mode.

**Request:**
```typescript
{
  event: "get_mode",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  mode: "idle" | "manual" | "autonomous"
}
```

### `get_motor_settings`

Get current motor control settings.

**Request:**
```typescript
{
  event: "get_motor_settings",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  rotation_correction_enabled: boolean,
  line_avoiding_enabled: boolean,
  position_based_speed_enabled: boolean
}
```

### `get_goal_settings`

Get goal detection color and calibration ranges.

**Request:**
```typescript
{
  event: "get_goal_settings",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  goal_color: "yellow" | "blue",
}
```

### `get_goal_color_calibration`

Get goal color calibration ranges used for goal detection.

**Request:**
```typescript
{
  event: "get_goal_color_calibration",
  data: {
    camera: "front" | "back"
  }
}
```

**Response:**
```typescript
{
  status: "ok"
  yellow_ranges: Array<{
    lower: [number, number, number],  // [H, S, V]
    upper: [number, number, number]   // [H, S, V]
  }>
  blue_ranges: Array<{
    lower: [number, number, number],  // [H, S, V]
    upper: [number, number, number]   // [H, S, V]
  }>
}
```

**HSV Range:**
- **H (Hue):** 0-179
- **S (Saturation):** 0-255
- **V (Value):** 0-255

### `get_goal_detection`

Get current goal detection result.

**Request:**
```typescript
{
  event: "get_goal_detection",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  goal_detected: boolean,
  alignment: number,           // -1.0 to 1.0 (center alignment)
  goal_center_x: number | null,
  goal_area: number,
  distance_mm: number | null,  // may be null if not available
  goal_height_pixels: number,
  camera_yaw_deg: number,
  goals_by_color: {
    yellow: {
      goal_detected: boolean,
      alignment: number,
      goal_center_x: number | null,
      goal_area: number,
      distance_mm: number | null,
      goal_height_pixels: number,
      camera_yaw_deg: number,
    },
    blue: {
      goal_detected: boolean,
      alignment: number,
      goal_center_x: number | null,
      goal_area: number,
      distance_mm: number | null,
      goal_height_pixels: number,
      camera_yaw_deg: number,
    }
  },
  enemy_goal_color: "yellow" | "blue",
  own_goal_color: "yellow" | "blue",
}
```

### `get_position_estimate`

Get estimated robot position on the field.

**Request:**
```typescript
{
  event: "get_position_estimate",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  x_mm: number | null,
  y_mm: number | null,
  confidence: number  // 0.0 to 1.0
}
```

### `get_detections`

Get detected objects, optionally filtered by camera.

**Request:**
```typescript
{
  event: "get_detections",
  data: {
    camera?: "front" | "back" | "both" // Default: "both"
  }
}
```

**Response:**
```typescript
{
  status: "ok",
  camera: "front" | "back" | "both",
  detections: {
    front?: Array<{
      object_type: string,
      x: number,
      y: number,
      width: number,
      height: number,
      confidence: number,
      color: [number, number, number], // [B, G, R]
      camera: "front" | "back" | null
    }>,
    back?: Array<{
      object_type: string,
      x: number,
      y: number,
      width: number,
      height: number,
      confidence: number,
      color: [number, number, number], // [B, G, R]
      camera: "front" | "back" | null
    }>
  }
}
```

### `get_ball_calibration`

Get current ball color calibration ranges.

**Request:**
```typescript
{
  event: "get_ball_calibration",
  data: {
    camera?: "front" | "back" // Default: "front"
  }
}
```

**Response:**
```typescript
{
  status: "ok",
  camera: "front" | "back",
  ranges: Array<{
    lower: [number, number, number],  // [H, S, V]
    upper: [number, number, number]   // [H, S, V]
  }>
}
```

### `get_goal_focal_length`

Get the focal length used for distance calculations.

**Request:**
```typescript
{
  event: "get_goal_focal_length",
  data: {
    camera?: "front" | "back" // Default: "front"
  }
}
```

**Response:**
```typescript
{
  status: "ok",
  camera: "front" | "back",
  focal_length_pixels: number
}
```

### `get_camera_settings`

Get current manual camera control values (persisted calibration values).

**Request:**
```typescript
{
  event: "get_camera_settings",
  data: {
    camera?: "front" | "back" | "both" // Default: "both"
  }
}
```

**Response:**
```typescript
{
  status: "ok",
  camera: "front" | "back" | "both",
  settings: {
    front?: {
      camera: "front",
      color_gains: [number, number], // [red_gain, blue_gain]
      exposure_time: number,          // microseconds
      analogue_gain: number
    },
    back?: {
      camera: "back",
      color_gains: [number, number], // [red_gain, blue_gain]
      exposure_time: number,          // microseconds
      analogue_gain: number
    }
  } | {
    camera: "front" | "back",
    color_gains: [number, number], // [red_gain, blue_gain]
    exposure_time: number,          // microseconds
    analogue_gain: number
  }
}
```

### `get_all_state_machines`

Get list of available autonomous state machines.

**Request:**
```typescript
{
  event: "get_all_state_machines",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  state_machines: string[]
}
```

### `get_autonomous_state`

Get current autonomous mode settings.

**Request:**
```typescript
{
  event: "get_autonomous_state",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  state_machine: string | null,
  always_face_goal_enabled: boolean,
  camera_ball_usage_enabled: boolean
}
```

### `get_line_calibration_status`

Get current line calibration status.

**Request:**
```typescript
{
  event: "get_line_calibration_status",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  active: boolean,                         // true when calibration is running
  phase: number,                           // 0 = idle, 1 = phase 1, 2 = phase 2
  current_thresholds: Array<[number, number]>,
  calibration_min: Array<number | null> | null,   // current phase min values
  calibration_max: Array<number | null> | null,   // current phase max values
  phase1_complete: boolean,
  phase1_min: Array<number | null> | null,
  phase1_max: Array<number | null> | null,
  phase2_complete: boolean,
  phase2_min: Array<number | null> | null,
  phase2_max: Array<number | null> | null
}
```

### `get_goal_distance_calibration_status`

Get goal distance calibration status.

**Request:**
```typescript
{
  event: "get_goal_distance_calibration_status",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  active: boolean,
  phase?: "initial" | "driving",
  initial_distance_mm?: number,
  line_distance_mm?: number,
  initial_height_pixels?: number | null,
  line_height_pixels?: number | null,
  camera?: "front" | "back"
}
```

---

## Robot Control (Mutations)

### `set_mode`

Change robot operation mode.

**Request:**
```typescript
{
  event: "set_mode",
  data: {
    mode: "idle" | "manual" | "autonomous"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

### `set_manual_control`

Send manual movement and rotation commands.

**Request:**
```typescript
{
  event: "set_manual_control",
  data: {
    move: {
      angle: number,    // Direction angle in degrees (0-360)
      speed: number     // Speed percentage (0-100)
    },
    rotate: number      // Rotation speed (-100 to 100)
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

### `reset_compass`

Reset compass heading to 0 degrees.

**Request:**
```typescript
{
  event: "reset_compass",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

### `set_motor_settings`

Update motor control settings.

**Request:**
```typescript
{
  event: "set_motor_settings",
  data: {
    rotation_correction_enabled?: boolean,
    line_avoiding_enabled?: boolean,
    position_based_speed_enabled?: boolean
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

### `set_goal_settings`

Update goal settings

**Request:**
```typescript
{
  event: "set_goal_settings",
  data: {
    goal_color?: "yellow" | "blue",
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

### `set_goal_color_calibration`

Update goal color calibration ranges

**Request:**
```typescript
{
  event: "set_goal_color_calibration"
  camera: "front" | "back" | "both"
  yellow_ranges?: Array<{
    lower: [number, number, number],  // [H, S, V]
    upper: [number, number, number]
  }>
  blue_ranges?: Array<{
    lower: [number, number, number],
    upper: [number, number, number]
  }>
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

### `set_ball_calibration`

Update ball color calibration ranges.

**Request:**
```typescript
{
  event: "set_ball_calibration",
  data: {
    camera?: "front" | "back" | "both", // Default: "both"
    ranges: Array<{
      lower: [number, number, number],  // [H, S, V]
      upper: [number, number, number]
    }>
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

### `set_goal_focal_length`

Set the focal length for goal distance calculations.

**Request:**
```typescript
{
  event: "set_goal_focal_length",
  data: {
    focal_length_pixels: number,  // Must be positive
    camera?: "front" | "back" | "both" // Default: "both"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  focal_length_pixels?: number,
  error?: string
}
```

### `set_camera_settings`

Manually set camera control values and apply them immediately. These values are persisted to calibration storage.

**Request:**
```typescript
{
  event: "set_camera_settings",
  data: {
    camera?: "front" | "back" | "both", // Default: "both"
    color_gains?: [number, number],     // [red_gain, blue_gain], positive values
    exposure_time?: number,             // microseconds, positive
    analogue_gain?: number              // positive
  }
}
```

**Note:** Provide at least one of `color_gains`, `exposure_time`, or `analogue_gain`.

**Response:**
```typescript
{
  status: "ok" | "error",
  camera?: "front" | "back" | "both",
  settings?: {
    front?: {
      camera: "front",
      color_gains: [number, number],
      exposure_time: number,
      analogue_gain: number
    },
    back?: {
      camera: "back",
      color_gains: [number, number],
      exposure_time: number,
      analogue_gain: number
    }
  },
  error?: string
}
```

### `set_autonomous_state`

Configure autonomous mode settings.

**Request:**
```typescript
{
  event: "set_autonomous_state",
  data: {
    state_machine?: string,
    always_face_goal_enabled?: boolean,
    camera_ball_usage_enabled?: boolean
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

### `set_line_thresholds`

Set line sensor detection thresholds.

**Request:**
```typescript
{
  event: "set_line_thresholds",
  data: {
    thresholds: Array<[number, number]>  // [min, max] for each line sensor
  }
}
```

**Note:** The `thresholds` array must have exactly `LINE_SENSOR_COUNT` pairs.

**Response:**
```typescript
{
  status: "ok" | "error",
  thresholds?: Array<[number, number]>,
  error?: string
}
```

---

## Calibration Procedures

### `camera_ball_distance_calibration`

Calibrate ball distance detection. Place ball at a known distance and call this endpoint.

**Request:**
```typescript
{
  event: "camera_ball_distance_calibration",
  data: {
    known_distance_mm: number,  // Distance from camera to ball in millimeters
    camera?: "front" | "back"   // Default: "front"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  calibration_constant?: number,
  error?: string
}
```

### `camera_auto_calibration`

Temporarily enable camera AWB and AE so the camera can adapt to current lighting, then disable both again and copy the learned values to all other cameras. The resulting gains/exposure values are also saved to calibration storage.

**Request:**
```typescript
{
  event: "camera_auto_calibration",
  data: {
    camera?: "front" | "back", // Default: "front"
    settle_time_s?: number              // Seconds to keep AWB/AE enabled (default: 2.0)
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  camera?: "front" | "back" | "both",
  result?: {
    color_gains: [number, number],    // [red_gain, blue_gain]
    exposure_time: number | null,     // microseconds
    analogue_gain: number | null,
    settle_time_s: number
  },
  error?: string
}
```

### `add_goal_color_range`

Add a new HSV range for goal color detection.

**Request:**
```typescript
{
  event: "add_goal_color_range",
  data: {
    goal_color: "yellow" | "blue",
    lower: [number, number, number],  // [H, S, V]
    upper: [number, number, number],  // [H, S, V]
    camera?: "front" | "back" | "both" // Default: "both"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  ranges?: Array<{
    lower: [number, number, number],
    upper: [number, number, number]
  }>,
  error?: string
}
```

### `remove_goal_color_range`

Remove a goal color range by index.

**Request:**
```typescript
{
  event: "remove_goal_color_range",
  data: {
    goal_color: "yellow" | "blue",
    index: number,  // Index of the range to remove
    camera?: "front" | "back" | "both" // Default: "both"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  ranges?: Array<{
    lower: [number, number, number],
    upper: [number, number, number]
  }>,
  error?: string
}
```

### `add_ball_color_range`

Add a new HSV range for ball color detection.

**Request:**
```typescript
{
  event: "add_ball_color_range",
  data: {
    lower: [number, number, number],  // [H, S, V]
    upper: [number, number, number],  // [H, S, V]
    camera?: "front" | "back" | "both" // Default: "both"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  ranges?: Array<{
    lower: [number, number, number],
    upper: [number, number, number]
  }>,
  error?: string
}
```

### `remove_ball_color_range`

Remove a ball color range by index.

**Request:**
```typescript
{
  event: "remove_ball_color_range",
  data: {
    index: number,  // Index of the range to remove
    camera?: "front" | "back" | "both" // Default: "both"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  ranges?: Array<{
    lower: [number, number, number],
    upper: [number, number, number]
  }>,
  error?: string
}
```

### `start_line_calibration`

Begin line sensor calibration.

**Request:**
```typescript
{
  event: "start_line_calibration",
  data: {
    phase: 1 | 2  // Calibration phase (default: 1)
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  phase?: number,
  message?: string,
  error?: string
}
```

**Phases:**
- **Phase 1:** Calibrate with white area
- **Phase 2:** Calibrate with black line

### `stop_line_calibration`

Stop line calibration and apply learned thresholds.

**Request:**
```typescript
{
  event: "stop_line_calibration",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  phase?: number,
  thresholds?: Array<[number, number]>,
  min_values?: number[],
  max_values?: number[],
  can_start_phase2?: boolean,
  error?: string
}
```

### `cancel_line_calibration`

Cancel ongoing line calibration without applying changes.

**Request:**
```typescript
{
  event: "cancel_line_calibration",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  phase?: number,
  message?: string,
  error?: string
}
```

### `start_goal_distance_calibration`

Begin goal distance calibration. Drive robot toward goal until it hits the line.

**Request:**
```typescript
{
  event: "start_goal_distance_calibration",
  data: {
    initial_distance?: number,  // Initial distance in mm (default: 200)
    line_distance?: number,     // Expected line distance in mm (default: 200)
    camera?: "front" | "back"   // Default: "front"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  message?: string,
  error?: string
}
```

### `stop_goal_distance_calibration`

Stop goal distance calibration and save results.

**Request:**
```typescript
{
  event: "stop_goal_distance_calibration",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  focal_length_pixels?: number,
  camera?: "front" | "back",
  message?: string,
  error?: string
}
```

### `cancel_goal_distance_calibration`

Cancel goal distance calibration without saving.

**Request:**
```typescript
{
  event: "cancel_goal_distance_calibration",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  message?: string,
  error?: string
}
```

### `compute_hsv_from_regions`

Analyze selected image regions and compute HSV ranges.

**Request:**
```typescript
{
  event: "compute_hsv_from_regions",
  data: {
    camera?: "front" | "back",  // Default: "front"
    regions: Array<{
      x: number,       // X coordinate
      y: number,       // Y coordinate
      width: number,   // Region width in pixels
      height: number   // Region height in pixels
    }>
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  lower?: [number, number, number],   // [H, S, V] minimum
  upper?: [number, number, number],   // [H, S, V] maximum
  error?: string
}
```

**Algorithm:**
- Extracts HSV values from all specified regions
- Computes 5th-95th percentile with margins:
  - Hue: ±5
  - Saturation: ±20
  - Value: ±20
- Returns union of all regions

---

## Bluetooth Communication

Bluetooth is handled by a dedicated background process. The API exposes state and messaging helpers for robot-to-robot communication.

### `get_bluetooth_state`

Get Bluetooth process status, local device identity, connected devices, paired devices, and selected peer robot metadata.

**Request:**
```typescript
{
  event: "get_bluetooth_state",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  bluetooth_enabled: boolean,
  process_alive: boolean,
  local_device: {
    device_id?: string,
    hostname?: string,
    ip_address?: string
  },
  connected_devices: Array<{
    name: string,
    mac_address: string,
    hostname?: string,
    ip_address?: string,
    last_connected?: number,
    is_connected: boolean,
    device_id?: string
  }>,
  paired_devices: Array<{
    name: string,
    mac_address: string,
    hostname?: string,
    ip_address?: string,
    last_connected?: number,
    is_connected: boolean,
    device_id?: string
  }>,
  other_robot: {
    mac_address?: string,
    name?: string,
    hostname?: string,
    ip_address?: string,
    note?: string
  }
}
```

### `set_bluetooth_enabled`

Enable or disable Bluetooth usage. When disabled, auto-connect is paused and active connections are closed.

**Request:**
```typescript
{
  event: "set_bluetooth_enabled",
  data: {
    enabled: boolean
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  bluetooth_enabled?: boolean,
  error?: string
}
```

### `set_other_robot`

Set or clear metadata for the selected "other robot".

**Request (set):**
```typescript
{
  event: "set_other_robot",
  data: {
    mac_address: string,
    name?: string,
    hostname?: string,
    ip_address?: string,
    note?: string
  }
}
```

**Request (clear):**
```typescript
{
  event: "set_other_robot",
  data: {
    clear: true
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  other_robot?: object,
  error?: string
}
```

### `bluetooth_connect_other_robot`

Connect to the selected robot (or explicit `mac_address`).

**Request:**
```typescript
{
  event: "bluetooth_connect_other_robot",
  data: {
    mac_address?: string
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  result?: {
    command_id: number,
    success: boolean,
    data: object,
    error?: string,
    timestamp: number
  },
  connected_devices?: object[],
  error?: string
}
```

### `bluetooth_disconnect_other_robot`

Disconnect from the selected robot (or explicit `mac_address`).

**Request:**
```typescript
{
  event: "bluetooth_disconnect_other_robot",
  data: {
    mac_address?: string
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  result?: {
    command_id: number,
    success: boolean,
    data: object,
    error?: string,
    timestamp: number
  },
  connected_devices?: object[],
  error?: string
}
```

### `bluetooth_send_message`

Send a custom Bluetooth message to the selected robot or an explicit `mac_address`.

**Request:**
```typescript
{
  event: "bluetooth_send_message",
  data: {
    mac_address?: string,
    message_type: string,
    content: string
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  result?: {
    command_id: number,
    success: boolean,
    data: {
      mac_address: string,
      message_id: string
    },
    error?: string,
    timestamp: number
  },
  error?: string
}
```

### `get_bluetooth_messages`

Get sent and received Bluetooth message history.

**Request:**
```typescript
{
  event: "get_bluetooth_messages",
  data: {
    clear?: boolean, // default false
    limit?: number   // optional last N messages
  }
}
```

**Response:**
```typescript
{
  status: "ok",
  received: Array<{
    message_type: string,
    content: string,
    sender_id?: string,
    sender_mac?: string,
    timestamp?: number,
    message_id?: string
  }>,
  sent: Array<{
    message_type: string,
    content: string,
    sender_id?: string,
    target_mac?: string,
    timestamp?: number,
    message_id?: string
  }>
}
```

### `bluetooth_list_pairable_devices`

List nearby discoverable Bluetooth devices that are available for pairing.

**Request:**
```typescript
{
  event: "bluetooth_list_pairable_devices",
  data: {
    timeout_seconds?: number  // Scan duration in seconds (default: 6, must be > 0)
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  result?: {
    command_id: number,
    success: boolean,
    data: {
      devices: Array<{
        name: string,
        mac_address: string,
        is_paired: boolean
      }>,
      timeout_seconds: number
    },
    error?: string,
    timestamp: number
  },
  devices?: Array<{
    name: string,
    mac_address: string,
    is_paired: boolean
  }>,
  error?: string
}
```

**Notes:**
- Requires `bluetoothctl` and sufficient permissions on the Raspberry Pi
- `timeout_seconds` controls how long active discovery runs before returning results
- `is_paired` indicates whether the discovered device already exists in the saved paired devices list

### `bluetooth_pair_device`

Pair a new Bluetooth device and store its metadata.

**Request:**
```typescript
{
  event: "bluetooth_pair_device",
  data: {
    mac_address: string,
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  result?: {
    command_id: number,
    success: boolean,
    data: object,
    error?: string,
    timestamp: number
  },
  paired_devices?: Array<{
    name: string,
    mac_address: string,
    hostname?: string,
    ip_address?: string,
    last_connected?: number,
    is_connected: boolean,
    device_id?: string
  }>,
  error?: string
}
```

### `bluetooth_unpair_device`

Unpair a previously paired Bluetooth device.

**Request:**
```typescript
{
  event: "bluetooth_unpair_device",
  data: {
    mac_address: string
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  result?: {
    command_id: number,
    success: boolean,
    data: object,
    error?: string,
    timestamp: number
  },
  paired_devices?: Array<{
    name: string,
    mac_address: string,
    hostname?: string,
    ip_address?: string,
    last_connected?: number,
    is_connected: boolean,
    device_id?: string
  }>,
  error?: string
}
```

### `set_bluetooth_pairing_mode`

Set Bluetooth pairing mode (enable or disable discoverability).

**Request:**
```typescript
{
  event: "set_bluetooth_pairing_mode",
  data: {
    enabled: boolean  // true to enable pairing mode, false to disable
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  result?: {
    command_id: number,
    success: boolean,
    data: {
      pairing_mode_enabled: boolean
    },
    error?: string,
    timestamp: number
  },
  pairing_mode_enabled?: boolean,
  error?: string
}
```

---

## Video Streaming

### `subscribe_video`

Start receiving video frames from the robot cameras.

**Request:**
```typescript
{
  event: "subscribe_video",
  data: {
    fps?: number,              // Frames per second (default from config)
    show_detections?: boolean, // Overlay detection boxes on the streamed camera(s) (default: true)
    camera?: "front" | "back" | "both" // Default: "both"
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  message?: string,
  error?: string
}
```

**Emitted Events:**

**`video_frame_front`** - Binary JPEG frame data from the front camera
```typescript
<bytes>  // Raw JPEG image data
```

**`video_frame_back`** - Binary JPEG frame data from the back camera
```typescript
<bytes>  // Raw JPEG image data
```

When `camera: "both"` is used, the server emits both `video_frame_front` and `video_frame_back`.

### `unsubscribe_video`

Stop receiving video frames.

**Request:**
```typescript
{
  event: "unsubscribe_video",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  error?: string
}
```

---

## Profiling & Performance

The API provides real-time control over profiling data collection. Profiling tracks function execution times, lock contention, and process metrics across all robot processes. Use the profiling endpoints to enable/disable collection at runtime, retrieve performance metrics, and analyze bottlenecks.

### `profiling_start`

Start collecting profiling data.

**Request:**
```typescript
{
  event: "profiling_start",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  message: string,
  error?: string
}
```

**Description:** Initializes the profiling collector and begins recording function execution times, lock contention events, and process metrics. Previous collected data is cleared when starting a new collection session.

### `profiling_stop`

Stop collecting profiling data without clearing accumulated data.

**Request:**
```typescript
{
  event: "profiling_stop",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  message: string,
  error?: string
}
```

**Description:** Stops the profiler from collecting new events. Data collected so far remains available for retrieval via `profiling_report`.

### `profiling_status`

Get current profiling status and statistics.

**Request:**
```typescript
{
  event: "profiling_status",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  is_collecting: boolean,
  total_function_events: number,
  total_lock_events: number,
  total_processes: number,
  collection_duration: number  // seconds
}
```

**Description:** Returns the current profiling state including whether collection is active and basic statistics about collected data.

### `profiling_report`

Get detailed profiling report with metrics and statistics.

**Request:**
```typescript
{
  event: "profiling_report",
  data: {
    include_stack_traces?: boolean  // Default: false (stack traces omitted to reduce network size)
  }
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  report?: {
    metadata: {
      collection_duration: number,
      start_time: number,
      end_time: number,
      total_function_events: number,
      total_lock_events: number,
      total_processes: number,
      is_collecting: boolean
    },
    processes: {
      [process_name]: {
        process_name: string,
        process_id: number,
        start_time: number,
        stop_time: number,
        function_count: number,
        lock_events_count: number
      }
    },
    functions: {
      by_name: {
        [function_name]: {
          count: number,
          total_time: number,
          min_time: number,
          max_time: number,
          avg_time: number,
          name: string
        }
      },
      sorted_by_total_time: [...]  // Top functions by total execution time
    },
    locks: {
      by_name: {
        [lock_name]: {
          acquire_count: number,
          total_wait_time: number,
          max_wait_time: number,
          contentions: number
        }
      },
      sorted_by_contention: [...]  // Locks sorted by contention count
    },
    timeline: {
      processes: [...],       // Process lifecycle events (max 1000)
      functions: [...],       // Function call events (max 1000)
      locks: [...]            // Lock events (max 1000)
    }
  },
  error?: string
}
```

**Description:** Returns comprehensive profiling data including:
- **Metadata:** Collection duration and event counts
- **Processes:** Per-process statistics and lifecycle
- **Functions:** Execution time statistics for all profiled functions
- **Locks:** Lock contention and wait time statistics
- **Timeline:** Time-series event data

**Data Fields Explanation:**
- `count` - Number of times the event occurred
- `total_time` - Sum of all durations (seconds)
- `avg_time` - Average time per occurrence
- `min_time`, `max_time` - Minimum and maximum durations
- `contentions` - Number of times a lock was contested
- `total_wait_time` - Total time processes waited for this lock
- `max_wait_time` - Maximum single wait duration

### `profiling_clear`

Clear all collected profiling data and reset the collector.

**Request:**
```typescript
{
  event: "profiling_clear",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok" | "error",
  message: string,
  error?: string
}
```

**Description:** Clears all collected profiling data. After clearing, the profiler remains in its current state (collecting or stopped). Start a new session with `profiling_start` to begin fresh data collection.

---

## Error Responses

All error responses follow this format:

```typescript
{
  status: "error",
  error: string  // Human-readable error message
}
```

Common error scenarios:
- **Invalid authentication:** Connection rejected before handshake
- **Internal server error:** Server-side exception occurred
- **Invalid parameters:** Request data doesn't match expected format
- **Invalid state:** Operation not allowed in current state (e.g., stopping when not started)

---

## Connection Example (JavaScript/TypeScript)

```typescript
import io from 'socket.io-client';

const token = btoa('your_auth_token');  // Base64 encode the token
const socket = io('ws://robot-ip:port', {
  query: {
    token: token
  }
});

socket.on('connect', () => {
  console.log('Connected');
  
  // Subscribe to updates
  socket.emit('subscribe_updates', {
    updates: {
      mode_changed: true,
      important_sensor_data_change: true,
      new_logs: true
    }
  }, (response) => {
    console.log('Subscription response:', response);
  });
  
  // Subscribe to video
  socket.emit('subscribe_video', {
    fps: 30,
    show_detections: true,
    camera: 'both'
  });
});

// Listen for sensor data changes
socket.on('important_sensor_data_change', (data) => {
  console.log('Sensor data changed:', data);
});

socket.on('video_frame_front', (frameData) => {
  // Front camera (binary JPEG data)
  const blob = new Blob([frameData], { type: 'image/jpeg' });
  const url = URL.createObjectURL(blob);
  // Display in <img src={url} /> or canvas
});

socket.on('video_frame_back', (frameData) => {
  // Back camera (binary JPEG data)
  const blob = new Blob([frameData], { type: 'image/jpeg' });
  const url = URL.createObjectURL(blob);
  // Display in <img src={url} /> or canvas
});

socket.on('disconnect', () => {
  console.log('Disconnected');
});
```

---

## Notes

- All timestamps are in seconds (Unix epoch)
- All distances in calibration are in millimeters (mm)
- All angles in degrees (0-360)
- HSV ranges: H (0-179), S (0-255), V (0-255) (OpenCV convention)
- State monitoring interval: 100ms
- Video frames are compressed as JPEG with configured quality
- Disconnected clients are automatically cleaned up after failed emission attempts
