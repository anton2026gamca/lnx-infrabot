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
  calibration: {
    yellow: {
      ranges: Array<{
        lower: [number, number, number],  // [H, S, V]
        upper: [number, number, number]   // [H, S, V]
      }>
    },
    blue: {
      ranges: Array<{
        lower: [number, number, number],
        upper: [number, number, number]
      }>
    }
  }
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
  distance_mm: number,
  goal_height_pixels: number
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

Get all detected objects in the current frame.

**Request:**
```typescript
{
  event: "get_detections",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  detections: Array<{
    object_type: string,
    x: number,
    y: number,
    width: number,
    height: number,
    confidence: number,  // 0.0 to 1.0
    color: [number, number, number]  // [R, G, B]
  }>
}
```

### `get_ball_calibration`

Get current ball color calibration ranges.

**Request:**
```typescript
{
  event: "get_ball_calibration",
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
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
  data: {}
}
```

**Response:**
```typescript
{
  status: "ok",
  focal_length_pixels: number
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
  phase: number,              // 0 = idle, 1 = phase 1, 2 = phase 2
  line_sensor_count: number,
  min_values: number[],
  max_values: number[],
  thresholds: [[number, number], ...]
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
  initial_distance_mm: number,
  line_distance_mm: number,
  samples: number,
  distance_constant: number | null
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

Update goal color and calibration ranges.

**Request (New Format - Multiple Ranges):**
```typescript
{
  event: "set_goal_settings",
  data: {
    goal_color?: "yellow" | "blue",
    calibration?: {
      yellow?: {
        ranges: Array<{
          lower: [number, number, number],  // [H, S, V]
          upper: [number, number, number]
        }>
      },
      blue?: {
        ranges: Array<{
          lower: [number, number, number],
          upper: [number, number, number]
        }>
      }
    }
  }
}
```

**Request (Legacy Format - Single Range):**
```typescript
{
  event: "set_goal_settings",
  data: {
    goal_color?: "yellow" | "blue",
    calibration?: {
      yellow?: {
        lower: [number, number, number],
        upper: [number, number, number]
      },
      blue?: {
        lower: [number, number, number],
        upper: [number, number, number]
      }
    }
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

### `set_ball_calibration`

Update ball color calibration ranges.

**Request (New Format - Multiple Ranges):**
```typescript
{
  event: "set_ball_calibration",
  data: {
    ranges: Array<{
      lower: [number, number, number],  // [H, S, V]
      upper: [number, number, number]
    }>
  }
}
```

**Request (Legacy Format - Single Range):**
```typescript
{
  event: "set_ball_calibration",
  data: {
    lower: [number, number, number],    // [H, S, V]
    upper: [number, number, number]
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
    focal_length_pixels: number  // Must be positive
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
    known_distance_mm: number  // Distance from camera to ball in millimeters
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

### `add_goal_color_range`

Add a new HSV range for goal color detection.

**Request:**
```typescript
{
  event: "add_goal_color_range",
  data: {
    goal_color: "yellow" | "blue",
    lower: [number, number, number],  // [H, S, V]
    upper: [number, number, number]   // [H, S, V]
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
    index: number  // Index of the range to remove
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
    upper: [number, number, number]   // [H, S, V]
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
    index: number  // Index of the range to remove
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
    line_distance?: number      // Expected line distance in mm (default: 200)
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
  distance_constant?: number,
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
    name: string,
    hostname?: string,
    ip_address?: string
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

### `set_bluetooth_discoverable`

Make this robot discoverable via Bluetooth, allowing other devices to find it.

**Request:**
```typescript
{
  event: "set_bluetooth_discoverable",
  data: {
    duration_seconds?: number  // Optional duration in seconds (null/omitted = indefinite, typically ~120s)
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
  discoverable?: boolean,
  error?: string
}
```

**Notes:**
- Requires root/sudo access on the Raspberry Pi
- `duration_seconds` is optional; if not specified, the device remains discoverable for ~120 seconds (system default)
- If `duration_seconds` is 0 or negative, an error is returned

### `set_bluetooth_not_discoverable`

Make this robot non-discoverable via Bluetooth, preventing other devices from finding it.

**Request:**
```typescript
{
  event: "set_bluetooth_not_discoverable",
  data: {}
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
  discoverable?: boolean,
  error?: string
}
```

**Notes:**
- Requires root/sudo access on the Raspberry Pi
- Immediately disables discoverable mode

---

## Video Streaming

### `subscribe_video`

Start receiving video frames from the robot's camera.

**Request:**
```typescript
{
  event: "subscribe_video",
  data: {
    fps?: number,              // Frames per second (default from config)
    show_detections?: boolean  // Overlay detection boxes (default: true)
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

**`video_frame`** - Binary JPEG frame data (repeated until unsubscribed)
```typescript
<bytes>  // Raw JPEG image data
```

The client receives raw JPEG bytes which can be decoded and displayed as video. Frames are sent continuously at the specified FPS rate.

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
    show_detections: true
  });
});

// Listen for sensor data changes
socket.on('important_sensor_data_change', (data) => {
  console.log('Sensor data changed:', data);
});

// Listen for video frames
socket.on('video_frame', (frameData) => {
  // frameData is binary JPEG data
  const blob = new Blob([frameData], { type: 'image/jpeg' });
  const url = URL.createObjectURL(blob);
  // Display in <img src={url} /> or canvas
});

socket.on('disconnect', () => {
  console.log('Disconnected');
});
```

---

## Configuration Constants

The following configuration values affect API behavior:

| Constant | Purpose |
|----------|---------|
| `AUTH_TOKEN` | Authentication token (base64 encoded for transmission) |
| `API_HOST` | Server host address |
| `API_PORT` | Server port number |
| `API_VIDEO_TARGET_FPS` | Default video frame rate |
| `API_VIDEO_JPEG_QUALITY` | JPEG compression quality (0-100) |

---

## Notes

- All timestamps are in seconds (Unix epoch)
- All distances in calibration are in millimeters (mm)
- All angles in degrees (0-360)
- HSV ranges: H[0-179], S[0-255], V[0-255] (OpenCV convention)
- State monitoring interval: 100ms
- Video frames are compressed as JPEG with configured quality
- Disconnected clients are automatically cleaned up after failed emission attempts
