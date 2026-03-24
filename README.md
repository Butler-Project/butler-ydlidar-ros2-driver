# tmini_driver_node

Lightweight ROS2 driver for YDLidar TMini family (TMini, TMini Plus, TMini Pro). Replaces the full YDLidar SDK (~100 files, 45MB) with ~500 lines of C++20 and zero external dependencies.

## Supported hardware

| Model | Code | Sensor type | Baud | Verified |
|-------|------|-------------|------|----------|
| TMini | 140 | Triangulation | 230400 | No |
| TMini Pro | 150 | ToF | 230400 | No |
| TMini Plus | 151 | ToF | 230400 | Yes |

All models use the same triangle packet format over serial (3 bytes/sample with intensity). Model code is auto-detected from the device at startup.

## Usage

```bash
# Build
colcon build --packages-select tmini_driver_node

# Run standalone
ros2 launch tmini_driver_node tmini.launch.py

# Run with Foxglove Bridge for visualization
ros2 launch tmini_driver_node tmini_debug.launch.py
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Laser scan at ~10 Hz, SensorDataQoS |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/lidar` | Serial port device path |
| `frame_id` | string | `lidar` | TF frame ID for scan messages |
| `baud_rate` | int | `230400` | Serial baud rate |
| `angle_min` | double | `-3.14159` | Minimum scan angle (rad) |
| `angle_max` | double | `3.14159` | Maximum scan angle (rad) |
| `range_min` | double | `0.03` | Minimum range filter (m) |
| `range_max` | double | `12.0` | Maximum range filter (m) |
| `invalid_range_is_inf` | bool | `false` | Use inf (vs 0) for invalid ranges |
| `enable_motor_pid` | bool | `false` | Enable scan frequency PID (not yet implemented) |
| `target_scan_freq_hz` | double | `10.0` | Target scan frequency for PID |

## Architecture

```
tmini_driver_node/
├── include/tmini_driver_node/
│   └── tmini_lidar.hpp          # Protocol constants, structs, TminiLidar class
├── src/
│   ├── tmini_lidar.cpp          # Serial I/O + packet parser (ROS-independent)
│   └── tmini_node.cpp           # ROS2 node wrapper (component + standalone)
├── params/tmini.yaml
├── launch/
│   ├── tmini.launch.py          # Lidar only
│   └── tmini_debug.launch.py    # Lidar + Foxglove Bridge
├── CMakeLists.txt
└── package.xml
```

The lidar library (`tmini_lidar`) has no ROS dependency and can be used standalone. The ROS node is a thin wrapper that converts `tmini::Scan` into `LaserScan` messages.

### Threading model

- **Reader thread** (`std::jthread`): Runs `reader_loop()`, reads serial data, parses packets, accumulates samples into scans. On scan completion (ring-start packet), delivers `Scan` via callback.
- **Main thread**: ROS2 spin. The scan callback publishes directly from the reader thread (thread-safe in rclcpp).

### Component support

Registered as `TminiDriverNode` via `rclcpp_components`. Can be loaded into a composable container alongside other nodes (e.g., odometry, IMU) or run as a standalone process.

## Protocol reference

Based on the YDLidar triangle protocol. All TMini models use this format regardless of physical sensor type (ToF vs triangulation).

### Serial configuration

230400 baud, 8N1, no flow control. Linux termios, no external serial library.

### Command format

```
[0xA5] [CMD]                          — no payload
[0xA5] [CMD|0x80] [SIZE] [DATA] [XOR] — with payload
```

| Command | Code | Description |
|---------|------|-------------|
| Scan | `0x60` | Start continuous scanning |
| Stop | `0x65` | Stop scanning |
| Force Stop | `0x00` | Force stop |
| Device Info | `0x90` | Query model, firmware, serial number |
| Health | `0x92` | Query device health status |

### Response header

```
[0xA5] [0x5A] [SIZE:30|SUBTYPE:2 as LE uint32] [TYPE]
```

TYPE: `0x04` = device info, `0x06` = health, `0x81` = measurement (continuous).

### Scan packet format (10-byte header + N * 3-byte samples)

```
Byte  0-1:  0xAA 0x55              sync word
Byte  2:    CT                     bit0 = ring_start, bits1-7 = freq * 10
Byte  3:    COUNT                  samples in this packet (1-80)
Byte  4-5:  FIRST_ANGLE            uint16 LE, bit0 = checkbit (must be 1), bits1-15 = angle
Byte  6-7:  LAST_ANGLE             uint16 LE, same encoding
Byte  8-9:  CHECKSUM               uint16 LE, XOR checksum
Byte 10+:   SAMPLES                COUNT * 3 bytes
```

### Sample format (3 bytes, triangle with intensity)

```
Byte 0:     QUAL                   8-bit intensity (lower 8 bits of 10-bit value)
Byte 1-2:   DIST                   uint16 LE
            DIST bits 0-1:         intensity bits 8-9 (MSBs of 10-bit intensity)
            DIST bits 2-15:        distance in quarter-mm units
```

**10-bit intensity** = `(DIST[1:0] << 8) | QUAL`

**Distance in meters** = `(DIST & 0xFFFC) / 4.0 / 1000.0`

### Angle encoding

Raw angle values are in units of 1/64 degree (0-23040 = 0-360 degrees).

```
angle_degrees = raw_angle / 64.0
```

The checkbit (bit 0) must be 1 for valid angles. Actual angle = raw >> 1.

Per-sample angle is linearly interpolated between FIRST_ANGLE and LAST_ANGLE:

```
sample_angle = first_angle + (last_angle - first_angle) / (count - 1) * index
```

Wrap-around handling: if last < first and first > 270deg and last < 90deg, add 360deg to last.

### Checksum algorithm

XOR-based, computed over uint16 words:

```
cs  = 0x55AA                        (sync word)
cs ^= FIRST_ANGLE                   (raw, with checkbit)
cs ^= each sample's qual byte       (XOR'd as uint16 with upper byte 0)
cs ^= each sample's dist uint16
cs ^= (CT | COUNT<<8)               (bytes 2-3 as uint16)
cs ^= LAST_ANGLE                    (raw, with checkbit)
```

### Model detection

The `get_device_info` response contains a model byte:
- 140 = TMini (triangulation)
- 150 = TMini Pro (ToF)
- 151 = TMini Plus (ToF)

`is_tof()` returns true for 150/151. Currently only affects logging; all models use identical packet format and distance encoding (quarter-mm).

### Startup sequence

1. Open serial port
2. Flush + send Stop (clear any previous scan)
3. `get_device_info` (0x90) — read model code
4. `get_device_health` (0x92) — verify status != error
5. `start_scan` (0x60) — read response header (type 0x81)
6. Launch reader thread — sync to 0xAA55, parse packets continuously

## Timestamping for motion compensation

The protocol provides no hardware timestamps. The driver records wall-clock times:

- `stamp_start`: captured when the ring-start packet (CT bit0 = 1) is received
- `stamp_end`: captured when the next ring-start arrives (completing the previous scan)

The node computes per-sample timing:
- `scan_time = stamp_end - stamp_start`
- `time_increment = scan_time / num_samples`
- `header.stamp` = wall time at scan start (converted from `steady_clock` to `system_clock`)

For motion compensation, each sample's timestamp can be reconstructed as:
```
t_sample = header.stamp + time_increment * sample_index
```

## Diagnostics

The reader thread logs packet statistics every 5 seconds:
```
diag: ok=500 cs_fail=0 sync_fail=0 scans=50 bps=3
```

- `ok`: packets parsed successfully
- `cs_fail`: checksum failures (first 5 are logged with details)
- `sync_fail`: sync/header read timeouts
- `scans`: complete scans delivered to ROS
- `bps`: bytes per sample (always 3 for TMini family)

## Not implemented (future work)

### Motor speed PID
Config fields `enable_motor_pid` and `target_scan_freq_hz` exist but PID control is not wired up. The lidar reports its current scan frequency via the CT byte (`freq_hz = (ct >> 1) * 0.1`). YDLidar provides commands to adjust speed:
- `0x0B` (AIMSPEED_ADD) — increase target speed
- `0x0C` (AIMSPEED_DIS) — decrease target speed
- `0x09` (AIMSPEED_ADDMIC) — fine increase
- `0x0A` (AIMSPEED_DISMIC) — fine decrease

A simple P or PI controller comparing `scan.scan_freq_hz` to `target_freq` and sending add/dis commands would suffice.

### Auto-reconnect
If the serial port disconnects, the reader thread will fail silently. Currently requires a node restart. Could be added by catching read failures in `reader_loop` and re-running the startup sequence.

### ROS2 services
The old driver had `start_scan` / `stop_scan` services. Not ported. Can be added by exposing `lidar_->stop()` / `lidar_->start()` through service callbacks.

### Angle correction for triangulation models
`triangle_angle_correction()` is implemented but not called. TMini (model 140) and TMini Plus/Pro (150/151) do not need 2nd-order angle correction per the SDK. The function exists for potential future support of other YDLidar triangle models (X4, S4, etc.) which use the same packet format but require:
```
correction = atan((21.8 * (155.3 - d)) / (155.3 * d)) * 180/pi * 64
```
where `d = dist_raw / 4.0`.

### PointCloud output
The old driver published `sensor_msgs/msg/PointCloud`. Not ported — the nav stack uses `LaserScan`. Trivial to add if needed.
