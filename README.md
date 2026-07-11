# storm_teleop

ROS 2 teleop stack for a 6-wheel skid-steer URC rover:

```
Xbox joystick → joy_node → teleop_twist_joy → /cmd_vel
    → diff_drive_controller (ros2_control, 50 Hz)
    → SparkCanHardware plugin → sparkcan / SocketCAN (can0)
    → 6× REV SPARK MAX (onboard velocity PID) → NEO V1.1 motors
```

Each SPARK MAX runs its own onboard velocity PID, so the plugin sends velocity
setpoints (not duty cycle) and the control loop stays fast regardless of the
50 Hz ros2_control rate.

## Dependencies

- ROS 2 (ament_cmake) with `ros2_control`, `diff_drive_controller`,
  `joint_state_broadcaster`, `joy`, `teleop_twist_joy`,
  `robot_state_publisher`, `xacro`
- [`sparkcan`](https://github.com/grayson-arendt/sparkcan) — installed as a
  system library (headers in `/usr/local/include`, `libsparkcan.so` in
  `/usr/local/lib`)
- `yaml-cpp` — used by the plugin to parse `config/sparks.yaml`

## Hardware / CAN setup

The 6 SPARK MAXs are addressed by CAN ID (set in `description/rover.urdf.xacro`):

| CAN ID | Joint                    | Side  | Inverted |
|--------|--------------------------|-------|----------|
| 1      | back_right_wheel_joint   | right | no       |
| 2      | middle_right_wheel_joint | right | no       |
| 3      | front_right_wheel_joint  | right | no       |
| 4      | front_left_wheel_joint   | left  | yes      |
| 5      | middle_left_wheel_joint  | left  | yes      |
| 6      | back_left_wheel_joint    | left  | yes      |

### Bring up `can0`

Use the provided script (idempotent; sets `restart-ms` for automatic bus-off
recovery). Adjust bitrate to your bus via env vars:

```bash
sudo scripts/can0_up.sh                       # can0 @ 1 Mbit/s
sudo IFACE=can0 BITRATE=500000 scripts/can0_up.sh
```

To bring `can0` up automatically at boot, install the systemd unit:

```bash
sudo cp systemd/storm-can0.service /etc/systemd/system/
sudo cp scripts/can0_up.sh /usr/local/bin/can0_up.sh
sudo chmod +x /usr/local/bin/can0_up.sh
sudo systemctl daemon-reload
sudo systemctl enable --now storm-can0.service
```

Verify: `ip -details link show can0` → shows `UP`, the bitrate, and `restart-ms`.

## Build

```bash
cd ~/ros2_ws          # your colcon workspace
colcon build --packages-select storm_teleop
source install/setup.bash
```

## Run

```bash
ros2 launch storm_teleop teleop.launch.py
```

This starts `robot_state_publisher`, `ros2_control_node`, the
`diff_drive_controller` + `joint_state_broadcaster` spawners, `joy_node`, and
`teleop_twist_joy`.

## Controller mapping (wired Xbox, xpad driver)

- **Hold A** — deadman/enable (release = instant stop)
- **Left stick Y** — forward/back
- **Left stick X** — turn
- **A + RB** — turbo

Verify your pad's axis/button numbers with:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

## Configuration

| File                        | What it configures |
|-----------------------------|--------------------|
| `config/sparks.yaml`        | SPARK MAX onboard velocity **PID gains** (`kp/ki/kd/kf`). Single source of truth for tuning. |
| `config/controllers.yaml`   | `diff_drive_controller` kinematics, velocity/accel limits, odometry, `cmd_vel_timeout`. |
| `config/joy_teleop.yaml`    | Joystick axes/buttons, speed scales, deadman/turbo. |
| `description/rover.urdf.xacro` | Robot model + `<ros2_control>` block (CAN IDs, inversion, `gear_ratio`, `burn_flash`, `require_all_sparks`, `spark_config` path). |

### PID gains (`config/sparks.yaml`)

The launch file resolves this file's absolute path and hands it to the plugin
via the URDF `spark_config` param; the plugin parses it in `on_init`.

- Precedence: `sparks.yaml` value → built-in default in
  `spark_can_hardware.hpp`. A missing file/key is non-fatal — the plugin warns
  and uses the default so the robot still comes up.
- `kF` (feedforward) does most of the work: `output ≈ kF × setpoint_rpm`,
  with `kF ≈ 1 / free_speed_RPM ≈ 1/5676` for NEO V1.1. Tune on the real robot.

### Persisting config to the SPARKs (`burn_flash`)

`<param name="burn_flash">` in the URDF defaults to `false`. The plugin
re-applies the full config (motor type, brake mode, PID, …) over CAN on every
launch, so flashing each boot only wears the limited flash. Set it to `true`
for a **single** launch to commit a persistent baseline to the devices, then
set it back to `false`.

### Pre-flight SPARK gate (`require_all_sparks`)

`<param name="require_all_sparks">` in the URDF defaults to `true`. During
`on_configure` the plugin waits for periodic status frames and checks bus
voltage for each of the 6 SPARKs. If any are missing and this is `true`, the
configure transition **fails** and the controller never activates — so the
rover won't drive with a dead wheel. Set it to `false` to allow coming up with
fewer than all six (limp mode).

## CAN bus health monitoring

`can_health_node` publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics`
so the CAN bus is visible while driving. It reads health **passively** — sysfs
counters + `ip -details link show` (netlink) — and **never opens a CAN socket**,
so it does not contend with the hardware plugin for the bus.

It reports three items: link up/down, controller state
(`ERROR-ACTIVE`/`WARNING`/`PASSIVE`/`BUS-OFF` — note `ERROR-ACTIVE` is the
*normal* state), and rising rx/tx error counts.

It launches by default with the teleop stack (`enable_can_health:=false` to
skip). Run/inspect standalone:

```bash
ros2 run storm_teleop can_health_node --ros-args -p interface:=can0
ros2 topic echo /diagnostics
ros2 run rqt_robot_monitor rqt_robot_monitor   # graphical view
```

This catches **bus-level** failure (bus-off, error storms), not a single silent
SPARK — see below.

## Known limitations

- **No mid-drive dropout detection.** sparkcan's getters return the last CAN
  frame's value with no staleness check and expose no connection status, so a
  SPARK falling off the bus mid-drive leaves its state frozen rather than
  flagged. Detecting this would require patching sparkcan to expose the
  per-frame timestamps it already stores.
