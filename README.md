# hedge_heading_to_master

ROS package that gives **each drone its heading (yaw) and distance to every other drone**, using Marvelmind hedgehog data from `/hedge_pos_ang`.

## Input

- **Topic:** `/hedge_pos_ang` (`marvelmind_nav/hedge_pos_ang`)
- **Source:** `rosrun marvelmind_nav hedge_rcv_bin /dev/ttyACM1` (or your serial port)

Each message carries one hedgehog update: `address`, `x_m`, `y_m`, `z_m`, `angle` (heading in degrees).

## Output

- **Topic:** `fleet_heading_distances` (`hedge_heading_to_master/FleetHeadingDistances`)
- **Debug Topic:** `fleet_heading_distances_debug` (`hedge_heading_to_master/FleetHeadingDistancesDebug`) with both filtered and raw arrays for comparison

For each drone (by `address`), the message contains:

- **heading_deg** — yaw angle in degrees (from `hedge_pos_ang.angle`)
- **other_addresses** — list of other drone addresses
- **distances_to_others** — distance in meters to each of those drones (same order as `other_addresses`)

So each drone can look up its own entry by `address` and read its heading and distances to all others.

## Messages

| Message | Description |
|--------|-------------|
| `DroneHeadingDistances` | One drone: `address`, `heading_deg`, `other_addresses[]`, `distances_to_others[]` |
| `FleetHeadingDistances` | `header` + `drones[]` (one `DroneHeadingDistances` per drone) |

## Usage

**Recommended: use the launch file** (starts the Marvelmind receiver and the heading/distances node):

```bash
roslaunch hedge_heading_to_master hedge_heading_to_master.launch
```

Override the serial port or baud if needed:

```bash
roslaunch hedge_heading_to_master hedge_heading_to_master.launch hedge_port:=/dev/ttyACM1 hedge_baud:=9600
```

### Test with one real drone (dummy second drone)

If you only have one drone, use the test launch to add a **dummy drone** (configurable position and angle) so the fleet node sees two drones and publishes heading + distance:

```bash
roslaunch hedge_heading_to_master test_with_dummy.launch
```

Override the dummy’s pose in the launch file or at runtime:

```bash
# In launch: pass args, e.g. address:=99 x_m:=1.5 y_m:=0.5 angle_deg:=90
roslaunch hedge_heading_to_master test_with_dummy.launch x_m:=1.5 angle_deg:=90

# Or change while running (node re-reads params each cycle):
rosparam set /dummy_hedge_publisher/x_m 3.0
rosparam set /dummy_hedge_publisher/angle_deg 180
```

Check output: `rostopic echo /fleet_heading_distances`
Compare raw vs filtered: `rostopic echo /fleet_heading_distances_debug`

### Test with two real drones (Master 21–22, Slave 1: 11–12)

If you have two paired mobile beacons (Master 21–22, Slave 1: 11–12), use:

```bash
roslaunch hedge_heading_to_master test_two_drones.launch
```

Position for each drone is the **midpoint** of its two beacons; heading is the joint heading from the primary beacon (22 and 12). The launch sets `hedge_pairs="22:21,22 12:11,12"`. Check: `rostopic echo /fleet_heading_distances` — you should see two drones (addresses 22 and 12) with midpoint positions.

### Alternative: run nodes manually

If you prefer to run the hedge receiver yourself (e.g. in a separate terminal):

```bash
rosrun marvelmind_nav hedge_rcv_bin /dev/ttyACM1
rosrun hedge_heading_to_master hedge_heading_distances_node.py
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~hedge_pos_ang_topic` | `/hedge_pos_ang` | Input topic from Marvelmind |
| `~fleet_heading_distances_topic` | `fleet_heading_distances` | Output topic |
| `~fleet_heading_distances_debug_topic` | `fleet_heading_distances_debug` | Debug output topic (raw + filtered values). |
| `~publish_rate` | 10 | Publish rate (Hz) |
| `~frame_id` | `marvelmind` | Header frame_id for output |
| `~hedge_pairs` | (empty) | Paired beacons: position = midpoint, heading from primary. e.g. `"22:21,22 12:11,12"` for Master (21–22) and Slave 1 (11–12). |
| `~hedge_addresses` | (empty) | Comma-separated addresses when not using `~hedge_pairs`. |
| `~primary_hedge_address` | -1 | Single joint address; used only if neither `~hedge_pairs` nor `~hedge_addresses` is set. |
| `~dummy_address` | 99 | Always accept this address (dummy); use with test_with_dummy.launch. |
| `~enable_kalman` | `true` | Enable/disable Kalman filtering before publish. |
| `~kf_q_distance` | `0.02` | Process noise for distance filter (higher = more responsive, less smooth). |
| `~kf_r_distance` | `0.2` | Measurement noise for distance filter (higher = smoother, more lag). |
| `~kf_q_angle` | `0.5` | Process noise for angle filters (`heading_error`, `other_heading_with_respect_to_drone`). |
| `~kf_r_angle` | `4.0` | Measurement noise for angle filters (higher = smoother, more lag). |

## Troubleshooting

**hedge_rcv_bin dies (exit code -1)**  
The Marvelmind receiver can exit after a while (e.g. serial disconnect, USB drop, or internal error). The driver now exits cleanly on serial errors (poll/read failure, POLLHUP/POLLERR). To see why it died, check the log:

```bash
cat ~/.ros/log/<run-id>/hedge_rcv_bin-2.log
```

Common causes: unplugged modem, wrong port (`ls /dev/ttyACM*`), permission (`sudo usermod -aG dialout $USER` then re-login), or baud mismatch. If it still crashes with -1 after ~1 min, run under GDB to get a backtrace: `gdb -ex run -ex bt -batch --args /path/to/hedge_rcv_bin /dev/ttyACM1 9600`.

## Scripted environment setup (Marvelmind API)

You can set stationary and hedgehog beacon positions and freeze the map **without the Marvelmind Dashboard**, using the Marvelmind Dashboard API and a script.

**Requirements**

- **libdashapi.so** (Linux) from Marvelmind — the Dashboard API library. Obtain it from [Marvelmind download](https://marvelmind.com/download/) or support. Put it where the node can load it (e.g. `export LD_LIBRARY_PATH=/path/to/dir:$LD_LIBRARY_PATH` or place in a directory on the default library path).
- **PyYAML**: `sudo apt install python3-yaml` (for the setup script).

**Workflow**

1. **Do not** run `hedge_rcv_bin` — the API node needs exclusive access to the modem port.
2. Start the API node (it will try to open the port):
   ```bash
   roslaunch hedge_heading_to_master setup_marvelmind_api.launch
   # Or: rosrun marvelmind_nav marvelmind_api_ros /dev/serial/by-id/usb-Marvelmind_...
   ```
3. Edit `config/beacon_positions_example.yaml` with your beacon addresses and positions in **millimeters** (`x_mm`, `y_mm`, `z_mm`).
4. Run the setup script:
   ```bash
   rosrun hedge_heading_to_master setup_marvelmind_env.py _beacon_config:=$(rospack find hedge_heading_to_master)/config/beacon_positions_example.yaml
   ```
   The script will: erase map, set each beacon location from the YAML, then freeze the map.
5. Stop the API node (Ctrl+C). For normal operation, use `hedge_rcv_bin` and your usual launch (e.g. `test_two_drones.launch`).

**YAML format**

- `stationary_beacons`: list of `{address, x_mm, y_mm, z_mm}`.
- `hedgehog_beacons`: optional list of `{address, x_mm, y_mm, z_mm}` for mobile beacons.

Saving the map to flash (persist across power cycles) may require additional API calls (e.g. `WRITE_FLASH_DUMP`); see Marvelmind documentation for your modem.

## Build

```bash
cd ~/catkin_ws
catkin_make
# or
catkin build
```

## Dependencies

- `marvelmind_nav` (for `hedge_pos_ang` message and `hedge_rcv_bin`)
- `rospy`, `std_msgs`
