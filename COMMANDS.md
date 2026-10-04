# Commands

## One-time setup

Let non-root users talk to the OAK-D, then replug the camera:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Create the Python venv (system Python 3.12 so ROS's `rclpy` works):

```bash
uv venv --system-site-packages --python /usr/bin/python3.12
uv sync
```

## Build and run a node

```bash
./src/robot_package/rebuild <node>   # e.g. oak_camera, imu_tracker, motor
```

Sources ROS, syncs the venv, builds `robot_package`, then runs the node.

## Dependencies

```bash
uv add <package>   # add a Python dependency (updates pyproject.toml + uv.lock)
uv sync            # install what pyproject.toml / uv.lock say
```

## Check the OAK-D camera node

Run these in a second terminal after `source /opt/ros/jazzy/setup.bash && source install/setup.bash`:

```bash
ros2 topic list                                                   # both /camera/... topics should appear
ros2 topic hz /camera/rgb/image_raw                               # frame rate (~15 Hz)
ros2 topic echo --once /camera/depth/image_raw --field encoding   # expect 16UC1
ros2 run rqt_image_view rqt_image_view                            # view the images (needs a display)
```

View the streams in OpenCV windows (needs a display; press `q` to quit). With `oak_camera` running in another terminal:

```bash
ros2 run robot_package camera_viewer
ros2 run robot_package camera_viewer --ros-args -p show_depth:=false -p max_depth_mm:=3000
```

Over SSH there is no display, so Qt fails with `could not connect to display`. To show the windows on the Pi's own monitor, point the viewer at its Wayland session:

```bash
XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 QT_QPA_PLATFORM=wayland ros2 run robot_package camera_viewer
```

Node parameters:

```bash
ros2 run robot_package oak_camera --ros-args -p fps:=30 -p publish_depth:=false
```

Only one process can use the camera at a time. `oak_camera` also publishes the IMU heading on `/imu/data` (`sensor_msgs/Imu`, for the EKF), the same topic as `imu_tracker`, so run `oak_camera` on its own and don't start `imu_tracker` alongside it. Use `-p publish_imu:=false` to turn the IMU off, or run `imu_tracker` by itself when you don't need images.

## Read the wheel encoders (Pico)

The Pico counts every edge of both encoder channels (GPIO 10/11 = motor A, 12/13 = motor B). Send `6` over `/dev/ttyACM0` and it replies:

```
ENC <count_a> <count_b> <time_us>
OK 6
```

Counts are cumulative since boot and go up when the robot drives forward; `time_us` is the Pico's clock, for computing wheel speed between two reads. If a wheel counts down while driving forward, flip `ENC_A_REVERSED` / `ENC_B_REVERSED` in `src/pi_pico_code/main.c`.

The `odom` node polls this and publishes `/wheel/odom` (`nav_msgs/Odometry`, `odom -> base_link`). It can run alongside `motor`; both share `/dev/ttyACM0`. Set the wheel geometry for your robot:

```bash
./src/robot_package/rebuild odom   # defaults: ticks_per_rev 1960 (490 pulses x 4), wheel_radius 0.0335, wheel_base 0.21
./src/robot_package/rebuild odom --ros-args -p wheel_base:=0.22   # override any of them
ros2 topic echo /wheel/odom --field twist.twist   # vx / yaw rate while driving
```

`ticks_per_rev` is 4 x (encoder pulses per motor turn) x (gear ratio). If turning in place reports too much or too little rotation, adjust `wheel_base`. Add `-p publish_tf:=true` only when running without the EKF.

## View the camera from another computer

On the Pi, run this next to whatever starts `oak_camera` (e.g. `robot.launch.py`). It publishes JPEG and PNG versions of the images, about 34 Mbit/s in total instead of about 170 Mbit/s raw:

```bash
ros2 launch robot_package compress.launch.py                # RGB + depth
ros2 launch robot_package compress.launch.py depth:=false   # RGB only, less bandwidth
```

On the laptop (see `nix-laptop/`), inside `nix develop`:

```bash
ros2 topic list                                  # camera/... topics from the Pi
ros2 run rqt_image_view rqt_image_view           # pick /camera/rgb/image_raw, set transport to "compressed"
```

Never view the raw `image_raw` topics from the laptop; use the compressed ones.

## Slot counter Pico (optional)

`src/pi_pico_slot_counter` is firmware for a Pico that only counts slotted-disc wheel sensors (GPIO 10 = motor A, GPIO 12 = motor B, one count per slot). Build it and copy the `.uf2` onto the Pico while holding BOOTSEL:

```bash
cd src/pi_pico_slot_counter && mkdir -p build && cd build && cmake .. && make -j4   # -> pi_pico_slot_counter.uf2
```

Send `6` and it replies `ENC <count_a> <count_b> <time_us>`, where the counts are the slots seen since the previous `6` (they reset on every read, unlike the motor Pico's running totals, so `odom` can't read this Pico as-is). Test it with `.venv/bin/python src/robot_package/robot_package/get_slot_node.py [port]`. Slot sensors can't sense direction, so counts are never negative, even when reversing or turning in place.

## Fuse wheel odometry + IMU (EKF)

One-time install of the EKF package:

```bash
sudo apt install ros-jazzy-robot-localization
```

With `imu_tracker` (or `oak_camera`) and `odom` (`/wheel/odom`) running, start the EKF:

```bash
ros2 launch robot_package localization.launch.py
```

The EKF (`config/ekf.yaml`) fuses the heading on `/imu/data` with `/wheel/odom` into `/odometry/filtered` and the `odom -> base_link` TF.

Check it:

```bash
ros2 topic hz /odometry/filtered                 # ~30 Hz
ros2 run tf2_ros tf2_echo odom base_link         # fused transform
```

The wheel-odometry node must use `frame_id: odom` / `child_frame_id: base_link`, fill non-zero covariances, and not publish its own `odom -> base_link` TF.

## Build a map (RTAB-Map)

One-time install of RTAB-Map:

```bash
sudo apt install ros-jazzy-rtabmap-slam
```

Needs three things running: `oak_camera`, the localization launch above (`/odometry/filtered` + `odom -> base_link` TF), and a static transform from `base_link` to the camera. RTAB-Map expects the camera frame in the optical convention (z forward, x right, y down), so the transform must include that rotation. Example for a camera mounted 10 cm forward and 20 cm up, facing forward:

```bash
ros2 run tf2_ros static_transform_publisher --x 0.1 --z 0.2 --roll -1.5708 --yaw -1.5708 --frame-id base_link --child-frame-id oak_camera
```

Then start the feeder and RTAB-Map together:

```bash
ros2 launch robot_package rtabmap.launch.py
ros2 launch robot_package rtabmap.launch.py new_map:=false database_path:=/home/levi/maps/lab.db   # keep adding to a saved map
```

`rtabmap_feeder` pairs the RGB and depth frames, publishes a `CameraInfo`, and holds frames back until odometry and TF are available. It logs how many frames it sent each 2 s and why it is holding back. Set the camera intrinsics with `-p fx:=... -p fy:=... -p cx:=... -p cy:=...` (the defaults are only a rough guess).

Check it and view the map:

```bash
ros2 topic hz /rtabmap_input/rgb/image   # ~15 Hz once odometry and TF are up
ros2 topic echo --once /map --field info # occupancy grid built by RTAB-Map
rtabmap-databaseViewer ~/.ros/rtabmap.db # browse the saved map (from ros-jazzy-rtabmap)
```

## Run everything

```bash
ros2 launch robot_package robot.launch.py
```

Starts `oak_camera` (images + IMU heading), `odom` (wheel odometry), the camera TF, the EKF and RTAB-Map. Nav2 and exploring are a separate launch (see below). Don't run any of those separately, and stop any other `oak_camera` first, because only one process can open the camera. Keep the robot still for the first second while the gyro calibrates.

Arguments (add as `name:=value`):

```bash
ros2 launch robot_package robot.launch.py cam_x:=0.12 cam_y:=0.0 cam_z:=0.2   # camera position on the robot, in meters
ros2 launch robot_package robot.launch.py mapping:=false                      # skip RTAB-Map, only odometry + camera
ros2 launch robot_package robot.launch.py new_map:=false                      # keep and extend the saved map
```

## Explore with Nav2

One-time install of Nav2:

```bash
sudo apt install ros-jazzy-navigation2
```

With `robot.launch.py` running, start Nav2 and the explorer in another terminal:

```bash
ros2 launch robot_package navigation.launch.py
```

It starts:
- `point_cloud_xyz`, which turns the depth image into `/camera/depth/points` for Nav2's obstacle layers
- the Nav2 servers (params in `config/nav2_params.yaml`), planning on RTAB-Map's `/map`
- `cmd_vel`, which turns Nav2's `/cmd_vel` into per-wheel duty on `/motor_cmd` and goes quiet when Nav2 stops, so `keyboard` still works in between
- `explore`

`explore` spins once to look around, then picks the nearest reachable edge of the map and sends it to Nav2 as a goal. If the camera sees past that edge before the robot gets there, it switches to the next one. Frontiers Nav2 can't reach, or that stay unmapped after the robot gets there (glass, black surfaces), are skipped. It logs "exploration done" when nothing is left. Ctrl+C stops everything. You can also send goals by hand from RViz's "2D Goal Pose".

Calibrate `cmd_vel` once, robot on the floor with `robot.launch.py` running: drive at full duty for a moment and read the speed:

```bash
ros2 topic pub -r 10 -t 20 /motor_cmd std_msgs/msg/String "data: '1 100'"
ros2 topic echo /wheel/odom --field twist.twist.linear.x   # in another terminal
```

Put that speed in the `cmd_vel` node's `max_wheel_speed` parameter (in `navigation.launch.py`), and raise `min_duty` if the wheels don't turn at small speeds. Check it with only the `cmd_vel` node running (not `navigation.launch.py`, whose explorer would drive too):

```bash
ros2 run robot_package cmd_vel --ros-args -p max_wheel_speed:=0.5
ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"   # /wheel/odom should read ~0.15
ros2 lifecycle get /bt_navigator                                                  # active once Nav2 is up
```

