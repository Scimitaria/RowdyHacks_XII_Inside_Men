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

Only one process can use the camera at a time, so don't run `oak_camera` and `imu_tracker` together.
