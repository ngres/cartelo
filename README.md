# Cartelo

Teleoperate ROS 2 robot arms using cartesian coordinate inputs from 3D trackers (VR controllers, smartphones, etc.).

> [!WARNING]
> This project is still under construction 🚧
> My goal is to create a LeRobot fine tuning dataset using an UR12e arm and a HTC Vive controller. If you are interested in combining LeRobot with ROS 2 check out [LeROS2](https://github.com/Nico0302/leros2).

## HTC VIVE <> SO-101 Setup

```bash
sudo curl -fsSL https://raw.githubusercontent.com/cntools/libsurvive/master/useful_files/81-vive.rules \
    -o /etc/udev/rules.d/81-vive.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

```bash
lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=so101_follower
```

```bash
ros2 launch so101_description bringup.launch.py usb_port:=/dev/ttyACM1
```

```bash
ros2 launch cartesian_teleoperation vive.launch.py
```

```bash
ros2 launch cartesian_teleoperation teleop.launch.py
```

```bash
ros2 launch so101_moveit_config moveit.launch.py use_fake_hardware:=true
```

## X11Forwarding

```bash
xhost +localhost
```
