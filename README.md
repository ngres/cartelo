# Cartelo

Teleoperate ROS 2 robot arms using cartesian coordinate inputs in real-time. 

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
