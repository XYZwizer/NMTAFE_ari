# Lidar

| info.           | Description |
| --------------- | ----------- |
| stock component | Yes         |
| contains speculation | Yes   |
| Location.       | Base of robot (see ARI documentation) |
| related artical | [DFRobot Lidar](https://www.dfrobot.com/product-2740.html), [RGB-D 3D ToF Sensor](https://core-electronics.com.au/rgb-d-3d-tof-sensor-camera-supports-ros1-and-ros2.html), [YDLIDAR TG15](https://github.com/pal-robotics/ari_navigation/tree/melodic-devel/ari_laser_sensors/config) |

## Description

The ARI robot uses a LIDAR sensor for navigation and mapping. The most common model is the YDLIDAR TG15, as referenced in the Autonomous Navigation Pack and configuration files.

## Details

- LIDAR is used for SLAM, obstacle avoidance, and navigation.
- The YDLIDAR TG15 is the default, but other models may be supported.
- Vision and laser-based self-localization and mapping (SLAM).
- Available RViZ and web plugins for navigation.

!speculation: There are mentions of RGBD sensors and other LIDAR models, but YDLIDAR TG15 is most likely.

## troubleshooting

### LIDAR not detected
- Check power and data connections.
- Verify correct model is set in ROS launch files.

### Poor mapping or navigation
- Ensure LIDAR is unobstructed and clean.
- Check for correct calibration and configuration in ROS. 