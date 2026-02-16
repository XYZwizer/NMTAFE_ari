# Localizer

| info.           | Description |
| --------------- | ----------- |
| stock component | Yes         |
| contains speculation | Yes   |
| Location.       | /opt/pal/gallium/share/ari_2dnav (localizer ROS package) |
| related artical | [PAL Application Management](https://docs.pal-robotics.com/sdk/23.12/management/application-management.html#application-management), [ARI Navigation Config](https://github.com/pal-robotics/ari_navigation/tree/melodic-devel/ari_laser_sensors/config) |

## Description

The localizer is responsible for ARI's self-localization using sensor data (e.g., cameras, LIDAR) and ROS packages. It is started by the pal_startup_manager and is critical for navigation and mapping.

## Details

- Uses /opt/pal/gallium/share/ari_2dnav as the main ROS package.
- Startup managed by pal_startup_manager and pal_startup_node.
- Relies on calibration data from /etc/calibration_data/ and ROS parameters.
- May require specific calibration files (e.g., device_info.yaml).
- Integrates with camera calibration and extrinsics (URDF/Xacro files).

!speculation: Some calibration files may be missing or incomplete on VMs; check real robot for full data.

## troubleshooting

### Localization not starting
- Check for missing calibration files in /etc/calibration_data/.
- Verify pal_startup_manager and related ROS nodes are running.
- Review logs for missing environment variables or parameters.

### Poor localization accuracy
- Ensure all sensors are calibrated.
- Check for correct extrinsics and URDF files.

</rewritten_file> 