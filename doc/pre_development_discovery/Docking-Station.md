# Docking Station

![Docking Station 1](../media/ARI-General-Docking-station-image1.png)

![Docking Station 2](../media/ARI-General-Docking-station-image2.png)

| info.           | Description |
| --------------- | ----------- |
| stock component | Yes         |
| contains speculation | No   |
| Location.       | See ARI documentation for physical location |
| related artical | [aruco_ros GitHub](https://github.com/pal-robotics/aruco_ros) |

## Description

The docking station is used for automatic charging and parking of the ARI robot. It uses visual markers (e.g., ArUco) for alignment and docking.

## Details

- Provides a safe and reliable way for ARI to recharge autonomously.
- Uses visual fiducials for precise alignment.
- Integrated with ARI's navigation and power management systems.

## troubleshooting

### Robot fails to dock
- Check visual markers are clean and visible.
- Ensure docking station is powered and unobstructed.
- Verify ARI's navigation and docking nodes are running. 