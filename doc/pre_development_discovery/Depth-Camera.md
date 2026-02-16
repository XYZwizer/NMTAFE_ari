# Depth Camera

![Depth Camera - Computerphile](../media/ARI-General-Depth-cameras-image1.png)

| info.           | Description |
| --------------- | ----------- |
| stock component | Yes         |
| contains speculation | No   |
| Location.       | Torso Front (RGB-D RealSense D435i), Torso Back (Intel Realsense T265, stereo-fisheye) |
| related artical | [Depth Camera - Computerphile](https://www.youtube.com/watch?v=bRkUGqsz6SI) |

## Description

The ARI robot uses two depth cameras: a RealSense D435i on the torso front and an Intel Realsense T265 (stereo-fisheye) on the torso back. These provide RGB-D and stereo depth data for navigation and perception.

## Details

- The D435i provides RGB-D data for front-facing perception.
- The T265 provides stereo-fisheye data for rear perception and localization.
- Both are standard components in the ARI platform.

## troubleshooting

### Camera not detected
- Check physical connections and USB ports.
- Verify drivers are installed and ROS nodes are running.

### Poor depth quality
- Clean camera lenses.
- Ensure adequate lighting and no obstructions. 