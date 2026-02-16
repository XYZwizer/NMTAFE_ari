# Respeaker Microphone Array

![Respeaker](../media/ARI-General-respeaker-image1.png)

| info.           | Description |
| --------------- | ----------- |
| stock component | Yes         |
| contains speculation | Yes   |
| Location.       | Head of robot (see ARI documentation) |
| related artical | [Seeed Respeaker](https://core-electronics.com.au/respeaker-usb-mic-array.html), [Seeed Wiki](https://wiki.seeedstudio.com/ReSpeaker-USB-Mic-Array/), [ODAS](https://github.com/introlab/odas), [VOSK](https://github.com/alphacep/vosk-api?tab=readme-ov-file) |

## Description

The Respeaker USB Microphone Array provides far-field voice pickup and is used for speech recognition and sound source localization on the ARI robot.

## Details

- Plug & Play, compatible with Linux/ROS.
- Far-field voice pickup up to 5m, 360° pattern.
- Acoustic algorithms: DOA, AEC, AGC, NS.
- Publishes filtered audio to /audio/channel0 in ROS.
- Supports VOSK and Google Cloud backends (VOSK preferred).
- Can be tuned via ROS for noise suppression.

!speculation: Noise suppression can be adjusted via ROS; may improve performance in crowds.

## troubleshooting

### Not detected in ROS
- Check USB connection and device recognition.
- Verify ROS node is running and publishing topics.

### Poor audio quality
- Adjust noise suppression settings.
- Test device directly via USB for diagnostics. 