# LLM Server

| info.           | Description |
| --------------- | ----------- |
| stock component | No         |
| contains speculation | Yes   |
| Location.       | External server or Jetson/Orin hardware |
| related artical | [PCPartPicker Example](https://au.pcpartpicker.com/list/Xywt4p) |

## Description

The LLM Server is used to host large language models for ARI, providing advanced conversational and reasoning capabilities. Hardware options vary based on required model size and performance.

## Details

- Xavier NX: ~GTX 1050 performance, 0.5-1B parameter models.
- Orin Nano: ~GTX 1650 performance.
- TX2: GT 1030/MX150, <500M parameter models.
- RTX 4070 SUPER: 7B-13B parameter models.
- AGX Orin: 1-2B parameter models.
- Jetson Nano: <100M parameter models.

!speculation: TX2 is the suggested upgrade for better performance.

## troubleshooting

### Model runs slowly
- Use more powerful hardware (see table above).
- Optimize model size and quantization.

### Server not detected
- Check network connectivity and server status.
- Verify correct model and dependencies are installed. 