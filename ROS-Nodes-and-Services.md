# ROS Nodes and Services

## Overview

This document summarizes the key ROS nodes, topics, and services used by the ARI robot, with examples and troubleshooting tips.

## Example: Publishing to a Topic

To publish a message to a topic (e.g., /chatter):

```json
{
  "op": "publish",
  "topic": "/chatter",
  "msg": {
    "data": "Hello via HTTP!"
  }
}
```

## Useful Links
- [PAL Robotics ARI SDK Topics](https://docs.pal-robotics.com/ari/sdk/23.1.12/topics.html)

## WebSocket Example

Connect to the robot's ROS bridge:

```bash
wscat -c ws://192.168.0.100:9090
```

Example service calls:
- List topics: `{ "op": "call_service", "service": "/rosapi/topics", "type": "rosapi/Topics" }`
- List services: `{ "op": "call_service", "service": "/rosapi/services", "type": "rosapi/Services" }`

## Troubleshooting
- Ensure all required ROS nodes are running.
- Use `rostopic list` and `rosservice list` to inspect available topics and services.
- Check logs for errors if communication fails. 