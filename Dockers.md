# Dockers

## Overview

This document explains how to use Docker containers with the ARI robot, including setup steps and troubleshooting common ROS networking issues.

## Running Docker Containers with ROS

### Common Issue: Container Can't Resolve Hostname

This is a common problem with ROS networking in Docker. To fix:

1. Check if the container can ping itself by hostname:
   ```bash
   ping $(hostname)
   ```
   If this fails, add the hostname to `/etc/hosts`:
   ```bash
   echo "127.0.0.1 $(hostname)" | sudo tee -a /etc/hosts
   ```
2. Set the ROS_HOSTNAME environment variable:
   ```bash
   export ROS_HOSTNAME=localhost
   ```
3. Try running `roscore` again:
   ```bash
   roscore &
   ```
4. Verify ROS is working:
   ```bash
   rostopic list
   ```
   You should see at least the `/rosout` topic.

### Using Explicit IP Addresses

If issues persist, configure ROS with explicit IPs:

```bash
unset ROS_HOSTNAME
unset ROS_MASTER_URI
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
roscore &
```

## Troubleshooting
- Ensure Docker containers have network access.
- Check environment variables for correct ROS networking setup. 