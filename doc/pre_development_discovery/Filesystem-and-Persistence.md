# Filesystem and Persistence

## Overview

This document explains which directories and files on the ARI robot are persistent across reboots, which should be backed up, and how to change non-persistent configurations.

## Persistent Directories

| Dir location        | Description                                      | Make copy | Justification                                      |
|---------------------|--------------------------------------------------|-----------|----------------------------------------------------|
| /media/root-ro      | Main filesystem (read-only)                      | N         | Unique data removed on reinstall                   |
| /boot/efi           | Bootloader filesystem                            | N         | Same for each install                              |
| /etc/calibration    | PAL-specific calibrations and user settings      | Y         | Needed for robot operation, unique since delivery   |
| /home               | User home dir, includes .ros and calibration     | Y         | Apps use calibration data from here                |
| /var/log            | Log files                                        | N~        | Usually not unique, too large to copy              |

**Also back up `/etc/fstab`** (contains boot mounting and settings).

## Checking Mounts and Partitions

Use the following commands to inspect disk partitions and mounts:

```bash
lsblk
mount
```

## Changing Non-Persistent Configurations

To edit configuration files (e.g., PID settings):

1. Remount filesystem as read-write:
   ```bash
   sudo rw
   ```
2. Edit the file (using `vi` if `nano` is not available):
   ```bash
   sudo vi /opt/pal/gallium/share/ari_controller_configuration/config/pids_v2.yaml
   ```
3. Remount as read-only:
   ```bash
   sudo ro
   ```

## Troubleshooting
- If unable to remount `/ro` as read-only, ensure no processes are using it.
- Always back up important configuration files before making changes. 