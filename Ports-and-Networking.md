# Ports and Networking

## Overview

This document summarizes the network ports used by the ARI robot and provides troubleshooting tips for common networking issues, including those encountered in virtual machine (VM) or simulator setups.

## Port List

| Port | Protocol | Usage/Service                | Notes                       |
|------|----------|------------------------------|-----------------------------|
| 22   | TCP      | SSH                          | Remote shell access         |
| 53   | TCP      | DNS                          | Domain name resolution      |
| 80   | TCP      | HTTP                         | Web UI & REST interface     |
| 443  | TCP      | HTTPS                        | Apache server               |
| 3000 | TCP      | HTTP                         | Grafana monitoring          |
| 8080 | TCP      | HTTP                         | Webcommander               |
| 9090 | TCP      | WebSocket                    | ROS communication bridge    |
| 11011| TCP      | HTTP                         | Presentation screen         |
| 8000 | TCP      | HTTP-ALT                     | Alternate HTTP              |
| 8001 | TCP      | VCOM-Tunnel                  | Knowledge base              |
| 8888 | TCP      | Sun Answerbook               |                             |
| 2003 | TCP      | Finger                       |                             |
| 2004 | TCP      | Mailbox                      |                             |
| 2809 | TCP      | CORBA                        |                             |
| 6969 | TCP      | ACMSoda                      |                             |

## Example Network Traffic

- Port 80: Used for arms control
- Port 9090: Used for move base

## Troubleshooting Networking

### Duplicate Routes in VM/Simulator

If you experience asymmetric connectivity (e.g., robot can ping laptop but not vice versa), check for duplicate routes:

- Remove conflicting route on eth1:
  ```bash
  sudo ip route del 192.168.0.0/24 dev eth1
  ```
- Ensure only one active route for the 192.168.0.0/24 network.

### General Tips
- Check physical connections and network cables.
- Verify correct IP addresses and subnet masks.
- Use `ifconfig` or `ip addr` to inspect network interfaces.
- Use `ping` and `traceroute` for connectivity testing. 