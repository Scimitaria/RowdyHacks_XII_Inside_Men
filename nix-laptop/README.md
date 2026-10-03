# Laptop ROS 2 environment (NixOS)

A ROS 2 Jazzy shell that talks to the robot's Pi over Tailscale using Cyclone DDS
with unicast discovery (Tailscale carries no multicast).

## Setup

1. NixOS config (once): `services.tailscale.enable = true;` and
   `networking.firewall.trustedInterfaces = [ "tailscale0" ];`
2. On the laptop, find its Tailscale address: `tailscale ip -4`
3. On the Pi, add it to the `<Peers>` list in `~/cyclonedds.xml`:
   `<Peer address="THAT_ADDRESS"/>`
4. On the laptop, from this directory: `nix develop`

## Check it

With `oak_camera` running on the Pi:

```bash
ros2 topic list                  # camera topics from the Pi should appear
```

Don't view the raw image topics over Tailscale (~110 Mbit/s for RGB); use the
compressed topics instead.

```bash
ros2 run rqt_image_view rqt_image_view
```

`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` are set by the shell
and must match the Pi's (see the end of its `~/.bashrc`). If `cyclonedds.xml`
isn't found, you ran `nix develop` from the wrong directory.
