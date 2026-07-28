# Manual AirSim/SITL Simulation

This path runs beside the automated M1 acceptance path. It only renders an
AirSim `settings.json` and prints the matching ArduCopter command. It does not
start, monitor or stop UE, AirSim or SITL.

## Prerequisites

- The selected UE 4.27 project must already contain and enable the AirSim
  1.8.1 plugin.
- ArduPilot SITL must exist under `/home/waves/ardupilot`, unless a different
  absolute path is supplied with `--ardupilot-root`.
- Windows Firewall must allow the UE/AirSim process to exchange UDP traffic
  with WSL.

The configuration is not tied to Blocks. It can be used with any UE map in an
AirSim-enabled project. Terrain, collision meshes, lighting and spawn clearance
remain properties of that map and must be checked before arming.

## Generate settings.json

Run this command in WSL, replacing `<WindowsUser>` with the Windows account
name:

```bash
cd ~/aeromind_ws/apm_lite
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.manual_settings \
  --output /mnt/c/Users/<WindowsUser>/Documents/AirSim/settings.json
```

The generator discovers two addresses on every run:

- the current WSL IPv4 address becomes AirSim `UdpIp`;
- the Windows default gateway becomes ArduCopter `--sim-address`.

Both can be overridden when network discovery is not correct:

```bash
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.manual_settings \
  --output /mnt/c/Users/<WindowsUser>/Documents/AirSim/settings.json \
  --wsl-ip 172.24.80.10 \
  --windows-host-ip 172.24.80.1
```

The command prints JSON containing the resolved addresses and `sitl_command`.
It always reports `processes_started: false`.

## Start the manually owned processes

1. Generate `settings.json` after WSL starts. A WSL NAT address can change
   after a reboot.
2. Open the chosen AirSim-enabled UE scene and start Play mode yourself.
3. Create a dedicated SITL state directory in WSL, enter it, and execute the
   printed `sitl_command` in that terminal.
4. In a second WSL terminal, enter `~/aeromind_ws/apm_lite`, run
   `PYTHONPATH=src python3 -m aeromind_apm_lite.ground.browser.app --mode sitl
   --host 127.0.0.1 --port 8000`, and open `http://127.0.0.1:8000/`.
5. Stop the ground station, SITL and UE yourself when the run is complete.

The generated vehicle uses ArduCopter lock-step on sensor port `9003` and
control port `9002`. SITL sends MAVLink to `127.0.0.1:14550`, where the Lite
ground/onboard runtime can listen independently.

## Camera baseline

`front_center` is a forward monocular Scene RGB camera (`ImageType: 0`) with a
default 1280 x 720 image, 90 degree horizontal field of view and a configurable
pose. It intentionally declares no D435, depth image or lidar because the
reviewed aircraft inventory does not establish that hardware.

The baseline can be adjusted without editing JSON:

```bash
PYTHONPATH=src python3 -m \
  aeromind_apm_lite.ground.simulation.manual_settings \
  --output /mnt/c/Users/<WindowsUser>/Documents/AirSim/settings.json \
  --camera-width 640 --camera-height 480 --camera-fov 78 \
  --camera-x 0.30 --camera-z -0.08 --camera-pitch -5
```

Match these values to the measured real camera only after its model, resolution,
field of view and mounting pose are known.

The current ground station reports this camera profile and its actual offline
state. It does not yet pull pixels from AirSim, and the VLM panel remains
unavailable rather than synthesizing a camera or semantic result.
