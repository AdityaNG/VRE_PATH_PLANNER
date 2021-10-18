# Getting Started

## Installing FSDS

Clone the FSDS repo to your home :

```bash
cd ~/
git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator
```

Download the Formula Student Driverless Simulator from https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/tag/v2.0.0

Save it to `~/Apps/fsds-v2.0.0-linux/`

## Setting up Python env

Use the `./setup_env.sh` script. The source
```bash
source ~/fsds/bin/activate
```

## Running the demo

```bash
python main.py
```

# Tasks

## Task 1

You are given a set of GPS waypoints(lat, lon), a set of obstacles(lat, lon, radius) and current GPS position(lat, lon). Produce a spline going through the waypoints, avoiding obstacles with a buffer of x meters.

## Task 2

Approximate vehicle GPS position by integrating accelerometer data twice. This is done because GPS will operate at <10Hz and we'll need position feedback closer to >30Hz.

Note that GPS data will give position and speed at 10Hz. Accelerometer will give this at say 30Hz. You can use the GPS speed, velocity to validate the accelerometers predicted speed and velocity.