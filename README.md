# ADAS — MPC Longitudinal Control

A from-scratch Advanced Driver Assistance System built to learn how a real
ADAS stack fits together: a physics-based driving simulator, a Model
Predictive Control (MPC) longitudinal controller, and a computer-vision
traffic light detector, all talking to each other over ROS 2 the same way
separate ECUs would over a vehicle network.

## What it does

- **[`PyDrivingSim/`](PyDrivingSim)** — a Python/pygame simulator with a
  double-track vehicle dynamics model (Pacejka tire model, throttle/brake
  pedal dynamics). It renders the car, road, traffic lights, cones, and
  speed-limit signs, and steps the vehicle physics forward each frame.
- **[`ros2_ws/`](ros2_ws)** — the ROS 2 packages that make the decisions:
  - `basic_agent` — a C++ MPC controller. Every cycle it takes the vehicle's
    current speed/position and a reference speed, solves a constrained
    quadratic program over a receding horizon (bounded acceleration and
    jerk), and returns the next acceleration command. It also reacts to
    the detected traffic light state to decide whether to cruise, follow,
    or stop.
  - `traffic_light_detector` — a C++/OpenCV node that receives a camera
    frame from the simulator (whichever traffic light image is "seen" at
    the vehicle's current position) and classifies it as red/yellow/green
    using HSV color thresholding.
  - `adas_msgs` — the shared message definitions (`ScenarioMsg`,
    `ManoeuvreMsg`, `TrafficLightStateMsg`) that let the simulator and the
    ROS 2 nodes talk to each other, modeled loosely on how a real ADASIS/ECU
    interface exchanges scenario and manoeuvre data.

**How a cycle works:** the simulator publishes a `ScenarioMsg` (speed,
acceleration, distance to the next traffic light, etc.) on `/scenario` and a
camera frame on `/traffic_light_image`. The detector classifies the frame and
publishes a `TrafficLightStateMsg` on `/traffic_light_state`. The MPC agent
combines both, solves for an acceleration command, and publishes a
`ManoeuvreMsg` on `/manoeuvre`. The simulator converts that acceleration into
a normalized pedal command (see [`pedal_acc_conversion.md`](pedal_acc_conversion.md))
and steps the vehicle. Design notes on the MPC formulation itself are in
[`mpc_notes.md`](mpc_notes.md); a broader concepts checklist for the whole
stack is in [`concepts.md`](concepts.md).

## Prerequisites

- Linux or WSL2
- Python 3.12 with `venv`
- ROS 2 (Humble or newer) with `rclpy`, `rclcpp`, `cv_bridge`, `sensor_msgs`,
  and OpenCV available
- An X server if running under WSL2 (pygame needs a display — see
  [`PyDrivingSim/README.md`](PyDrivingSim/README.md) for the WSL2 display
  setup)

## Setup

```bash
# 1. Python simulator environment (uses --system-site-packages so it can
#    see the ROS 2 Python packages, cv2, etc. installed system-wide)
cd PyDrivingSim
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
deactivate

# 2. Build the ROS 2 workspace
cd ../ros2_ws
colcon build
```

## Running it

Three terminals: the ROS 2 nodes, the simulator, and (optionally) the plots
afterwards.

```bash
# terminal 1 — MPC controller + traffic light detector nodes
cd ros2_ws
source install/setup.bash
ros2 launch basic_agent adas.launch.py

# terminal 2 — simulator
cd PyDrivingSim
source .venv/bin/activate
python simulator.py
```

Press `Ctrl+C` in terminal 2 to end the run — this flushes a per-cycle log
(`PyDrivingSim/agent_log.csv`) of tracking and actuation data.

```bash
# terminal 3 — plot the run
cd PyDrivingSim
source .venv/bin/activate
python plot_results.py
```

This produces `tracking.png` (actual vs. reference velocity) and
`actuation.png` (requested/actual acceleration and pedal command).

## Repo layout

```
ADAS/
├── PyDrivingSim/        Python + pygame simulator, vehicle dynamics model, plotting
├── ros2_ws/              ROS 2 workspace
│   └── src/
│       ├── adas_msgs/               shared message definitions
│       ├── basic_agent/             MPC longitudinal controller node
│       └── traffic_light_detector/  OpenCV traffic light classifier node
├── concepts.md            learning checklist for the underlying concepts
├── mpc_notes.md           MPC theory → application → code notes
└── pedal_acc_conversion.md  pedal/acceleration mapping derivation
```
