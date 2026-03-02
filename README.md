# 🦾 Unitree D1 Arm — IK Pick & Place

A C++ pick-and-place controller for the **Unitree D1 robotic arm** mounted on the Go2 quadruped. Uses MuJoCo-based inverse kinematics and Unitree's DDS middleware to command the arm and read real-time joint feedback — no blind delays, gripper closes only when the arm has actually reached its target.

```
┌─────────────────────────────────────────────────────┐
│  Your PC  ──DDS──►  rt/arm_Command  ──►  D1 Arm     │
│           ◄──DDS──  rt/arm Feedback ◄──  D1 Arm     │
└─────────────────────────────────────────────────────┘
```

---

## Features

- **Feedback-based motion**: polls `rt/arm Feedback` at 20 Hz — gripper triggers only when joints are within 3° of target
- **MuJoCo IK engine**: solves 6-DOF inverse kinematics from a Cartesian XYZ target
- **Drag teaching support**: free/lock all joints for hand-guided pose recording
- **Stall-based grip detection**: detects object contact from gripper position divergence (no force sensor required)
- **Interactive CLI**: live `pick x y z` / `put x y z` / `exit` commands

---

## Repository Structure

```
d1_pick_place/
├── CMakeLists.txt
├── README.md
├── include/
│   └── kinematics.hpp          # MuJoCo IK engine header
├── src/
│   ├── deploy.cpp              # Main pick & place controller  ← you are here
│   ├── kinematics.cpp          # MuJoCo IK implementation
│   └── drag_teach.cpp          # Drag teaching / pose recording tool
├── config/
│   └── d1_kinematics.xml       # MuJoCo model of the D1 arm
└── build/                      # CMake build output (git-ignored)
```

---

## Prerequisites

### Hardware
- Unitree D1 arm connected to your PC via **Ethernet**
- PC running **Ubuntu 20.04 or 22.04** (tested on 22.04)
- D1 arm powered on and network interface configured

### Software Dependencies

| Dependency | Version | Purpose |
|---|---|---|
| CMake | ≥ 3.16 | Build system |
| GCC / G++ | ≥ 9 | C++17 compiler |
| Unitree D1 SDK (`unitree_sdk2`) | latest | DDS middleware & arm message types |
| MuJoCo | ≥ 2.3 | Physics engine for IK |
| Eigen3 | ≥ 3.3 | Linear algebra (used by IK) |

---

## Environment Setup

### Step 1 — Install system dependencies

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libboost-all-dev
```

### Step 2 — Install the Unitree D1 SDK

Clone and install `unitree_sdk2` (follow Unitree's official instructions):

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

> **Note:** The SDK installs headers to `/usr/local/include/unitree` and libraries to `/usr/local/lib`. If you install to a custom prefix, set `CMAKE_PREFIX_PATH` accordingly (see Step 5).

### Step 3 — Install MuJoCo

Download the MuJoCo release for Linux:

```bash
# Download MuJoCo 2.3.7 (or latest)
wget https://github.com/google-deepmind/mujoco/releases/download/2.3.7/mujoco-2.3.7-linux-x86_64.tar.gz
tar -xzf mujoco-2.3.7-linux-x86_64.tar.gz
sudo mv mujoco-2.3.7 /opt/mujoco

# Add to your environment (add this to ~/.bashrc to make permanent)
export MUJOCO_DIR=/opt/mujoco
export LD_LIBRARY_PATH=$MUJOCO_DIR/lib:$LD_LIBRARY_PATH
```

Apply immediately without reopening terminal:

```bash
source ~/.bashrc
```

### Step 4 — Configure the network interface

The Unitree SDK needs to know which network interface is connected to the arm.

Find your interface name:

```bash
ip link show
# Look for the interface connected to the arm, e.g. eth0, enp3s0, ens33
```

Set it in your environment:

```bash
export UNITREE_NETWORK_INTERFACE=eth0   # replace with your actual interface
```

Add to `~/.bashrc` to persist:

```bash
echo 'export UNITREE_NETWORK_INTERFACE=eth0' >> ~/.bashrc
source ~/.bashrc
```

> **Tip:** To verify the arm is reachable, ping its IP. The D1 arm defaults to `192.168.123.110`:
> ```bash
> ping 192.168.123.110
> ```

### Step 5 — Clone this repository

```bash
git clone https://github.com/hanxzh-ops/d1_pick_place.git
cd d1_pick_place
```

---

## Build

```bash
# Create and enter the build directory
mkdir build && cd build

# Configure — adjust paths if your SDK/MuJoCo are in non-default locations
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DMUJOCO_DIR=/opt/mujoco

# Compile (use -j to parallelise across CPU cores)
make -j$(nproc)
```

If the Unitree SDK was installed to a custom prefix:

```bash
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DMUJOCO_DIR=/opt/mujoco \
    -DCMAKE_PREFIX_PATH=/path/to/unitree_sdk2/install
```

On success you will see:

```
[100%] Linking CXX executable deploy
[100%] Built target deploy
```

---

## CMakeLists.txt Reference

For reference, here is the `CMakeLists.txt` used for this project:

```cmake
cmake_minimum_required(VERSION 3.16)
project(d1_pick_place)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Find dependencies ──────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)

# MuJoCo
set(MUJOCO_DIR "/opt/mujoco" CACHE PATH "MuJoCo installation directory")
find_library(MUJOCO_LIB mujoco HINTS ${MUJOCO_DIR}/lib REQUIRED)

# Unitree SDK2
find_package(unitree_sdk2 REQUIRED)

# ── Main deploy executable ─────────────────────────────────────────────
add_executable(deploy
    src/deploy.cpp
    src/kinematics.cpp
)

target_include_directories(deploy PRIVATE
    include/
    ${MUJOCO_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(deploy
    unitree_sdk2
    ${MUJOCO_LIB}
    Eigen3::Eigen
    pthread
)

# ── Drag teaching tool ─────────────────────────────────────────────────
add_executable(drag_teach
    src/drag_teach.cpp
)

target_include_directories(drag_teach PRIVATE
    include/
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(drag_teach
    unitree_sdk2
    pthread
)
```

---

## Configuration

Before running, edit the **absolute path** to your MuJoCo XML model inside `src/deploy.cpp`:

```cpp
// Line ~44 in deploy.cpp — set this to wherever your XML file lives
D1TaskSequence() :
    publisher(TOPIC_CMD),
    ik_engine("/home/YOUR_USERNAME/d1_pick_place/config/d1_kinematics.xml"),
    //         ↑ change this path
```

Also verify the two threshold constants at the top of `deploy.cpp` suit your setup:

```cpp
// How close (degrees) each joint must be before gripper triggers
#define POSITION_THRESHOLD_DEG   3.0    // tighten for precision, loosen for speed

// Maximum wait time before falling back to best-effort
#define POSITION_WAIT_TIMEOUT_MS 8000   // 8 seconds
```

---

## Running

### Step 1 — Power on the arm

Ensure the D1 arm is powered and the Ethernet cable is connected to  PC. Confirm network access:

```bash
ping 192.168.123.110
```

### Step 2 — Enable the arm and launch the controller

From the `build/` directory:

```bash
cd build
./deploy
```

You should see:

```
[SYSTEM] Enabling Arm Motors...
[ACTION] Moving to Folding Pose...
[INFO] Position reached.
Ready! Commands: 'pick x y z'  |  'put x y z'  |  'exit'
>
```

### Step 3 — Send commands

Pick an object at coordinate (0.3, 0.0, 0.1) — XYZ in metres, relative to the arm base:

```
> pick 0.3 0.0 0.1
```

Place it at (0.25, 0.15, 0.1):

```
> put 0.25 0.15 0.1
```

Exit and return the arm to fold:

```
> exit
```

### Coordinate Frame

```
        Z (up)
        │
        │
        └──── Y (left)
       /
      X (forward, away from arm base)

Origin = arm base mounting point
Units  = metres
```

---

## Drag Teaching Tool

Use `drag_teach` to record poses by physically guiding the arm by hand:

```bash
cd build
./drag_teach
```

| Key | Action |
|---|---|
| `f` | FREE all joints — arm goes limp, you can drag it |
| `l` | LOCK all joints — motors hold position |
| `r` | Record current pose as a waypoint |
| `p` | Replay all recorded waypoints in order |
| `s` | Save recorded poses to `recorded_poses.txt` |
| `q` | Quit (motors lock automatically) |

**Typical workflow:**

```
1. Press f    → arm is now freely movable by hand
2. Move arm to position 1 by hand
3. Press r    → waypoint 1 saved
4. Move arm to position 2 by hand
5. Press r    → waypoint 2 saved
6. Press l    → arm locks
7. Press p    → arm replays the recorded motion
8. Press s    → save to file for use in scripts
```

---

## Troubleshooting

**Arm not responding to commands**
- Confirm `ping 192.168.123.110` succeeds
- Check that `UNITREE_NETWORK_INTERFACE` matches the interface connected to the arm (`ip link show`)
- Ensure the arm is powered (LED should be lit) and in a non-error state

**IK solver throws an exception**
- The target XYZ is outside the arm's reachable workspace (~0.6 m radius)
- Check the path to `d1_kinematics.xml` is correct and the file exists
- Try a closer/more central target first to verify IK is working

**Gripper closes before arm reaches position / timeout warning**
- Increase `POSITION_WAIT_TIMEOUT_MS` if the arm moves slowly
- Increase `POSITION_THRESHOLD_DEG` if the arm oscillates around the target
- Check that `rt/arm Feedback` is being received: the live angle display in `drag_teach` will show dashes if no feedback is arriving

**Build error: `unitree_sdk2` not found**
- Confirm the SDK was installed: `ls /usr/local/include/unitree`
- If installed to a custom path, pass `-DCMAKE_PREFIX_PATH=//path` to cmake

**Build error: MuJoCo not found**
- Confirm `libmujoco.so` exists: `ls /opt/mujoco/lib`
- Pass the correct path: `cmake .. -DMUJOCO_DIR=//mujoco/path`

---

## How It Works — Quick Reference

```
performPick(x, y, z)
│
├─ solve_ik(x,y,z)           → compute joint angles from Cartesian target
├─ moveToFold()              → safe intermediate pose
├─ moveJoints(OPEN)          → open gripper
├─ moveJoints(target, OPEN)  → move arm to target pose
├─ waitUntilReached()        → poll rt/arm Feedback until joints within 3°
├─ moveJoints(target, CLOSE) → close gripper NOW (arm is confirmed there)
└─ moveToFold()              → retreat to home
```

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

## References

- [Unitree D1 Arm Development Guide](https://support.unitree.com/home/en/developer/D1Arm_services)
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)
- [MuJoCo Documentation](https://mujoco.readthedocs.io)
