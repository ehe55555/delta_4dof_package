# Delta 4DOF

ROS 2 Jazzy + Gazebo Harmonic simulation and control GUI.

Prerequisite: Ubuntu 24.04 with the ROS 2 Jazzy apt repository configured.
The launcher checks ROS 2, but it does not configure the ROS apt repository or
install the ROS distribution itself.

## Portable folder

The source package only needs:

- `src/`
- `RUN_DELTA_4DOF.sh`
- `run_delta_control.sh`
- `run_gazebo.sh`
- `RUN_DELTA_4DOF.desktop`
- `README.md`

Do not copy `.git/`, `build/`, `install/` or `log/` to another machine.
Those directories are generated locally.

Create a clean folder:

```bash
./RUN_DELTA_4DOF.sh --package
```

The default output is a sibling folder named `delta_4dof_package`.
An explicit destination can also be used:

```bash
./RUN_DELTA_4DOF.sh --package /path/to/delta_4dof
```

## First run

Open a terminal in the copied folder:

```bash
chmod +x RUN_DELTA_4DOF.sh
./RUN_DELTA_4DOF.sh --install-deps
```

The launcher will:

1. Check ROS 2 Jazzy, Gazebo, colcon and Python modules.
2. Install missing project dependencies when `--install-deps` is supplied.
3. Build the workspace.
4. Create `Delta 4DOF Control` on the Desktop.
5. Start Gazebo, bridges and the control GUI.

The adjacent `RUN_DELTA_4DOF.desktop` can also be opened with
**Run as a Program** for the first run.

## Later runs

Open `Delta 4DOF Control` on the Desktop or run:

```bash
./RUN_DELTA_4DOF.sh
```

The launcher automatically rebuilds when a file under `src/` is newer than
the current installation.

## Commands

```bash
./RUN_DELTA_4DOF.sh --check
./RUN_DELTA_4DOF.sh --build --no-run
./RUN_DELTA_4DOF.sh --rebuild --no-run
./RUN_DELTA_4DOF.sh --install-desktop --no-run
./RUN_DELTA_4DOF.sh --package
```

The latest launcher output is saved in `last_run_delta_4dof.log`.

## Joint controller

The three active Delta joints use:

```text
torque = Kp * (q_ref - q)
       + Ki * integral(q_ref - q)
       + Kd * (qd_ref - qd)
       + Kv * qd_ref
       + Ka * qdd_ref
```

The tuned values are in `src/descripe/models/descripe/model.sdf`:

```xml
<kp>4.0</kp>
<ki>1.0</ki>
<kd>0.18</kd>
<kv>0.020</kv>
<ka>0.0</ka>
<torque_limit>0.25</torque_limit>
<integral_limit>0.25</integral_limit>
<anti_windup_gain>5.0</anti_windup_gain>
```

`Kv` compensates velocity-proportional joint load. The controller also uses
back-calculation anti-windup so the integral term recovers after torque
saturation. The separate `twist_kp`, `twist_kd` and `twist_torque_limit`
settings stabilize passive-link rotation and are not part of the active-joint
PID.

## Workspace tracking benchmark

The workspace planner merges only collinear voxel runs, so a full scan still
passes through every workspace voxel. Each segment uses quintic interpolation
and is automatically slowed when a joint would exceed `60 deg/s` or
`150 deg/s^2`.

With Gazebo and both bridge nodes running, execute:

```bash
ros2 run delta_control workspace_benchmark \
  --divisions 6 \
  --speed 0.04 \
  --preserve-voxels \
  --output /tmp/delta_workspace_benchmark.json
```

The report includes joint position and velocity errors, Cartesian error,
peak torque, saturation percentage and the coordinates of the worst samples.
