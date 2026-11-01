# Rover EKF

Reusable C++ extended Kalman filter (EKF) library and the integration guide for
the rover's dual-estimator ROS 2/Nav2 setup. The library is maintained in this
repository. The ROS 2 pipeline, PID firmware, and rover model are in separate
repositories; this README links the relevant source areas and describes how
they work together.

## Contents

- [System overview](#system-overview)
- [Repository and change map](#repository-and-change-map)
- [Frames, topics, and sensor flow](#frames-topics-and-sensor-flow)
- [Build and install the C++ library](#build-and-install-the-c-library)
- [Use the C++ API](#use-the-c-api)
- [Build and run the ROS 2 pipeline](#build-and-run-the-ros-2-pipeline)
- [Run Nav2](#run-nav2)
- [Parameters and calibration](#parameters-and-calibration)
- [Timestamp and update behavior](#timestamp-and-update-behavior)
- [Verification and known requirements](#verification-and-known-requirements)

## System overview

The filter estimates a planar five-value state:

| Index | State | Meaning | Unit |
| ---: | --- | --- | --- |
| 0 | `px` | X position | m |
| 1 | `py` | Y position | m |
| 2 | `theta` | Heading | rad |
| 3 | `v` | Forward velocity | m/s |
| 4 | `omega` | Vertical angular velocity | rad/s |

Prediction assumes constant forward and angular velocity over `dt`:

```text
px'    = px + v cos(theta) dt
py'    = py + v sin(theta) dt
theta' = normalize(theta + omega dt)
v'     = v
omega' = omega
```

The filter propagates covariance with the motion Jacobian and process noise.
Measurement corrections use a Kalman gain solved with LDLT and update the
covariance in Joseph form.

ROS 2 creates two independent `RoverEKF` objects, each with its own state and
covariance:

| Estimator | Wheel encoders | IMU gyro Z | GPS position | Purpose |
| --- | --- | --- | --- | --- |
| Local | Yes | Yes | No | Smooth continuous pose in `odom`; owns `odom → base_link` |
| Global | Yes | Yes | Yes | GPS-referenced pose in `map`; supports `map → odom` correction |

Every accepted encoder sample and IMU gyro sample is applied to both filters.
Each accepted GPS fix is applied only to the global filter. A 20 Hz timer
predicts and publishes both estimates. This keeps the local transform
continuous while GPS corrections move the global transform.

## Repository and change map

| Project | Location | Change record |
| --- | --- | --- |
| Rover EKF library (this repo) | `Rover_EKF_` | Installable header-only `RoverEKF::RoverEKF` CMake target with Eigen export; compatibility headers/source retained; demo uses public header; tests cover prediction, updates, independent instances, encoder conversion, and invalid inputs. |
| ROS 2 sensor pipeline | `ros2_ws/src/rover_sensor_pipeline` | `sensor_ingestion_node.cpp` owns both filters and sensor conversion; `rover_nav2.launch.py` and `config/nav2_params.yaml` add Nav2 bringup; CMake consumes the installed library; `test/test_tf_composition.cpp` checks transform composition. |
| PID firmware | `PID` | `src/BasicMotorControl.cpp` publishes `/motor_rpms`, `/gps/fix`, and `/imu/data`; the GPS frame ID was aligned to `gps_link`. RPM arrays remain six signed values in FL, FR, ML, MR, BL, BR order without a source timestamp. |
| Rover model/control | `CSE_Recruitment` | `urdf/rover.xacro` activates sensor frames; `config/rover_controllers.yaml` disables diff-drive odometry TF so the local EKF owns that edge. Gazebo sensor plugins remain excluded because the included plugins target Ignition but this launch uses Gazebo Classic. |

The detailed ROS package guide is at
[`rover_sensor_pipeline/README.md`](../ros2_ws/src/rover_sensor_pipeline/README.md).
Each project has a `STATUS.md` with its verification record. In the model
repository, existing unrelated working-tree edits and generated files were
preserved; they are not part of the EKF/Nav2 changes described here.
The ROS workspace is not inside this library repository, so build it as a
separate workspace and make the installed library visible through
`CMAKE_PREFIX_PATH`.

Library files:

- `include/RoverEKF.hpp`: public API and implementation.
- `include/RoverEKF_impl.hpp`: compatibility include for older clients.
- `src/RoverEKFLibrary.cpp`: compatibility translation unit; new clients use
  the public header and CMake target.
- `RoverEKF.cpp`: standalone example executable.
- `tests/RoverEKFTests.cpp`: prediction, measurement, independence, encoder
  conversion, and invalid-input tests.
- `CMakeLists.txt` and `cmake/`: build, install, and `find_package` export.

## Frames, topics, and sensor flow

The intended TF tree is:

```text
map
└── odom
    └── base_link
```

The local estimator broadcasts `odom → base_link`. After the first valid GPS
fix establishes the global origin, the node computes
`map → odom = T_map_base_link × inverse(T_odom_base_link)`. The controller's
`enable_odom_tf` is false to maintain one publisher for `odom → base_link`.
The node publishes:

| Topic | Message | Frames / purpose |
| --- | --- | --- |
| `/odometry/local` | `nav_msgs/msg/Odometry` | `odom` to `base_link`; local pose and velocity |
| `/odometry/global` | `nav_msgs/msg/Odometry` | `map` to `base_link`; published after GPS origin is known |
| `/motor_rpms` | `std_msgs/msg/Float32MultiArray` | Firmware encoder input; six values ordered FL, FR, ML, MR, BL, BR |
| `/joint_states` | `sensor_msgs/msg/JointState` | Alternative encoder source for simulation |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gyroscope Z in rad/s; updates both estimators |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | Latitude/longitude; updates global estimator only |
| `/scan` | `sensor_msgs/msg/LaserScan` | Nav2 costmap input; must be supplied by the robot/simulator |

For firmware RPM input, the node averages the three left and three right motor
speeds, converts RPM to wheel linear speed using the configured radius, then
computes `v = (v_left + v_right)/2` and
`omega = (v_right - v_left)/wheel_base`. For joint states it maps the known
rover joint names to left/right sides and performs the same conversion.

The PID firmware integration already publishes `/imu/data`: the firmware
samples its BNO055 via `AsyncBno055` and publishes the resulting `sensor_msgs/msg/Imu`.
The PID controller itself is not the source of the IMU measurement. Its
best-effort QoS is matched by the ingestion node. Firmware RPM messages have
no header timestamp. GPS has frame ID `gps_link`, corresponding to the model's
antenna frame.

The GPS node uses the first valid fix as a local ENU origin, converts later
latitude/longitude fixes to approximate east/north metres, and compensates for
the configured GPS antenna offset to estimate `base_link` position. This is a
local tangent-plane approximation; it is not a geodetic map alignment system.

## Build and install the C++ library

Requirements: C++11 compiler, CMake 3.16 or newer, and Eigen3 3.3 or newer.

From the root of this repository:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /tmp/rover_ekf_prefix
```

Downstream CMake projects can use:

```cmake
find_package(RoverEKF CONFIG REQUIRED)
target_link_libraries(my_node PRIVATE RoverEKF::RoverEKF)
```

`RoverEKF::RoverEKF` exports the public include directory, C++11 requirement,
and Eigen3 dependency. The library is header-only, so consumers should include
`RoverEKF.hpp` and do not need to compile the compatibility `.cpp` file.

To build the standalone demo:

```sh
g++ -std=c++11 -O2 -Iinclude RoverEKF.cpp -o /tmp/rover_ekf_demo
/tmp/rover_ekf_demo
```

If Eigen is outside the compiler's default include path, add the appropriate
`-I` path.

## Use the C++ API

The filter exposes state/covariance setup and prediction plus the three sensor
corrections:

```cpp
RoverEKF ekf;
RoverEKF::StateVector state = RoverEKF::StateVector::Zero();
ekf.setState(state);  // required before predict()
ekf.setCovariance(RoverEKF::StateMatrix::Identity());

ekf.predict(dt_seconds);
ekf.updateEncoder(forward_mps, yaw_rate_radps, sigma_v, sigma_omega);
ekf.updateGyro(gyro_z_radps, sigma_gyro_radps);
ekf.updateGPS(local_x_m, local_y_m, sigma_x_m, sigma_y_m);

const RoverEKF::StateVector &estimate = ekf.getState();
// estimate: px, py, theta, v, omega
```

Call only updates for measurements that are available and valid. The position
API expects local Cartesian metres, not latitude/longitude. `sigma_*` values
are standard deviations; the helper methods square them to form measurement
noise. Set process noise for the platform and prediction interval rather than
using example defaults unchanged. The generic fixed-size `update(...)` method
is available for custom measurement models with a measurement vector, model,
Jacobian, and covariance.

The optional `encoderToVelocity()` helper converts left/right encoder tick
increments into linear and angular velocities. Its `dt` must be positive and
ticks must be increments over that interval. Set wheel radius, wheel base, and
ticks per revolution to match the hardware counting convention.

## Build and run the ROS 2 pipeline

The ROS 2 package is at
`/home/vini/Desktop/rover_t/ros2_ws/src/rover_sensor_pipeline` in this setup.
It requires ROS 2 Humble, Eigen3, and the library installed above. Build from
the ROS workspace root:

```sh
source /opt/ros/humble/setup.bash
cmake -S /home/vini/Desktop/rover_t/Rover_EKF_ \
  -B /tmp/rover_ekf_build \
  -DCMAKE_INSTALL_PREFIX=/tmp/rover_ekf_prefix
cmake --build /tmp/rover_ekf_build
cmake --install /tmp/rover_ekf_build

cd /home/vini/Desktop/rover_t/ros2_ws
CMAKE_PREFIX_PATH=/tmp/rover_ekf_prefix:${CMAKE_PREFIX_PATH} \
  colcon build --packages-select rover_sensor_pipeline --symlink-install
source install/setup.bash
```

To run just the sensor node with hardware-style RPM input:

```sh
ros2 run rover_sensor_pipeline sensor_ingestion_node \
  --ros-args -p encoder_source:=motor_rpms
```

To use `/joint_states` instead (for example, from a simulator):

```sh
ros2 run rover_sensor_pipeline sensor_ingestion_node \
  --ros-args -p encoder_source:=joint_states
```

The launch file starts the node for you when bringing up Nav2. Outside Nav2,
make sure the rover description publishes the sensor transforms and only one
node publishes each TF edge.

## Run Nav2

The launch file starts the dual EKF node, map server, Nav2 planner/controller,
smoother, behavior tree navigator, waypoint follower, velocity smoother, and
lifecycle manager. A map YAML is required and must use the same coordinate
alignment as the EKF `map` frame.

Hardware or an already-running robot stack:

```sh
ros2 launch rover_sensor_pipeline rover_nav2.launch.py \
  map:=/absolute/path/to/aligned_map.yaml \
  use_sim_time:=false start_simulation:=false
```

Include the rover simulation launch and use simulated time:

```sh
source /home/vini/Desktop/rover/CSE_Recruitment/install/setup.bash
ros2 launch rover_sensor_pipeline rover_nav2.launch.py \
  map:=/absolute/path/to/aligned_map.yaml \
  use_sim_time:=true start_simulation:=true
```

When `start_simulation` is true, the EKF selects `/joint_states`; otherwise it
selects `/motor_rpms`. The configured Nav2 controller reads local velocity
feedback from `/odometry/local` and sends smoothed commands to
`/rover_drive_controller/cmd_vel_unstamped`.

Before using Nav2, verify that the robot/simulator publishes `/scan`, that the
map is aligned to the GPS-origin `map` frame, and that the drive controller,
sensor transforms, and Nav2 use the same clock setting. The current model has
laser geometry but its Gazebo sensor plugin is excluded, and no map file or
`/scan` publisher is included here. Nav2 bringup is therefore configured but
obstacle-aware navigation needs those runtime inputs.

## Parameters and calibration

The sensor node declares these primary parameters (defaults shown):

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `encoder_source` | `motor_rpms` | `motor_rpms` or `joint_states` |
| `encoder_topic` | `/motor_rpms` | RPM array input |
| `joint_states_topic` | `/joint_states` | JointState input |
| `imu_topic` | `/imu/data` | IMU input |
| `gps_topic` | `/gps/fix` | GPS input |
| `wheel_radius` | `0.10` m | Wheel radius for RPM conversion |
| `wheel_base` | `0.50` m | Left/right wheel track for yaw rate |
| `gyro_sigma` | `0.02` rad/s | Gyro measurement standard deviation |
| `velocity_sigma` | `0.05` m/s | Encoder forward-speed standard deviation |
| `yaw_rate_sigma` | `0.05` rad/s | Encoder yaw-rate standard deviation |
| `gps_sigma` | `3.0` m | Fallback GPS position standard deviation |
| `gyro_z_sign` | `1.0` | IMU mounting sign (`1` or `-1`) |
| `left_wheel_sign`, `right_wheel_sign` | `1.0` | Encoder polarity (`1` or `-1`) |
| `gps_offset_x`, `gps_offset_y` | `0.0`, `0.12` m | Antenna position relative to `base_link` |
| `initial_map_yaw` | `0.0` rad | Starting global heading |
| `max_measurement_age` | `1.0` s | Maximum age for stamped sensor messages |

When GPS reports usable covariance, its horizontal diagonal variances are used;
otherwise `gps_sigma` is used. The current map yaw and sign defaults are
initial guesses. Check wheel polarity and gyro sign with controlled movement,
confirm the antenna offset against the assembled robot, and align the map to
the ENU origin before navigation. The Nav2 costmap robot radius is configured
as `0.55` m and should be checked against the actual rover footprint.

## Timestamp and update behavior

IMU, GPS, and JointState samples with zero, stale, too-far-future, or
non-increasing source stamps are rejected. Firmware `/motor_rpms` has no source
timestamp, so the node can only use callback receipt time and cannot reject
old/reordered RPM samples. Accepted samples update filters immediately; the
node does not rewind and replay delayed measurements. A 20 Hz timer predicts
each filter and stamps odometry and transforms with the prediction-cycle time.
All stamped sensor streams and the node must use compatible ROS clocks.

## Verification and known requirements

Previously completed verification recorded in the project status files:

- Library CMake configure/build, CTest, install, downstream `find_package`
  consumer build, and standalone C++11 demo compilation passed.
- ROS package `colcon build` and its transform composition test passed; launch
  Python and Nav2 YAML syntax checks passed.
- PID firmware build for Teensy 4.1 passed.
- Rover model Xacro expansion passed.

Nav2 packages were not installed in the environment where these changes were
made, so a Nav2 runtime launch was not verified. No `/scan` source or aligned
map file is included. Wheel/gyro signs, GPS origin/map alignment, and physical
footprint still need validation on the target rover. See each project's
`STATUS.md` for the detailed per-repository record.
