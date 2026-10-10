# Rover EKF

This document describes the `RoverEKF` implementation in `Rover_EKF_/RoverEKF.cpp`. It is a C++ extended Kalman filter (EKF) for estimating the planar pose and motion of a differential-drive rover using wheel encoder, local position, and IMU gyro measurements.

## State and motion model

The filter state is a five-element Eigen column vector:

| Index | Symbol | Meaning | Unit |
| ---: | --- | --- | --- |
| 0 | `px` | Local Cartesian X position | m |
| 1 | `py` | Local Cartesian Y position | m |
| 2 | `theta` | Heading | rad |
| 3 | `v` | Forward velocity | m/s |
| 4 | `omega` | Angular velocity about the vertical axis | rad/s |

Assuming constant forward and angular velocity during a prediction interval `dt`, the motion model is:

```text
px'    = px + v cos(theta) dt
py'    = py + v sin(theta) dt
theta' = normalize(theta + omega dt)
v'     = v
omega' = omega
```

Heading is normalized to `[-pi, pi]`. Prediction propagates covariance with the motion-model Jacobian and process-noise matrix `Q`. Measurement updates solve for the Kalman gain using LDLT and update covariance in Joseph form. Covariance is symmetrized after prediction and correction.

## Measurement models

| Measurement | Expected values | Method |
| --- | --- | --- |
| Local position | X and Y in metres | `updateGPS(x, y, sigma_x, sigma_y)` |
| Wheel encoder | Forward speed and angular speed | `updateEncoder(v, omega, sigma_v, sigma_omega)` |
| IMU gyro | Z-axis angular rate in rad/s | `updateGyro(gyro_z, sigma_gyro)` |

The position update expects local Cartesian coordinates in metres, not latitude and longitude. Convert GPS fixes before passing them to the filter. The gyro measurement observes angular velocity directly. The state does not include gyro bias, acceleration, or lateral velocity.

## Requirements

- C++11 or newer.
- Eigen 3 headers available to the compiler; the implementation includes `<Eigen/Dense>`.

The implementation currently includes a demonstration `main()` as well as the filter, so it builds as a standalone example. For example, with GCC or Clang and Eigen in a standard include path:

```sh
g++ -std=c++11 -O2 Rover_EKF_/RoverEKF.cpp -o rover_ekf_demo
./rover_ekf_demo
```

If Eigen is installed outside a standard include path, add its parent directory using the compiler's `-I` option.

## `RoverEKF` API

### Types and initialization

```cpp
static constexpr int STATE_SIZE = 5;
using StateVector = Eigen::Matrix<double, 5, 1>;
using StateMatrix = Eigen::Matrix<double, 5, 5>;

RoverEKF();
void setState(const StateVector& initial_state);
void setCovariance(const StateMatrix& initial_covariance);
void setProcessNoise(const StateMatrix& process_noise);
const StateVector& getState() const;
const StateMatrix& getCovariance() const;
```

The constructor initializes the state to zero, covariance to identity, and process noise to example diagonal values. The filter requires an explicit `setState` call before `predict`; this call also marks it initialized. `predict` throws `std::runtime_error` if the state has not been initialized and `std::invalid_argument` when `dt <= 0`.

`setCovariance` and `setProcessNoise` accept full 5×5 matrices. Covariance entries are variances or covariances in the corresponding state units. The example process-noise values are placeholders and should be tuned for the rover and prediction cadence.

### Prediction and measurement updates

```cpp
void predict(double dt);
void updateGPS(double gps_x, double gps_y,
               double sigma_x, double sigma_y);
void updateEncoder(double encoder_velocity, double encoder_omega,
                   double sigma_v, double sigma_omega);
void updateGyro(double gyro_z, double sigma_gyro);
```

`dt` is elapsed time in seconds. Each `sigma_*` argument is a measurement standard deviation in the same units as its measurement. The convenience methods square these values to construct diagonal measurement-noise matrices.

The generic fixed-size `update<MEASUREMENT_SIZE>(z, h, H, R)` method supports custom measurement models. Supply the measurement vector `z`, predicted measurement `h(x)`, measurement Jacobian `H`, and measurement-noise covariance `R`.

## Differential-drive encoder conversion

```cpp
struct EncoderConfig {
    double wheel_radius_m;
    double wheel_base_m;
    int ticks_per_revolution;
};

struct EncoderMeasurement {
    double velocity;
    double angular_velocity;
};

EncoderMeasurement encoderToVelocity(
    long long left_ticks,
    long long right_ticks,
    double dt,
    const EncoderConfig& config);
```

The tick arguments are increments during the interval, not cumulative totals. The helper converts ticks to wheel distances using wheel circumference, divides by `dt` to get wheel speeds, then calculates:

```text
v     = (v_right + v_left) / 2
omega = (v_right - v_left) / wheel_base
```

Configure ticks per revolution to match the hardware's counting convention, including any quadrature multiplier. The helper checks that `dt > 0`; it does not validate wheel dimensions or tick resolution. Supply physically valid positive configuration values.

## Typical filter cycle

Initialize the filter once, then for each time interval convert encoder tick increments, predict, and apply available sensor updates:

```cpp
RoverEKF ekf;

RoverEKF::StateVector initial_state;
initial_state << 0.0, 0.0, 0.0, 0.0, 0.0;
ekf.setState(initial_state);

RoverEKF::StateMatrix initial_covariance = RoverEKF::StateMatrix::Identity();
ekf.setCovariance(initial_covariance);

EncoderConfig config{0.05, 0.30, 2048}; // radius [m], wheel base [m], ticks/rev
const double dt = 0.02;                 // seconds
const EncoderMeasurement wheels = encoderToVelocity(20, 22, dt, config);

ekf.predict(dt);
ekf.updateEncoder(wheels.velocity, wheels.angular_velocity, 0.05, 0.03);
ekf.updateGPS(0.08, 0.01, 2.0, 2.0); // local X/Y [m], standard deviations [m]
ekf.updateGyro(0.02, 0.02);            // gyro Z [rad/s], standard deviation [rad/s]

const RoverEKF::StateVector& state = ekf.getState();
// state(0..4) = px, py, theta, v, omega
```

The dimensions and sensor values in this example are illustrative. For a rover, initialize from its startup estimate, derive `dt` from timestamps, use encoder increments for the same interval, and tune process and measurement noise from observed sensor behavior.

## Assumptions and limitations

- The model is planar and assumes a differential-drive rover with no lateral motion.
- There is no GPS coordinate conversion, timestamp synchronization, outlier rejection, measurement gating, logging, or hardware I/O.
- The convenience measurement methods assume independent measurement axes because their noise matrices are diagonal. Use the generic update for correlated measurement noise.
- The convenience update methods do not validate standard deviations. Supply finite, nonnegative uncertainties that produce a well-conditioned innovation covariance.
- Measurement updates do not check whether `setState` has been called; initialize before using them.
- Process noise is added once per prediction, so tune it with the chosen prediction interval in mind.
