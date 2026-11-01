#include <cmath>
#include <iostream>
#include <stdexcept>

#include "RoverEKF.hpp"

namespace {

void require(bool condition, const char *message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testPrediction()
{
    RoverEKF ekf;
    RoverEKF::StateVector state;
    state << 0.0, 0.0, 0.0, 1.0, 0.0;
    ekf.setState(state);
    ekf.setProcessNoise(RoverEKF::StateMatrix::Zero());

    ekf.predict(0.5);
    require(std::abs(ekf.getState()(0) - 0.5) < 1e-12,
            "straight-line prediction x");
    require(std::abs(ekf.getState()(1)) < 1e-12,
            "straight-line prediction y");
    require(std::abs(ekf.getCovariance()(0, 0) - 1.25) < 1e-12,
            "prediction covariance propagation");
}

void testMeasurementUpdates()
{
    RoverEKF ekf;
    RoverEKF::StateVector state = RoverEKF::StateVector::Zero();
    ekf.setState(state);
    ekf.setCovariance(RoverEKF::StateMatrix::Identity());

    ekf.updateEncoder(2.0, 0.5, 0.1, 0.1);
    require(ekf.getState()(3) > 1.9 && ekf.getState()(4) > 0.49,
            "encoder update corrects velocities");

    ekf.updateGyro(1.0, 0.1);
    require(ekf.getState()(4) > 0.7,
            "gyro update corrects angular velocity");

    ekf.updateGPS(4.0, -2.0, 0.1, 0.1);
    require(ekf.getState()(0) > 3.9 && ekf.getState()(1) < -1.9,
            "GPS update corrects position");
    require((ekf.getCovariance() - ekf.getCovariance().transpose()).norm() < 1e-12,
            "measurement covariance remains symmetric");
}

void testIndependentInstances()
{
    RoverEKF local;
    RoverEKF global;
    local.setState(RoverEKF::StateVector::Zero());
    global.setState(RoverEKF::StateVector::Zero());

    local.updateEncoder(1.0, 0.0, 0.1, 0.1);
    global.updateEncoder(1.0, 0.0, 0.1, 0.1);
    global.updateGPS(5.0, 3.0, 0.1, 0.1);

    require(std::abs(local.getState()(0)) < 1e-12 &&
            std::abs(local.getState()(1)) < 1e-12,
            "global position measurement does not change local instance");
    require(global.getState()(0) > 4.9 && global.getState()(1) > 2.9,
            "global instance accepts position measurement");
}

void testEncoderConversion()
{
    EncoderConfig config{0.1, 0.5, 100};
    const auto forward = encoderToVelocity(100, 100, 1.0, config);
    const double expected_wheel_speed = 2.0 * 3.14159265358979323846 * 0.1;
    require(std::abs(forward.velocity - expected_wheel_speed) < 1e-12,
            "encoder conversion linear speed");
    require(std::abs(forward.angular_velocity) < 1e-12,
            "equal wheel increments have zero yaw rate");

    const auto turn = encoderToVelocity(0, 100, 1.0, config);
    require(turn.angular_velocity > 0.0,
            "right wheel faster gives positive yaw rate");
}

void testInvalidInputs()
{
    RoverEKF ekf;
    bool predict_rejected = false;
    try {
        ekf.predict(0.1);
    } catch (const std::runtime_error &) {
        predict_rejected = true;
    }
    require(predict_rejected, "prediction requires initialized state");

    EncoderConfig config{0.1, 0.5, 100};
    bool dt_rejected = false;
    try {
        (void)encoderToVelocity(1, 1, 0.0, config);
    } catch (const std::invalid_argument &) {
        dt_rejected = true;
    }
    require(dt_rejected, "encoder conversion rejects nonpositive dt");
}

}  // namespace

int main()
{
    try {
        testPrediction();
        testMeasurementUpdates();
        testIndependentInstances();
        testEncoderConversion();
        testInvalidInputs();
    } catch (const std::exception &error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return 1;
    }
    std::cout << "All RoverEKF tests passed\n";
    return 0;
}
