#include <cmath>
#include <iostream>
#include <stdexcept>
#include "RoverEKF.hpp"

int main()
{

    // Create EKF


    RoverEKF ekf;


    // Initial state
    // [px, py, theta, v, omega]
    // Start at origin, stationary.

    RoverEKF::StateVector initial_state;

    initial_state <<
        0.0,    // px
        0.0,    // py
        0.0,    // theta
        0.0,    // v
        0.0;    // omega


    ekf.setState(
        initial_state
    );


    // Initial covariance - placeholders
  

    RoverEKF::StateMatrix P0;

    P0.setZero();


    P0(0, 0) =
        1.0;       // px variance


    P0(1, 1) =
        1.0;       // py variance


    P0(2, 2) =
        0.1;       // theta variance


    P0(3, 3) =
        1.0;       // velocity variance


    P0(4, 4) =
        0.1;       // angular velocity variance


    ekf.setCovariance(
        P0
    );


    // Rover wheel configuration
    // just placeholders that we need to replace with our vals

    EncoderConfig encoder_config;

    encoder_config.wheel_radius_m =
        0.05;

    encoder_config.wheel_base_m =
        0.30;

    encoder_config.ticks_per_revolution =
        2048;


    // --------------------------------------------------------
    // One example sensor cycle
    //
    // Imagine this loop runs repeatedly.
    // --------------------------------------------------------

    double dt =
        0.02;       // 50 Hz


    // 1. ENCODERS

    // These should be TICK DIFFERENCES during this timestep,
    // not total accumulated ticks.

    long long left_ticks =
        20;

    long long right_ticks =
        22;


    EncoderMeasurement encoder =
        encoderToVelocity(
            left_ticks,
            right_ticks,
            dt,
            encoder_config
        );


    // 2. PREDICT

    ekf.predict(dt);


    // 3. ENCODER UPDATE

    ekf.updateEncoder(
        encoder.velocity,
        encoder.angular_velocity,

        // PLACEHOLDER sensor standard deviations
        0.05,       // velocity sigma [m/s]
        0.03        // angular velocity sigma [rad/s]
    );


    // 4. GPS UPDATE
    // These are LOCAL X/Y COORDINATES IN METRES.
    // NOT latitude/longitude.
    // GPS conversion will be added separately.

    double gps_x =
        0.08;

    double gps_y =
        0.01;


    ekf.updateGPS(
        gps_x,
        gps_y,

        // PLACEHOLDER GPS uncertainty
        2.0,        // sigma x [m]
        2.0         // sigma y [m]
    );

    // 5. IMU GYRO UPDATE
   

    // Gyroscope Z measurement in rad/s.

    double gyro_z =
        0.02;


    ekf.updateGyro(
        gyro_z,

        // PLACEHOLDER gyro uncertainty
        0.02
    );


// printing the result, should change this to an output of sorts later

    std::cout
        << "\nEstimated state:\n";

    std::cout
        << "px     = "
        << ekf.getState()(0)
        << " m\n";


    std::cout
        << "py     = "
        << ekf.getState()(1)
        << " m\n";


    std::cout
        << "theta  = "
        << ekf.getState()(2)
        << " rad\n";


    std::cout
        << "v      = "
        << ekf.getState()(3)
        << " m/s\n";


    std::cout
        << "omega  = "
        << ekf.getState()(4)
        << " rad/s\n";


    std::cout
        << "\nCovariance P:\n"
        << ekf.getCovariance()
        << "\n";


    return 0;
}




        
        
        

