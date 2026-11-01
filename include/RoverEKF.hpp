#ifndef ROVER_EKF_HEADER_ONLY_HPP
#define ROVER_EKF_HEADER_ONLY_HPP
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>


class RoverEKF
{
public:

    //constant expression 
    static constexpr int STATE_SIZE = 5;

    using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
    using StateMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

    RoverEKF()
    {
        x_.setZero();

        P_.setIdentity();

        Q_.setZero();

        initialized_ = false;

        //placeholders of process noise  
        Q_(0, 0) = 0.01;     // px
        Q_(1, 1) = 0.01;     // py
        Q_(2, 2) = 0.001;    // theta
        Q_(3, 3) = 0.05;     // v
        Q_(4, 4) = 0.01;     // omega
    }


    //initializations of the state, covariance and process noise

    void setState(const StateVector& initial_state)
    {
        x_ = initial_state;
        initialized_ = true;
    }


    void setCovariance(const StateMatrix& initial_covariance)
    {
        P_ = initial_covariance;
    }


    void setProcessNoise(const StateMatrix& process_noise)
    {
        Q_ = process_noise;
    }


// all the return fns 

    const StateVector& getState() const
    {
        return x_;
    }


    const StateMatrix& getCovariance() const
    {
        return P_;
    }

//1. Prediction
// x^- = f(x)
// P^- = F P F^T + Q

    void predict(double dt)
    {
        StateMatrix F = StateMatrix::Identity();
        if (!initialized_)
        {
            throw std::runtime_error(
                "EKF has not been initialized."
            );
        }


        if (dt <= 0.0)
        {
            throw std::invalid_argument(
                "dt must be greater than zero."
            );
        }
    
        //exrtacting the current states present in x[5] belongs to R5


        double px    = x_(0);
        double py    = x_(1);
        double theta = x_(2);
        double v     = x_(3);
        double omega = x_(4);

        /* Nonlinear motion model
         px_new = px + v*cos(theta)*dt
         py_new = py + v*sin(theta)*dt
         theta_new = theta + omega*dt
           v_new = v
         omega_new = omega*/


        x_(0) =
            px + v * std::cos(theta) * dt;

        x_(1) =
            py + v * std::sin(theta) * dt;

        x_(2) =
            normalizeAngle(
                theta + omega * dt
            );

        x_(3) = v;

        x_(4) = omega;

        /* Process Jacobian
         F = df/dx
         [ 1  0  -v sin(theta)dt   cos(theta)dt  0 ]
         [ 0  1   v cos(theta)dt   sin(theta)dt  0 ]
         [ 0  0        1                0       dt]
         [ 0  0        0                1        0 ]
         [ 0  0        0                0        1 ]*/


        F(0, 2) =
            -v * std::sin(theta) * dt;

        F(0, 3) =
            std::cos(theta) * dt;


        F(1, 2) =
            v * std::cos(theta) * dt;

        F(1, 3) =
            std::sin(theta) * dt;


        F(2, 4) =
            dt;
        

// covariance prediction - P^- = F P F^T + Q

        P_ =
            F * P_ * F.transpose()
            + Q_;


        // Keep P numerically symmetric
        P_ =
            0.5 * (P_ + P_.transpose());
    }

    /* generic measurement update 
    y = z - h(x)
    S = H P H^T + R
    K = P H^T S^-1
    x = x + Ky
    P = (I-KH)P(I-KH)^T + KRK^T
    */

    template<int MEASUREMENT_SIZE>

    void update(
        const Eigen::Matrix<double, MEASUREMENT_SIZE, 1>& z,
        const Eigen::Matrix<double, MEASUREMENT_SIZE, 1>& h,
        const Eigen::Matrix<double,
                            MEASUREMENT_SIZE,
                            STATE_SIZE>& H,
        const Eigen::Matrix<double,
                            MEASUREMENT_SIZE,
                            MEASUREMENT_SIZE>& R
    )
    {

        // y = z - h(x)

        Eigen::Matrix<double, MEASUREMENT_SIZE, 1>
            innovation;

        innovation =
            z - h;



        // S = H P H^T + R

        Eigen::Matrix<double,
                      MEASUREMENT_SIZE,
                      MEASUREMENT_SIZE>
            S;

        S =
            H * P_ * H.transpose()
            + R;


        //K = P H^T S^-1
        Eigen::Matrix<double,
                      STATE_SIZE,
                      MEASUREMENT_SIZE>
            K;


        K =
            P_
            * H.transpose()
            * S.ldlt().solve(
                Eigen::Matrix<double,
                              MEASUREMENT_SIZE,
                              MEASUREMENT_SIZE>::Identity()
            );

// state correction:    x = x + Ky

        x_ =
            x_ + K * innovation;


        // Heading must remain within [-pi, pi]
        x_(2) =
            normalizeAngle(x_(2));


        // Covariance correction
        // Joseph form:
        // P =
        // (I-KH)P(I-KH)^T + KRK^T
        // This is more numerically stable.

        StateMatrix I =
            StateMatrix::Identity();


        StateMatrix A =
            I - K * H;


        P_ =
            A * P_ * A.transpose()
            + K * R * K.transpose();


        // Keep covariance symmetric
        P_ =
            0.5 * (P_ + P_.transpose());
    }



    // GPS UPDATE
    // Measurement:
    // z_GPS = [px_gps]
    //         [py_gps]
    // h(x) = [px]
    //        [py]


    void updateGPS(
        double gps_x,
        double gps_y,
        double sigma_x,
        double sigma_y
    )
    {

        
        // Actual measurement

        Eigen::Vector2d z;

        z << gps_x,
             gps_y;


        // Predicted measurement
        // h(x)

        Eigen::Vector2d h;

        h << x_(0),
             x_(1);


        // Measurement Jacobian
        // H =
        // [1 0 0 0 0]
        // [0 1 0 0 0]


        Eigen::Matrix<double, 2, STATE_SIZE> H;

        H.setZero();

        H(0, 0) = 1.0;
        H(1, 1) = 1.0;


        // Measurement noise covariance
        // R =
        // [sigma_x^2     0   ]
        // [    0     sigma_y^2]


        Eigen::Matrix2d R;

        R.setZero();

        R(0, 0) =
            sigma_x * sigma_x;

        R(1, 1) =
            sigma_y * sigma_y;


        // Generic EKF update

        update<2>(
            z,
            h,
            H,
            R
        );
    }


    // ENCODER UPDATE
    // Measurement:
    // z_encoder = [v_encoder]
    //             [omega_encoder]
    // h(x) = [v]
    //        [omega]
 

    void updateEncoder(
        double encoder_velocity,
        double encoder_omega,
        double sigma_v,
        double sigma_omega
    )
    {

        Eigen::Vector2d z;

        z << encoder_velocity,
             encoder_omega;


        // Predicted measurement

        Eigen::Vector2d h;

        h << x_(3),
             x_(4);


        // H =
        //
        // [0 0 0 1 0]
        // [0 0 0 0 1]

        Eigen::Matrix<double, 2, STATE_SIZE> H;

        H.setZero();

        H(0, 3) = 1.0;
        H(1, 4) = 1.0;


        // Measurement noise

        Eigen::Matrix2d R;

        R.setZero();

        R(0, 0) =
            sigma_v * sigma_v;

        R(1, 1) =
            sigma_omega * sigma_omega;


        update<2>(
            z,
            h,
            H,
            R
        );
    }


    // IMU GYROSCOPE UPDATE
    // For this first version:
    // z_gyro = omega
    // H = [0 0 0 0 1]


    void updateGyro(
        double gyro_z,
        double sigma_gyro
    )
    {

        Eigen::Matrix<double, 1, 1> z;

        z << gyro_z;


        Eigen::Matrix<double, 1, 1> h;

        h << x_(4);


        Eigen::Matrix<double, 1, STATE_SIZE> H;

        H.setZero();

        H(0, 4) = 1.0;


        Eigen::Matrix<double, 1, 1> R;

        R(0, 0) =
            sigma_gyro * sigma_gyro;


        update<1>(
            z,
            h,
            H,
            R
        );
    }


    // ANGLE NORMALIZATION

    static double normalizeAngle(double angle)
    {
        constexpr double PI =
            3.14159265358979323846;


        while (angle > PI)
            angle -= 2.0 * PI;


        while (angle < -PI)
            angle += 2.0 * PI;


        return angle;
    }


private:

    // THE ACTUAL EKF MEMORY

    StateVector x_;

    StateMatrix P_;

    StateMatrix Q_;

    bool initialized_;
};


// DIFFERENTIAL DRIVE ENCODER PROCESSING

struct EncoderConfig
{
    double wheel_radius_m;

    double wheel_base_m;

    int ticks_per_revolution;
};


struct EncoderMeasurement
{
    double velocity;

    double angular_velocity;
};


// Convert encoder tick differences into:
// v
// omega

EncoderMeasurement encoderToVelocity(
    long long left_ticks,
    long long right_ticks,
    double dt,
    const EncoderConfig& config
)
{

    constexpr double PI =
        3.14159265358979323846;


    if (dt <= 0.0)
        throw std::invalid_argument(
            "dt must be greater than zero."
        );


    // Wheel circumference

    double circumference =
        2.0
        * PI
        * config.wheel_radius_m;


    // Distance travelled by each wheel

    double left_distance =
        (static_cast<double>(left_ticks)
         / config.ticks_per_revolution)
        * circumference;


    double right_distance =
        (static_cast<double>(right_ticks)
         / config.ticks_per_revolution)
        * circumference;


    // Wheel velocities

    double left_velocity =
        left_distance / dt;


    double right_velocity =
        right_distance / dt;


    // Differential-drive equations

    double v =
        (right_velocity + left_velocity)
        / 2.0;


    double omega =
        (right_velocity - left_velocity)
        / config.wheel_base_m;


    return {
        v,
        omega
    };
}



#endif
