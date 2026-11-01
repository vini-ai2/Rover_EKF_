# Rover_EKF status

## Change log

- Added an installable CMake `RoverEKF::RoverEKF` interface target with Eigen3 dependency export.
- Consolidated implementation on `include/RoverEKF.hpp`; kept `RoverEKF_impl.hpp` as a compatibility include and the old source translation unit as a compatibility shim.
- Added focused tests for prediction, updates, instance independence, encoder conversion, and invalid inputs.
- Updated the standalone demo to include the public library header.

## Verification

- CMake configure/build: passed with GCC 11.4 and Eigen3 3.4.
- CTest: 1 focused test executable passed.
- Install and downstream `find_package(RoverEKF)` consumer build: passed.
- Standalone demo C++11 compile: passed.
