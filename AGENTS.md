# Repository notes

- Keep `RoverEKF` as the single filter implementation; consumers should include `RoverEKF.hpp`.
- Build and test with CMake/CTest. Keep ROS integration in `../ros2_ws` and firmware in `../PID`.
- Record project changes and verification in `STATUS.md`.
