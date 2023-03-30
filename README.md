# Fixposition GNSS Transformation Lib

-   [Ubuntu 20.04 / 22.04 ![](./../../actions/workflows/build_test_cmake.yml/badge.svg)](./../../actions/workflows/build_test_cmake.yml)
-   [ROS1 melodic / noetic ![](./../../actions/workflows/build_test_ros.yml/badge.svg)](./../../actions/workflows/build_test_ros.yml)
-   [ROS2 foxy / humble ![](./../../actions/workflows/build_test_ros2.yml/badge.svg)](./../../actions/workflows/build_test_ros2.yml)

`fixposition_gnss_tf`

This is a simple C++ library for geodetic coordinate transformations. This should also serve as an example of how to calculate spatial coordinate transformations between the following coordinate frames:

-   ECEF: Earth-Center-Earth-Fixed
-   ENU: East-North-Up (Local tangiantial plane)
-   LLH: Latitude, Longitude, Height, based on WGS84 Ellipsoid

## Dependencies

-   [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), tested with version [3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7)
-   [CMake](https://cmake.org/)
-   [Yaml-Cpp](https://yaml-cpp.docsforge.com/)

For the tests:

-   [Googletest](https://github.com/google/googletest)

Build system:

-   Plain CMake
-   catkin (for ROS1)
-   colcon (for ROS2)

## Installing dependencies on Ubuntu system

```
 sudo apt update
 sudo apt install -y wget
 wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 sudo apt-get update
 sudo apt install -y build-essential cmake
 sudo apt install -y libeigen3-dev
 sudo apt install -y libyaml-cpp-dev
```

if you also want to run the unit-tests:

```
 sudo apt-get install libgtest-dev
 cd /usr/src/gtest
 sudo cmake .
 sudo make
 sudo make install
```

## Build

### Using catkin (ROS1):

-   Create a ament workspace with the structure:

```
catkin_ws
└── src
    └── fixposition_gnss_tf
    └── <Other Pacakges>
```

-   in the workspace do:

```bash
catkin build fixposition_gnss_tf
```

-   and to build and run unit tests

```bash
catkin build fixposition_gnss_tf -DBUILD_TESTING=ON
catkin run_tests fixposition_gnss_tf
```

### Using colcon (ROS2):

-   Create a ament workspace with the structure:

```
ament_ws
└── src
    └── fixposition_gnss_tf
    └── <Other Pacakges>
```

-   in the workspace do:

```bash
colcon build --packages-select fixposition_gnss_tf
```

-   and to build and run unit tests

```bash
# build with testing enabled
colcon build --packages-select fixposition_gnss_tf --cmake-args -DBUILD_TESTING=ON
# run the tests
colcon test --packages-select fixposition_gnss_tf
# check the results
colcon test-result --test-result-base build/fixposition_gnss_tf/
```

### Using CMake:

-   setup build directory

```bash
mkdir build
cd build
```

-   build without unit tests

```
cmake ..
make
```

-   build and run unit tests

```bash
cmake .. -DBUILD_TESTING=ON
make test
```

## Example Usage

See `test/gnss_test.cpp` for examples of how to use these functions

## Documentation

-   Make sure you have [Doxygen](https://www.doxygen.nl/index.html) installed
-   Run `doxygen Doxyfile` and then open [doc_gen/html/index.html](doc_gen/html/index.html) for the documentation

## Related Readings

-   [Wikipedia - Geographic coordinate conversion](https://en.wikipedia.org/wiki/Geographic_coordinate_conversion)
-   [ESA NAvipedia - Transformations between ECEF and ENU coordinates](https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates)

# License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
