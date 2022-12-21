# Fixposition GNSS Transformation Lib

`fixposition_gnss_tf`

This is a simple C++ library for geodetic coordinate transformations. This should also serve as an example of how to calculate spatial coordinate transformations between the following coordinate frames:

-  ECEF: Earth-Center-Earth-Fixed
-  ENU: East-North-Up (Local tangiantial plane)
-  LLH: Latitude, Longitude, Height, based on WGS84 Ellipsoid

## Dependencies

-  [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), tested with version [3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7)
-  [CMake](https://cmake.org/)
-  [Catkin](http://wiki.ros.org/catkin)
-  [Yaml-Cpp](https://yaml-cpp.docsforge.com/)

For the tests:

-  [Googletest](https://github.com/google/googletest)

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

## Build

### Using colcon: (Recommended)
-  Create a ament workspace with the structure:

```
ament_ws
└── src
    └── gnsstransformationlib
    └── <Other Pacakges>
```

-  in the workspace do:

```bash
colcon build --packages-select fixposition_gnss_tf
```
- and to build and run unit tests

```bash
colcon test --packages-select fixposition_gnss_tf
```

### Using CMake:

```bash
mkdir build
cd build
cmake ..
make
```

## Example Usage

See `test/gnss_test.cpp` for examples of how to use these functions

## Documentation

-  Make sure you have [Doxygen](https://www.doxygen.nl/index.html) installed
-  Run `doxygen Doxyfile` and then open [doc_gen/html/index.html](doc_gen/html/index.html) for the documentation

## Related Readings

-  [Wikipedia - Geographic coordinate conversion](https://en.wikipedia.org/wiki/Geographic_coordinate_conversion)
-  [ESA NAvipedia - Transformations between ECEF and ENU coordinates](https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates)

# License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
