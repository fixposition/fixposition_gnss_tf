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

For the tests:

-  [Googletest](https://github.com/google/googletest)

## Build

### Using [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) (_recommended_):

-  Create a catkin workspace with the structure:

```
catkin_ws
└── src
    └── gnsstransformationlib
    └── <Other Catkin Pacakges>
```

-  in the workspace do:

```bash
catkin build fixposition_gnss_tf
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
