# GENERAL ========================================================================================================
cmake_minimum_required(VERSION 3.5)
project(fixposition_gnss_tf LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

# DEPENDENCIES =========================================================================================================
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED)

# define catkin package for this project
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# BUILD SHARED LIBRARY =================================================================================================
add_library(${PROJECT_NAME} SHARED
  src/gnss_tf.cpp
)

target_include_directories(${PROJECT_NAME} INTERFACE ${catkin_LIBRARIES})
# link library against dependencies
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

# TEST =================================================================================================================
if (CATKIN_ENABLE_TESTING)
  find_package(yaml-cpp  REQUIRED)

  catkin_add_gtest(gnss_test test/gnss_test.cpp)
  target_compile_definitions(gnss_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test/")
  target_link_libraries(gnss_test ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

  include(GoogleTest)
  gtest_discover_tests(gnss_test)
endif()
  