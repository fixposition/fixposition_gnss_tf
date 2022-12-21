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
find_package(ament_cmake REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

# BUILD SHARED LIBRARY =================================================================================================
add_library(${PROJECT_NAME} SHARED
  src/gnss_tf.cpp
)

# link library against dependencies
target_link_libraries(${PROJECT_NAME}  ${EIGEN3_LIBRARIES})

ament_export_dependencies()
# define ament package for this project
ament_package()

# TEST =================================================================================================================
if (BUILD_TESTING)
  find_package(yaml-cpp  REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(gnss_test test/gnss_test.cpp)
  target_compile_definitions(gnss_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test/")
  target_link_libraries(gnss_test ${PROJECT_NAME}  yaml-cpp)

  include(GoogleTest)
  gtest_discover_tests(gnss_test)
endif()
  
