# GENERAL ========================================================================================================
cmake_minimum_required(VERSION 3.5)
project(fixposition_gnss_tf VERSION 3.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

# DEPENDENCIES =========================================================================================================
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

# BUILD SHARED LIBRARY =================================================================================================
add_library(${PROJECT_NAME} SHARED
  src/gnss_tf.cpp
)

# link library against dependencies
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})

list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

# INSTALL ==============================================================================================================
# define ament package for this project
install(
  DIRECTORY include
  DESTINATION .
)

# Mark executables and/or libraries for installation
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
# See doc: https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html#example-generating-package-files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

# TEST =================================================================================================================
if(BUILD_TESTING)
  include(${PROJECT_SOURCE_DIR}/cmake/testing.cmake)
  find_package(yaml-cpp REQUIRED)

  add_gtest(
    TARGET gnss_test
    SOURCES test/gnss_test.cpp
    LINK_LIBS ${PROJECT_NAME} yaml-cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/test
  )
endif()
