# Global vars ----------------------------------------------------------------------------------------------------------
set(GTEST_PREFIX "gtest")
set(BUILD_TEST_TARGET "build_test")

enable_testing()

if(NOT CATKIN_DEVEL_PREFIX) # not using ROS1
    # Deps -------------------------------------------------------------------------------------------------------------
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    # CMake modules ----------------------------------------------------------------------------------------------------
    include(GoogleTest)

    add_custom_target(${BUILD_TEST_TARGET})
endif()

# Macros ---------------------------------------------------------------------------------------------------------------
macro(add_gtest)
    # Arg parse
    set(options)
    set(one_value_args TARGET WORKING_DIRECTORY)
    set(multi_value_args SOURCES INCLUDE_DIRS LINK_DIRS LINK_LIBS)
    cmake_parse_arguments(ADD_GTEST "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(CATKIN_DEVEL_PREFIX AND CATKIN_ENABLE_TESTING) # using ROS1 with catkin
        # Call catkin macro instead
        catkin_add_gtest(
            ${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            ${ADD_GTEST_SOURCES}
            WORKING_DIRECTORY ${ADD_GTEST_WORKING_DIRECTORY}
        )
        target_link_libraries(${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            ${catkin_LIBRARIES}
            ${ADD_GTEST_LINK_LIBS}
        )
    elseif((DEFINED ENV{COLCON}) AND BUILD_TESTING) # ROS2 / build with colcon
        # Call ament_add_gtest macro
        find_package(ament_cmake_gtest REQUIRED)

        ament_add_gtest(
            ${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            ${ADD_GTEST_SOURCES}
            WORKING_DIRECTORY ${ADD_GTEST_WORKING_DIRECTORY})

        target_link_libraries(
            ${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            ${ADD_GTEST_LINK_LIBS}
        )
    elseif(NOT CATKIN_DEVEL_PREFIX AND NOT(DEFINED ENV{COLCON})) # plain CMake
        # Make targets
        add_executable(${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            ${ADD_GTEST_SOURCES}
        )

        target_include_directories(${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            PRIVATE ${ADD_GTEST_INCLUDE_DIRS}
        )
        target_link_libraries(${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            PRIVATE
            ${GTEST_BOTH_LIBRARIES}
            ${CMAKE_THREAD_LIBS_INIT}
            ${ADD_GTEST_LINK_LIBS}
        )

        # Discover tests
        gtest_add_tests(
            TARGET ${GTEST_PREFIX}_${ADD_GTEST_TARGET}
            SOURCES ${ADD_GTEST_SOURCES}
            WORKING_DIRECTORY ${ADD_GTEST_WORKING_DIRECTORY}
        )

        # Add to global custom target
        add_dependencies(${BUILD_TEST_TARGET} ${GTEST_PREFIX}_${ADD_GTEST_TARGET})
    endif()
endmacro()
