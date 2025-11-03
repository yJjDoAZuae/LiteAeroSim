cmake_minimum_required(VERSION 3.16)

enable_testing()

# Require Eigen for tests that may use it
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# Prefer an installed GTest, otherwise fetch it
include(FetchContent)
find_package(GTest QUIET)

if(NOT GTest_FOUND)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    )

    # Prevent gtest from using install() in its CMake
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# Collect test sources in this folder (add new test files here)
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp"
)

if(TEST_SOURCES)
    add_executable(liteaerosim_test ${TEST_SOURCES})

    # Public headers are in top-level include/ (library already sets this, but keep explicit include for convenience)
    target_include_directories(liteaerosim_test PRIVATE $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)

    # Use C++17 for tests
    target_compile_features(liteaerosim_test PUBLIC cxx_std_17)

    # Link to core library and test framework
    target_link_libraries(liteaerosim_test
        PRIVATE
            liteaerosim
            GTest::gtest_main
            Eigen3::Eigen
    )

    # Register with CTest and enable test discovery
    include(GoogleTest)
    gtest_discover_tests(liteaerosim_test)
else()
    message(STATUS "No test sources found in ${CMAKE_CURRENT_SOURCE_DIR}. Add *.cpp files (e.g. my_test.cpp).")
endif()
