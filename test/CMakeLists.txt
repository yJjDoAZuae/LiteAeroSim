# ---------------------------------------------------------------------------
# LiteAeroSim unit tests
#
# enable_testing() is called at the project root. All external dependencies
# are fetched and made available by the root CMakeLists.txt.
# ---------------------------------------------------------------------------

# Collect test sources: files matching *_test.cpp / *_test.cc in this directory.
# To add a new test file, name it <subject>_test.cpp and it will be picked up
# automatically on the next CMake configure.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc"
)

if(NOT TEST_SOURCES)
    message(STATUS "liteaerosim_test: no test sources found in ${CMAKE_CURRENT_SOURCE_DIR} — skipping test target")
    return()
endif()

add_executable(liteaerosim_test ${TEST_SOURCES})

target_include_directories(liteaerosim_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

target_compile_features(liteaerosim_test PRIVATE cxx_std_17)

# Absolute path to the test data directory, available as LAS_TEST_DATA_DIR in all test files.
target_compile_definitions(liteaerosim_test PRIVATE
    LAS_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/data"
)

# liteaerosim brings in Eigen, JSON, and trochoids transitively (PRIVATE).
# Eigen3::Eigen and nlohmann_json::nlohmann_json are listed explicitly here
# so that test files may include their headers directly (e.g. for test fixtures
# that construct Eigen matrices or JSON objects directly).
# liteaerosim_proto is listed explicitly to resolve proto symbols used inside
# the liteaerosim static library (proto dep is PRIVATE to liteaerosim).
target_link_libraries(liteaerosim_test
    PRIVATE
        # gtest_main must come before liteaerosim (which transitively introduces
        # SDL2 and libmingw32). GNU ld processes archives left to right; if
        # libmingw32 is processed before gtest_main, crtexewin.o wins the
        # 'main' symbol and expects WinMain which is not defined.
        GTest::gtest_main
        GTest::gtest
        liteaerosim
        liteaerosim_proto
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

# SDL2 static library on Windows embeds the GUI subsystem. Force console
# subsystem so the test binary uses main() not WinMain().
if(WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(liteaerosim_test PRIVATE -mconsole -Wl,--subsystem,console)
endif()

# Register individual test cases with CTest via Google Test discovery.
# WORKING_DIRECTORY keeps test-generated output files (e.g. CSVs) inside the
# build tree instead of the source root.
include(GoogleTest)
gtest_discover_tests(liteaerosim_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
