cmake_minimum_required(VERSION 3.16)
project(LiteAeroSim LANGUAGES CXX)

# Project-wide defaults
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_SHARED_LIBS "Build libraries as shared instead of static" OFF)

# Build the library in src/ and use include/ for public headers
add_subdirectory(src)

# add test subdirectory (will configure an executable and unit tests)
add_subdirectory(test)

set(EIGEN3_ROOT_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(EIGEN3_ROOT_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${EIGEN3_ROOT_DIR}")
endif()

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# Link Eigen to targets
target_link_libraries(liteaerosim PRIVATE Eigen3::Eigen)

