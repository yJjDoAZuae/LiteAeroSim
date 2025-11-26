cmake_minimum_required(VERSION 3.16)
project(liteaerosim-src LANGUAGES CXX)

# Collect source files in src/ (recurses)
file(GLOB_RECURSE LITEAEROSIM_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.m"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.mm"
)

set(EIGEN3_ROOT_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if(EIGEN3_ROOT_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${EIGEN3_ROOT_DIR}")
endif()

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

include(FetchContent)
find_package(nlohmann_json QUIET)

# https://github.com/nlohmann/json/tree/v3.12.0?tab=readme-ov-file#cmake
# https://json.nlohmann.me/integration/cmake/#fetchcontent
# https://deepwiki.com/nlohmann/json/5.3-cmake-integration
if(NOT json_FOUND)
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    )

    FetchContent_MakeAvailable(nlohmann_json)
endif()

# Library target (honors BUILD_SHARED_LIBS)
add_library(liteaerosim ${LITEAEROSIM_SOURCES})

# Public headers live in the top-level include/ directory
target_include_directories(liteaerosim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link Eigen to targets
target_link_libraries(liteaerosim PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json)

target_compile_features(liteaerosim PUBLIC cxx_std_17)
set_target_properties(liteaerosim PROPERTIES POSITION_INDEPENDENT_CODE ${CMAKE_POSITION_INDEPENDENT_CODE})

# # Optional: install rules for packaging
# install(TARGETS liteaerosim
#     EXPORT liteaerosimTargets
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

# install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/" DESTINATION include)

# install(EXPORT liteaerosimTargets FILE liteaerosimTargets.cmake NAMESPACE liteaerosim:: DESTINATION lib/cmake/liteaerosim)
