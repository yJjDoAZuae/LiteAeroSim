# ---------------------------------------------------------------------------
# LiteAeroSim standalone tools
#
# Each tool is a separate executable that links liteaerosim.
# Tools are not part of the library and are excluded from the lib glob.
# ---------------------------------------------------------------------------

# joystick_verify: streams ManualInputFrame to stdout for notebook display.
# Requires SDL2 (transitively via liteaerosim).
add_executable(joystick_verify
    ${CMAKE_CURRENT_SOURCE_DIR}/joystick_verify.cpp
)
target_link_libraries(joystick_verify
    PRIVATE
        liteaerosim
        liteaerosim_proto
        SDL2::SDL2
)
target_compile_features(joystick_verify PRIVATE cxx_std_17)

# Force console subsystem so joystick_verify can write to stdout.
if(WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(joystick_verify PRIVATE -mconsole)
endif()
