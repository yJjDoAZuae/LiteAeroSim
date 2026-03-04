# ---------------------------------------------------------------------------
# liteaerosim_proto — generated C++ from proto/liteaerosim.proto
#
# Protobuf is fetched (or found on the system) by the root CMakeLists.txt
# before this subdirectory is processed.
# ---------------------------------------------------------------------------

# Protobuf is fetched/found by the root CMakeLists.txt — do not call
# find_package again here. protobuf_generate_cpp() and protobuf::libprotobuf
# are already available.

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/liteaerosim.proto
)

add_library(liteaerosim_proto STATIC ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(liteaerosim_proto PUBLIC protobuf::libprotobuf)

# Generated headers live in the build tree; expose them to consumers.
target_include_directories(liteaerosim_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_compile_features(liteaerosim_proto PUBLIC cxx_std_17)

# Suppress warnings from generated code — we do not own it.
target_compile_options(liteaerosim_proto PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-w>
    $<$<CXX_COMPILER_ID:MSVC>:/w>
)
