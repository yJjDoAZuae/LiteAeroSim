# ---------------------------------------------------------------------------
# liteaero_sim_gdext — Godot 4 GDExtension shared library
#
# Design authority: docs/architecture/godot_plugin.md
#
# Built only when LITEAERO_SIM_BUILD_GODOT_PLUGIN=ON (root CMakeLists.txt).
# Requires the main project's protobuf build to have run first so that
# liteaerosim.pb.cc / liteaerosim.pb.h are present in the build tree.
#
# Source files (register_types.cpp, SimulationReceiver.cpp) are implemented
# as part of roadmap item GP-1.  Enabling this option before GP-1 is complete
# will produce a clear error below.
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(liteaero_sim_gdext CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ---------------------------------------------------------------------------
# Derive godot-cpp git tag from godot/project.godot.
#
# godot/project.godot is written by the Godot editor and is the authoritative
# source for the engine version.  The tag derived here must match the installed
# Godot editor version.
#
# Tag scheme (godot-cpp changed versioning at Godot 4.6):
#   Godot <= 4.5  →  godot-cpp tag: godot-X.Y(.Z)-stable
#   Godot >= 4.6  →  godot-cpp uses independent versioning (10.x).
#                    The tag is specified by GODOT_CPP_TAG_46_PLUS below.
#                    See: https://docs.godotengine.org/en/stable/tutorials/scripting/cpp/about_godot_cpp.html
#
# Failure modes handled with informative errors:
#   1. project.godot not found at expected path.
#   2. No config/features= line present (corrupt file or pre-4.0 format).
#   3. No version token found in the features line.
#   4. Version string contains non-stable identifiers (rc, beta, alpha, dev).
#   5. FetchContent fails (network unavailable or tag absent from godot-cpp).
# ---------------------------------------------------------------------------

# godot-cpp release to use for Godot 4.6+.
# Check https://github.com/godotengine/godot-cpp/releases for the latest tag.
set(GODOT_CPP_TAG_46_PLUS "10.0.0-rc1")

set(_GODOT_PROJECT_FILE "${CMAKE_SOURCE_DIR}/godot/project.godot")

# --- Failure mode 1: project.godot not found ---
if(NOT EXISTS "${_GODOT_PROJECT_FILE}")
    message(FATAL_ERROR
        "\n"
        "  [liteaero_sim_gdext] Cannot locate the Godot project file:\n"
        "    ${_GODOT_PROJECT_FILE}\n"
        "\n"
        "  The Godot 4 project is expected at godot/ relative to the\n"
        "  repository root.  If the project directory has moved, update\n"
        "  _GODOT_PROJECT_FILE in:\n"
        "    godot/addons/liteaero_sim/src/CMakeLists.txt\n")
endif()

file(STRINGS "${_GODOT_PROJECT_FILE}" _godot_features_lines
     REGEX "^config/features=")

# --- Failure mode 2: config/features= line missing ---
if(NOT _godot_features_lines)
    message(FATAL_ERROR
        "\n"
        "  [liteaero_sim_gdext] The line 'config/features=' was not found in:\n"
        "    ${_GODOT_PROJECT_FILE}\n"
        "\n"
        "  This line is written by the Godot editor to record the engine\n"
        "  version.  Possible causes:\n"
        "    - project.godot was hand-edited and the line was removed.\n"
        "    - The file is from a Godot 3 project (incompatible format).\n"
        "  Open the project in a Godot 4 stable release and re-run CMake.\n")
endif()

# Use only the first matching line (there should never be more than one).
list(GET _godot_features_lines 0 _godot_features_line)

# Extract the first quoted token that is a version number.
# Expected form: config/features=PackedStringArray("4.6", "Forward Plus")
string(REGEX MATCH "\"([0-9]+\\.[0-9]+(\\.[0-9]+)?)\"" _version_match
       "${_godot_features_line}")
set(GODOT_VERSION "${CMAKE_MATCH_1}")

# --- Failure mode 3: no version token found ---
if(NOT GODOT_VERSION)
    message(FATAL_ERROR
        "\n"
        "  [liteaero_sim_gdext] Could not parse a version number from:\n"
        "    ${_godot_features_line}\n"
        "\n"
        "  Expected the first quoted entry to be a stable version string of\n"
        "  the form \"X.Y\" or \"X.Y.Z\" (e.g. \"4.5\" or \"4.6\").\n"
        "\n"
        "  The file may have been hand-edited, or written by an unsupported\n"
        "  Godot version.  Open the project in a stable Godot 4 release and\n"
        "  re-run CMake.\n")
endif()

# --- Failure mode 4: non-stable version identifier ---
# The regex above only matches digits-and-dots, so release candidates
# (e.g. "4.6-rc1") will fail the initial match.  This guard is a belt-and-
# suspenders check that the captured string is strictly digits and dots.
if(NOT GODOT_VERSION MATCHES "^[0-9]+(\\.[0-9]+)+$")
    message(FATAL_ERROR
        "\n"
        "  [liteaero_sim_gdext] Parsed version '${GODOT_VERSION}' does not\n"
        "  match the expected stable release format (X.Y or X.Y.Z, digits\n"
        "  only).\n"
        "\n"
        "  Release candidates, betas, alpha builds, and dev snapshots are\n"
        "  not supported by this build configuration.  Switch to a stable\n"
        "  Godot 4 release and re-run CMake.\n")
endif()

# Derive godot-cpp tag.
# Godot 4.6+ uses godot-cpp independent versioning (10.x); earlier releases
# used per-Godot tags (godot-X.Y-stable).
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)" _v_match "${GODOT_VERSION}")
set(_godot_major "${CMAKE_MATCH_1}")
set(_godot_minor "${CMAKE_MATCH_2}")

if(_godot_major GREATER_EQUAL 4 AND _godot_minor GREATER_EQUAL 6)
    set(GODOT_CPP_TAG "${GODOT_CPP_TAG_46_PLUS}")
else()
    set(GODOT_CPP_TAG "godot-${GODOT_VERSION}-stable")
endif()

message(STATUS
    "[liteaero_sim_gdext] Godot version from project.godot: "
    "${GODOT_VERSION}  ->  godot-cpp tag: ${GODOT_CPP_TAG}")

# --- Failure mode 5: FetchContent fails (pre-flight guidance) ---
# CMake cannot validate that the derived tag exists before fetching.
# These STATUS messages appear immediately before the fetch so that a
# "repository not found" or "revision not found" error from git is accompanied
# by actionable context.
message(STATUS
    "[liteaero_sim_gdext] Fetching godot-cpp ${GODOT_CPP_TAG} from GitHub.")
message(STATUS
    "[liteaero_sim_gdext] If the fetch fails:")
message(STATUS
    "  - 'revision not found': tag '${GODOT_CPP_TAG}' does not exist.")
message(STATUS
    "    Verify the Godot editor version at godot/project.godot matches a")
message(STATUS
    "    released godot-cpp tag: https://github.com/godotengine/godot-cpp/tags")
message(STATUS
    "  - 'could not resolve host': no network access during configure.")
message(STATUS
    "    Run with a network connection or pre-populate the FetchContent cache.")

FetchContent_Declare(
    godot-cpp
    GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
    GIT_TAG        "${GODOT_CPP_TAG}"
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(godot-cpp)

# ---------------------------------------------------------------------------
# Source file guard — GP-1 implementation check
#
# The C++ source files are part of roadmap item GP-1 and have not been
# committed yet.  This guard produces a clear error rather than a confusing
# "cannot find source file" message from add_library.
# ---------------------------------------------------------------------------
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/register_types.cpp")
    message(FATAL_ERROR
        "\n"
        "  [liteaero_sim_gdext] GDExtension C++ source files are not present:\n"
        "    ${CMAKE_CURRENT_SOURCE_DIR}/register_types.cpp\n"
        "\n"
        "  These files are implemented as roadmap item GP-1.\n"
        "  See docs/architecture/godot_plugin.md for the full specification.\n"
        "  Until GP-1 is complete, do not set LITEAERO_SIM_BUILD_GODOT_PLUGIN=ON.\n")
endif()

# ---------------------------------------------------------------------------
# protobuf — provided by Conan via find_package (same as rest of project)
# ---------------------------------------------------------------------------
find_package(protobuf REQUIRED CONFIG)

# ---------------------------------------------------------------------------
# Generated proto sources (produced by the proto/ build step in the parent).
# ---------------------------------------------------------------------------
set(_PROTO_SRC "${CMAKE_SOURCE_DIR}/build/proto/liteaerosim.pb.cc")
set(_PROTO_HDR_DIR "${CMAKE_SOURCE_DIR}/build/proto")

if(NOT EXISTS "${_PROTO_SRC}")
    message(FATAL_ERROR
        "\n"
        "  [liteaero_sim_gdext] Generated protobuf source not found:\n"
        "    ${_PROTO_SRC}\n"
        "\n"
        "  Build the main project first to generate liteaerosim.pb.cc:\n"
        "    cmake --build build --target proto\n"
        "  Then re-run cmake with -DLITEAERO_SIM_BUILD_GODOT_PLUGIN=ON.\n")
endif()

# ---------------------------------------------------------------------------
# GDExtension shared library
# ---------------------------------------------------------------------------
add_library(liteaero_sim_gdext SHARED
    register_types.cpp
    SimulationReceiver.cpp
    "${_PROTO_SRC}"
)

target_include_directories(liteaero_sim_gdext PRIVATE
    "${_PROTO_HDR_DIR}"
    "${CMAKE_SOURCE_DIR}/include"
)

target_link_libraries(liteaero_sim_gdext PRIVATE
    godot-cpp
    protobuf::libprotobuf
    $<$<BOOL:${WIN32}>:ws2_32>
)

# On MinGW, copy libwinpthread-1.dll into the bin/ directory so Godot can find
# it at load time.  Godot is a regular Windows application; ucrt64/bin is not
# on its PATH, so the DLL must live beside the plugin or in a system directory.
if(MINGW)
    find_file(WINPTHREAD_DLL "libwinpthread-1.dll"
        HINTS "${CMAKE_CXX_COMPILER_OBJECT_REG_HINT}"
              "C:/msys64/ucrt64/bin"
              "C:/msys64/mingw64/bin"
        NO_DEFAULT_PATH
    )
    if(WINPTHREAD_DLL)
        add_custom_command(TARGET liteaero_sim_gdext POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${WINPTHREAD_DLL}"
                "${CMAKE_SOURCE_DIR}/godot/addons/liteaero_sim/bin/libwinpthread-1.dll"
            COMMENT "[liteaero_sim_gdext] Copying ${WINPTHREAD_DLL} -> bin/"
        )
    else()
        message(WARNING
            "[liteaero_sim_gdext] libwinpthread-1.dll not found — "
            "Godot will fail to load the plugin with Error 126. "
            "Copy libwinpthread-1.dll from ucrt64/bin into "
            "godot/addons/liteaero_sim/bin/ manually.")
    endif()
endif()

# Output the shared library into the bin/ directory declared in liteaero_sim.gdextension.
set_target_properties(liteaero_sim_gdext PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY
        "${CMAKE_SOURCE_DIR}/godot/addons/liteaero_sim/bin"
    LIBRARY_OUTPUT_DIRECTORY
        "${CMAKE_SOURCE_DIR}/godot/addons/liteaero_sim/bin"
)

# On Windows (MinGW), strip the default "lib" prefix so the output is
# liteaero_sim_gdext.dll — matching the path in the .gdextension manifest.
# On Linux and macOS the "lib" prefix is conventional for .so/.dylib and is kept
# (manifest entries for those platforms use the lib-prefixed name).
if(WIN32)
    set_target_properties(liteaero_sim_gdext PROPERTIES PREFIX "")
endif()
