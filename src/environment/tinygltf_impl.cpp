// Single translation unit that instantiates the tinygltf implementation.
// All other files that include tiny_gltf.h must NOT define TINYGLTF_IMPLEMENTATION.
#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_EXTERNAL_IMAGE
#include "tiny_gltf.h"
