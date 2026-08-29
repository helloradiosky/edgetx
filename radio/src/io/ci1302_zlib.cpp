/*
 * Minimal zlib inflate for CI1302 ISP agent (from stb_image).
 * Only compiled into V14/V14LCD voice-sensor firmware builds.
 */
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#define STBI_NO_THREAD_LOCALS
#include "stb/stb_image.h"
