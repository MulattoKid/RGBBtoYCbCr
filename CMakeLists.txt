cmake_minimum_required(VERSION 3.7)
project(RGBtoYCbCr)

# IMPORTANT directories
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/)
set(OUTPUT_DIR ${BUILD_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUTPUT_DIR})

# Source files
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/)
set(RGBtoYCbCr_SRC_FILES ${SRC_DIR}/RGBtoYCbCr.c ${SRC_DIR}/RGBtoYCbCr.h)

# Create targets
add_executable(RGBtoYCbCr ${RGBtoYCbCr_SRC_FILES})

# Set target include directories
target_include_directories(RGBtoYCbCr PRIVATE ${SRC_DIR}/external)

# Set C version
set_property(TARGET RGBtoYCbCr PROPERTY C_STANDARD 99)

# Set target libraries
target_link_libraries(RGBtoYCbCr PRIVATE m)

# Set COMPILATION FLAGS and LINKING
if(MSVC)
	# https://docs.microsoft.com/en-us/cpp/build/reference/compiler-options-listed-by-category?view=vs-2019
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Od /Zi -W4 /MP")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /O3 /Zi -W4 /MP")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O3 -W4 /MP")
	set_property(TARGET RGBtoYCbCr PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${OUTPUT_DIR})
else()
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g -Wall -march=native")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -O3 -g -Wall -march=native")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -Wall -march=native")
endif()
