cmake_minimum_required(VERSION 3.2)
project(libicsneo-socketcan-daemon VERSION 3.1.1)

set(CMAKE_CXX_STANDARD 17)

include(GNUInstallDirs)

# Enable Warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-switch -Wno-nested-anon-types -Wno-gnu-anonymous-struct -Wno-unknown-pragmas -Wno-zero-length-array")

# Generate build info header
execute_process(
	COMMAND git rev-parse --abbrev-ref HEAD
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_BRANCH
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
	COMMAND git describe --abbrev=6 --dirty --always --tags
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_DESCRIBE
	ERROR_VARIABLE GIT_DESCRIBE
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
configure_file(src/buildinfo.h.template ${CMAKE_CURRENT_BINARY_DIR}/generated/buildinfo.h)
include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory("third-party/libicsneo")

add_executable(libicsneo-socketcan-daemon src/main.cpp)
target_link_libraries(libicsneo-socketcan-daemon icsneocpp)