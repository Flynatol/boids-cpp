cmake_minimum_required(VERSION 3.16)

# CMake 4.x removes compatibility with very old policies used by vendored deps.
# Set a minimum policy version once at the top so subprojects still configure.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
	set(CMAKE_POLICY_VERSION_MINIMUM "3.5")
endif()

include(FetchContent)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) #link time optimization

project(cpp_boids)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(BOIDS_ENABLE_TRACY "Enable Tracy profiling" OFF)
option(BOIDS_ENABLE_AVX512 "Build optional AVX-512 simulation fast path" ON)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

set(BUILD_EXAMPLES OFF CACHE BOOL " " FORCE)	# Disable raylib examples
set(BUILD_GAMES OFF CACHE BOOL " " FORCE)		# Disable raylib games
set(USE_EXTERNAL_GLFW OFF CACHE BOOL " " FORCE) # Tell raylib to use external GLFW

add_subdirectory(thirdparty/glad)
add_subdirectory(thirdparty/imgui-docking)
add_subdirectory(thirdparty/glfw)
add_subdirectory(thirdparty/raylib)
add_subdirectory(thirdparty/rlimgui)

FetchContent_Declare(
	tracy
	GIT_REPOSITORY https://github.com/wolfpld/tracy.git
	GIT_TAG v0.13.1
	GIT_SHALLOW TRUE
	GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(tracy)

file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

set(BOIDS_AVX512_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/sim/simulation_avx512.cpp")

add_executable("${CMAKE_PROJECT_NAME}")

set_property(TARGET "${CMAKE_PROJECT_NAME}" PROPERTY CXX_STANDARD 20)

target_compile_definitions("${CMAKE_PROJECT_NAME}" PUBLIC GLFW_INCLUDE_NONE=1)

#target_compile_options("${CMAKE_PROJECT_NAME}" PRIVATE -march=native)
if(MSVC)
	target_compile_options("${CMAKE_PROJECT_NAME}" PRIVATE /arch:AVX2)
endif()

if(BOIDS_ENABLE_AVX512)
	if(MSVC)
		set_source_files_properties("${BOIDS_AVX512_SOURCE}" PROPERTIES COMPILE_OPTIONS "/arch:AVX512")
	else()
		set_source_files_properties("${BOIDS_AVX512_SOURCE}" PROPERTIES COMPILE_OPTIONS "-mavx512f")
	endif()
	set_source_files_properties("${BOIDS_AVX512_SOURCE}" PROPERTIES COMPILE_DEFINITIONS "BOIDS_COMPILE_AVX512=1")
endif()

if(PRODUCTION_BUILD)
	# setup the ASSETS_PATH macro to be in the root folder of your exe
	target_compile_definitions("${CMAKE_PROJECT_NAME}" PUBLIC RESOURCES_PATH="./resources/")

	# remove the option to debug asserts.
	target_compile_definitions("${CMAKE_PROJECT_NAME}" PUBLIC PRODUCTION_BUILD=1)

else()
	# This is useful to get an ASSETS_PATH in your IDE during development
	target_compile_definitions("${CMAKE_PROJECT_NAME}" PUBLIC RESOURCES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/resources/")
	target_compile_definitions("${CMAKE_PROJECT_NAME}" PUBLIC PRODUCTION_BUILD=0)

endif()

target_sources("${CMAKE_PROJECT_NAME}" PRIVATE ${MY_SOURCES}  )

if(MSVC) # If using the VS compiler...

	target_compile_definitions("${CMAKE_PROJECT_NAME}" PUBLIC _CRT_SECURE_NO_WARNINGS)

	#set_target_properties("${CMAKE_PROJECT_NAME}" PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup") #no 
	#set_property(TARGET "${CMAKE_PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreadedDebug<$<CONFIG:Debug>:Debug>")
	#set_property(TARGET "${CMAKE_PROJECT_NAME}" PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")

endif()

#target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories("${CMAKE_PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories("${CMAKE_PROJECT_NAME}" PRIVATE "${tracy_SOURCE_DIR}/public")

target_link_libraries("${CMAKE_PROJECT_NAME}" PRIVATE raylib imgui rlimgui glfw)

if(BOIDS_ENABLE_TRACY)
	target_link_libraries("${CMAKE_PROJECT_NAME}" PRIVATE TracyClient)
endif()
