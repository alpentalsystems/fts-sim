cmake_minimum_required(VERSION 3.16)
project(fts_bridge C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_library(bridge_core STATIC src/mavlink_hb.cpp src/gate.cpp src/convert.cpp ${REPO}/common/proto.c)
target_include_directories(bridge_core PUBLIC src ${REPO}/common/include)

enable_testing()
foreach(name mavlink_hb gate convert)
	add_executable(test_${name} tests/test_${name}.cpp)
	target_include_directories(test_${name} PRIVATE ${REPO}/tests/host)
	target_link_libraries(test_${name} PRIVATE bridge_core)
	add_test(NAME ${name} COMMAND test_${name})
endforeach()

option(FTS_BRIDGE_GZ "Build the Gazebo bridge executable" ON)
if(FTS_BRIDGE_GZ)
	find_package(Threads REQUIRED)
	find_package(gz-transport13 REQUIRED)
	find_package(gz-msgs10 REQUIRED)
	add_executable(fts_bridge src/main.cpp src/pty.cpp)
	target_link_libraries(fts_bridge PRIVATE bridge_core Threads::Threads
		gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
endif()
