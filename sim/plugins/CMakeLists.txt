cmake_minimum_required(VERSION 3.16)
project(fts_sim_plugins CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(FtsParachute SHARED parachute.cpp)
target_link_libraries(FtsParachute PRIVATE
	gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)

enable_testing()
add_executable(test_parachute test_parachute.cpp)
target_include_directories(test_parachute PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/host)
target_compile_definitions(test_parachute PRIVATE TEST_WORLD="${CMAKE_CURRENT_SOURCE_DIR}/test_drop.sdf")
target_link_libraries(test_parachute PRIVATE
	gz-sim8::gz-sim8 gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
add_test(NAME parachute COMMAND test_parachute)
set_tests_properties(parachute PROPERTIES
	ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR};GZ_IP=127.0.0.1")
