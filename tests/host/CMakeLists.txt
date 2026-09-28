cmake_minimum_required(VERSION 3.20)
project(fts_host_tests C)

enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(test_smoke test_smoke.c)
add_test(NAME smoke COMMAND test_smoke)

add_executable(test_proto test_proto.c ${REPO}/common/proto.c)
target_include_directories(test_proto PRIVATE ${REPO}/common/include)
add_test(NAME proto COMMAND test_proto)

add_executable(test_geo test_geo.c ${REPO}/fts/src/logic/geo.c)
target_include_directories(test_geo PRIVATE ${REPO}/fts/src/logic)
target_link_libraries(test_geo m)
add_test(NAME geo COMMAND test_geo)

add_executable(test_fts test_fts.c ${REPO}/fts/src/logic/fts.c ${REPO}/fts/src/logic/geo.c)
target_include_directories(test_fts PRIVATE ${REPO}/fts/src/logic ${REPO}/common/include)
target_link_libraries(test_fts m)
add_test(NAME fts COMMAND test_fts)

add_executable(test_pbit test_pbit.c ${REPO}/fts/src/logic/pbit.c)
target_include_directories(test_pbit PRIVATE ${REPO}/fts/src/logic)
target_link_libraries(test_pbit m)
add_test(NAME pbit COMMAND test_pbit)
