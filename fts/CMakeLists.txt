cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fts_firmware)

target_include_directories(app PRIVATE src/logic ../common/include)
target_sources(app PRIVATE
	src/main.c
	src/logic/app.c
	src/logic/fts.c
	src/logic/geo.c
	src/logic/pbit.c
	../common/proto.c
)
