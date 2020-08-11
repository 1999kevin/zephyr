# Find Zephyr. This also loads Zephyr's build system.
cmake_minimum_required(VERSION 3.13.1)

SET(BOARD_ROOT ${BOARD_ROOT} /Users/liuke/Desktop/internship/zephyr/app/)

find_package(Zephyr)
project(my_zephyr_app)

# Add your source file to the "app" target. This must come after
# find_package(Zephyr) which defines the target.
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/BT.c)

# message(INFO "linker is ${LINKER_SCRIPT}" )