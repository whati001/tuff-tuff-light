# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tuff-tuff-light-central)

target_sources(app PRIVATE src/main.c)


# Custom files and folders

target_sources(app PRIVATE
    src/trailer_listener/trailer_listener.c
    src/ttl_central/ttl_central.c
)

zephyr_library_include_directories(src/trailer_listener)
zephyr_library_include_directories(src/ttl_central)