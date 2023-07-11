# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Specify board
set(BOARD lora_e5_dev_board)
# set(BOARD b_l072z_lrwan1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Define your application's CMake project
project(app LANGUAGES C VERSION 1.0.0)

target_sources(app PRIVATE src/main.c)


