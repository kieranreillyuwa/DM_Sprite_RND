#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

zephyr_include_directories(
  kernel/include
  ${ARCH_DIR}/${ARCH}/include
)


zephyr_include_directories(includes)
zephyr_library_sources(src/main.c)

set_shared(IMAGE ${IMAGE_NAME} APPEND PROPERTY BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/${CONFIG_ARM_ENTRY_VENEERS_LIB_NAME})