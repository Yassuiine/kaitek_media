#
# Copyright (c) 2025 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

if(NOT SYSBUILD)
  message(FATAL_ERROR
    " This is a multi-image application that should be built using sysbuild.\n"
    " Add --sysbuild argument to west build command to prepare all the images."
  )
endif()

project(BM20_C)

# Windows Device Guard workaround:
# Build static libraries with "ar qcs" and skip the separate ranlib step.
# This avoids calling arm-zephyr-eabi-ranlib.exe when it is policy-blocked.
if(WIN32)
  set(CMAKE_C_ARCHIVE_CREATE
      "<CMAKE_AR> qcs <TARGET> <LINK_FLAGS> <OBJECTS>"
      CACHE STRING "Create C static archive without separate ranlib" FORCE)
  set(CMAKE_CXX_ARCHIVE_CREATE
      "<CMAKE_AR> qcs <TARGET> <LINK_FLAGS> <OBJECTS>"
      CACHE STRING "Create CXX static archive without separate ranlib" FORCE)
  set(CMAKE_ASM_ARCHIVE_CREATE
      "<CMAKE_AR> qcs <TARGET> <LINK_FLAGS> <OBJECTS>"
      CACHE STRING "Create ASM static archive without separate ranlib" FORCE)

  set(CMAKE_C_ARCHIVE_FINISH "" CACHE STRING "Skip C ranlib step" FORCE)
  set(CMAKE_CXX_ARCHIVE_FINISH "" CACHE STRING "Skip CXX ranlib step" FORCE)
  set(CMAKE_ASM_ARCHIVE_FINISH "" CACHE STRING "Skip ASM ranlib step" FORCE)
endif()

target_sources(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ble_receiver.c
)
