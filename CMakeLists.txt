#[[
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
]]


cmake_minimum_required(VERSION 3.5)

project(VCR)

set(OPENVR_DIR ${CMAKE_SOURCE_DIR}/src/thirdparty/openvr/)

find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR_DIR}/bin
    ${OPENVR_DIR}/lib
  PATH_SUFFIXES
	win64 Win64 x64
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)

set(OPENVR_INCLUDE_DIR ${OPENVR_DIR}/headers)
set(SHARED_SRC_DIR ${OPENVR_DIR}/samples/shared)

file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)

include_directories(
  ${OPENVR_INCLUDE_DIR}
  ${OPENVR_DIR}/samples/shared
  ${OPENVR_DIR}/samples/drivers/utils
  src/
  src/thirdparty
)

file(READ "${CMAKE_SOURCE_DIR}/src/aux_files/version.txt" VCR_VERSION)
set(OUT_DIR ${CMAKE_SOURCE_DIR}/_out/VCR_${VCR_VERSION})
add_definitions(-DVCR_VERSION="${VCR_VERSION}")
message(STATUS "VCR version ${VCR_VERSION}")

add_subdirectory(src/capture)
add_subdirectory(src/convert)
add_subdirectory(src/helper)
add_subdirectory(src/replay)
add_subdirectory(src/sample)



add_custom_target(VCR ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/aux_files ${OUT_DIR}

  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/thirdparty/licenses ${OUT_DIR}/3rd_party_licenses

  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/sample ${OUT_DIR}/sample
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/shared ${OUT_DIR}/sample/shared
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/thirdparty/cereal ${OUT_DIR}/sample/thirdparty/cereal
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/src/thirdparty/licenses/cereal_license.txt ${OUT_DIR}/sample/thirdparty
  
  COMMAND ${CMAKE_COMMAND} -E make_directory ${OUT_DIR}/logs

  SOURCES ${CMAKE_SOURCE_DIR}/src/aux_files/version.txt
)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT capture)
