#[[
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
]]


set( TARGET_NAME capture )

add_executable( ${TARGET_NAME}
  capture.h
  capture.cpp
  ../shared/logging.h
  ../shared/nvprint.hpp
  ../shared/nvprint.cpp
  ../shared/VRData.h )

target_compile_definitions( ${TARGET_NAME} PUBLIC PROJECT_NAME="${TARGET_NAME}" )

target_link_libraries( ${TARGET_NAME} ${OPENVR_LIBRARIES} )

set_target_properties( ${TARGET_NAME} PROPERTIES CXX_STANDARD 20 )

set_target_properties( ${TARGET_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUT_DIR}/${TARGET_NAME}
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUT_DIR}/${TARGET_NAME} )

target_compile_options( ${TARGET_NAME} PUBLIC "/constexpr:steps 2048576" )

target_include_directories( ${TARGET_NAME} PUBLIC ../thirdparty/cereal/include )

add_custom_command( TARGET ${TARGET_NAME} POST_BUILD 
  COMMAND ${CMAKE_COMMAND} -E copy ${OPENVR_DIR}/bin/win64/openvr_api.dll ${OUT_DIR}/${TARGET_NAME}/openvr_api.dll )