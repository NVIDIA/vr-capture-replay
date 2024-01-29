#[[
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
]]


set(TARGET_NAME helper)
add_compile_definitions(PROJECT_NAME="${TARGET_NAME}")

add_executable(${TARGET_NAME}
  helper.cpp
  ../shared/logging.h
  ../shared/nvprint.hpp
  ../shared/nvprint.cpp
)

target_link_libraries(${TARGET_NAME}
  ${OPENVR_LIBRARIES}
)

set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 17)

set_target_properties( ${TARGET_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUT_DIR}/${TARGET_NAME}
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUT_DIR}/${TARGET_NAME}
)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD 
  COMMAND ${CMAKE_COMMAND} -E copy ${OPENVR_DIR}/bin/win64/openvr_api.dll ${OUT_DIR}/${TARGET_NAME}/openvr_api.dll
)