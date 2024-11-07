#[[
 * SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
]]


set( TARGET_NAME capture )

add_executable( ${TARGET_NAME}
  capture.h
  capture.cpp
  ../shared/logging.h
  ../shared/nvprint.hpp
  ../shared/nvprint.cpp
  ../shared/VRData.h )

target_compile_definitions( ${TARGET_NAME} PUBLIC PROJECT_NAME="${TARGET_NAME}" _CRT_SECURE_NO_WARNINGS )

target_link_libraries( ${TARGET_NAME} ${OPENVR_LIBRARIES} )

set_target_properties( ${TARGET_NAME} PROPERTIES CXX_STANDARD 20 )

set_target_properties( ${TARGET_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUT_DIR}/${TARGET_NAME}
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUT_DIR}/${TARGET_NAME} )

target_compile_options( ${TARGET_NAME} PUBLIC "/constexpr:steps 2048576" )

target_include_directories( ${TARGET_NAME} PUBLIC ../thirdparty/cereal/include ../thirdparty/magic_enum/include )

add_custom_command( TARGET ${TARGET_NAME} POST_BUILD 
  COMMAND ${CMAKE_COMMAND} -E copy ${OPENVR_DIR}/bin/win64/openvr_api.dll ${OUT_DIR}/${TARGET_NAME}/openvr_api.dll )