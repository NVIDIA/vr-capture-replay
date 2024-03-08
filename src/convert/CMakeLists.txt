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

cmake_minimum_required( VERSION 3.5 )

set( TARGET_NAME convert )

add_executable( ${TARGET_NAME}
  convert.cpp
  VRData_0.8.h
  ../shared/logging.h
  ../shared/nvprint.hpp
  ../shared/nvprint.cpp
  ../shared/VRData.h )

target_compile_definitions( ${TARGET_NAME} PUBLIC PROJECT_NAME="${TARGET_NAME}" )

get_directory_property( hasParent PARENT_DIRECTORY )
if( hasParent )
  set_target_properties( ${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUT_DIR}/${TARGET_NAME}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUT_DIR}/${TARGET_NAME} )
else()
  project( Sample )
  set_target_properties( ${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR} )
  set_target_properties( ${TARGET_NAME} PROPERTIES DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} VS_STARTUP_PROJECT ${TARGET_NAME} )
endif()

set_target_properties( ${TARGET_NAME} PROPERTIES CXX_STANDARD 17 )

target_include_directories( ${TARGET_NAME} PUBLIC . shared thirdparty ../thirdparty/cereal/include )

target_link_libraries( ${TARGET_NAME} )

