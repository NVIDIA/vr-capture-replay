/*
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
 */

#pragma once

#ifndef NOMINMAX
#  define NOMINMAX
#endif
#include <windows.h>  // windows.h needs to be included before psapi.h !
/**/
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <openvr.h>
#include <psapi.h>
#include <set>
#include <string>
#include <thread>
#include <vector>

#pragma warning( push )
#pragma warning( disable : 4267 )

#define STRING2( x ) #x
#define STRING( x )  STRING2( x )
// override some defaults for magic_enum because it's normally not built for huge enums
// like vr::ETrackedDeviceProperty - and for those huge numbers we have to also
// override /constexpr:steps in the project's CMakeLists.txt
#if 1  // set to 0 for faster compilation
#  pragma message( __FILE__ ":" STRING( __LINE__ ) ": This will take a while because of a big magic_enum range" )
#  define MAGIC_ENUM_NO_CHECK_FLAGS
#  define MAGIC_ENUM_RANGE_MIN 0
#  define MAGIC_ENUM_RANGE_MAX 7005
#endif
#include "magic_enum/magic_enum.hpp"
#include "quick_arg_parser/quick_arg_parser.hpp"
#pragma warning( pop )

#include "shared/VRData.h"
