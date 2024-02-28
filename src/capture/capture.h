/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
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

void initVR();

void initChaperone();

void printDeviceInfo( const vr::TrackedDeviceIndex_t deviceId );

void collectDeviceProperties( const vr::TrackedDeviceIndex_t deviceId, VRData::Device & device );

vr::EVRInputError getActionHandle( VRData::Action action, vr::VRActionHandle_t & a );

void setupActions( std::string modelNumber, VRData::Hand hand );

void printConfiguration();

// optional action to start recording
std::string setupStartAction();

// optional action to segment the recording
void setupSegmentationAction();

void waitForStartAction();

void writeHardwareData();

void writeTrackingData();

std::vector<VRData::DevicePose> getDevicePoses();

void initOverlay();

void notifyHMD( std::string message, uint32_t overrideTime = 0 );