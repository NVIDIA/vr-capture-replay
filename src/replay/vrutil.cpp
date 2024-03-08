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

#include "vrutil.h"

#include "Dxgi1_4.h"
#include "Windows.h"

#include <openvr.h>
#include <string>

int setChaperone( const VRData::Chaperone & c )
{
  vr::EVRInitError eError = vr::VRInitError_None;
  vr::VR_Init( &eError, vr::VRApplication_Utility );

  if ( eError == vr::VRInitError_None )
  {
    vr::IVRChaperoneSetup * setup = vr::VRChaperoneSetup();

    setup->RoomSetupStarting();

    if ( vr::ETrackingUniverseOrigin( c.m_universeOrigin ) == vr::ETrackingUniverseOrigin::TrackingUniverseSeated )
    {
      setup->SetWorkingSeatedZeroPoseToRawTrackingPose( (vr::HmdMatrix34_t *)&c.m_origin );
    }
    else
    {
      setup->SetWorkingStandingZeroPoseToRawTrackingPose( (vr::HmdMatrix34_t *)&c.m_origin );
    }
    float w = c.m_playArea[0];
    float d = c.m_playArea[1];

    setup->SetWorkingPlayAreaSize( w, d );

    w /= 2;
    d /= 2;

    vr::HmdVector2_t p[] = { { -w, -d }, { w, -d }, { w, d }, { -w, d } };
    setup->SetWorkingPerimeter( p, 4 );

    setup->CommitWorkingCopy( vr::EChaperoneConfigFile_Live );

    setup->ShowWorkingSetPreview();

    vr::VR_Shutdown();
  }
  return eError;
}

GPUResult hasGPU( std::string & description )
{
  vr::EVRInitError eError   = vr::VRInitError_None;
  vr::IVRSystem *  vrSystem = vr::VR_Init( &eError, vr::VRApplication_Utility );

  if ( eError == vr::VRInitError_None )
  {
    // check for NV GPU
    int32_t nAdapterIndex = 0;
    vrSystem->GetDXGIOutputInfo( &nAdapterIndex );

    UINT            nDXGIFactoryFlags = 0;
    IDXGIFactory4 * pFactory;
    CreateDXGIFactory2( nDXGIFactoryFlags, IID_PPV_ARGS( &pFactory ) );

    IDXGIAdapter1 * pAdapter;
    if ( FAILED( pFactory->EnumAdapters1( nAdapterIndex, &pAdapter ) ) )
    {
      return GPUResult::enumFailed;
    }
    DXGI_ADAPTER_DESC1 adapterDesc;
    pAdapter->GetDesc1( &adapterDesc );
    pAdapter->Release();

    std::wstring ws( adapterDesc.Description );
#pragma warning( push )
#pragma warning( disable : 4244 )
    std::string s( ws.begin(), ws.end() );
#pragma warning( pop )
    description = s;

    // a GPU is considered to be no software adapter
    return ( adapterDesc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE ) ? GPUResult::noGPU : GPUResult::foundGPU;
  }
  else
  {
    return GPUResult::initFailed;
  }
}
