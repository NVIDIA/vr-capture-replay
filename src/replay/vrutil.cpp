/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */


#include "vrutil.h"

#include <openvr.h>

#include "Windows.h"
#include "Dxgi1_4.h"

#include <string>

int setChaperone(const VRData::Chaperone& c)
{
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::VR_Init(&eError, vr::VRApplication_Utility);

    if (eError == vr::VRInitError_None)
    {
        vr::IVRChaperoneSetup* setup = vr::VRChaperoneSetup();

        setup->RoomSetupStarting();

        if (vr::ETrackingUniverseOrigin(c.m_universeOrigin) == vr::ETrackingUniverseOrigin::TrackingUniverseSeated)
        {
            setup->SetWorkingSeatedZeroPoseToRawTrackingPose((vr::HmdMatrix34_t*)&c.m_origin);
        }
        else
        {
            setup->SetWorkingStandingZeroPoseToRawTrackingPose((vr::HmdMatrix34_t*)&c.m_origin);
        }
        float w = c.m_playArea[0];
        float d = c.m_playArea[1];

        setup->SetWorkingPlayAreaSize(w, d);

        w /= 2;
        d /= 2;

        vr::HmdVector2_t p[] = { {-w, -d}, {w, -d}, {w, d}, {-w, d} };
        setup->SetWorkingPerimeter(p, 4);

        setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);

        setup->ShowWorkingSetPreview();

        vr::VR_Shutdown();
    }
    return eError;

}

GPUResult hasNVGPU(std::string& description)
{
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem* vrSystem = vr::VR_Init(&eError, vr::VRApplication_Utility);

    if (eError == vr::VRInitError_None)
    {
        // check for NV GPU
        int32_t nAdapterIndex = 0;
        vrSystem->GetDXGIOutputInfo(&nAdapterIndex);

        UINT nDXGIFactoryFlags = 0;
        IDXGIFactory4* pFactory;
        CreateDXGIFactory2(nDXGIFactoryFlags, IID_PPV_ARGS(&pFactory));

        IDXGIAdapter1* pAdapter;
        if (FAILED(pFactory->EnumAdapters1(nAdapterIndex, &pAdapter)))
        {
            return GPUResult::ENUMFAILED;
        }
        DXGI_ADAPTER_DESC1 adapterDesc;
        pAdapter->GetDesc1(&adapterDesc);
        
        std::wstring ws(adapterDesc.Description);
#pragma warning( push )
#pragma warning( disable : 4244 )
        std::string s(ws.begin(), ws.end());
#pragma warning(pop)
        description = s;

        if (adapterDesc.VendorId == 0x10DE)
        {
            return NV_GPU;
        }
        else
        {
            return NO_NV_GPU;
        }
    }
    else
    {
        return INITFAILED;
    }
}


