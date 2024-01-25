/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */


#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "OpenVR_HMDDriver.h"
#include "log.h"

#include <algorithm>
#include <chrono>

OpenVR_HMDDriver::OpenVR_HMDDriver()
{
    LOG("OpenVR_HMDDriver::OpenVR_HMDDriver()\n");

    InitializeCriticalSectionAndSpinCount(&m_texSetCS, 0x442);
}

OpenVR_HMDDriver::~OpenVR_HMDDriver()
{
    LOG("OpenVR_HMDDriver::~OpenVR_HMDDriver()\n");

    if (m_runVSyncThread)
    {
        m_runVSyncThread = false;
        m_vSyncThread.join();
    }

    DeleteCriticalSection(&m_texSetCS);
}

void OpenVR_HMDDriver::setDeviceData(VRData::HMD hmd, bool generateVSync, VSyncStyle style)
{
    m_nRenderWidth = hmd.m_renderWidth;
    m_nRenderHeight = hmd.m_renderHeight;

    m_sSerialNumber = hmd.m_device.m_stringProperties[vr::Prop_SerialNumber_String].m_value;
    m_sModelNumber = hmd.m_device.m_stringProperties[vr::Prop_ModelNumber_String].m_value;
    m_flSecondsFromVsyncToPhotons = hmd.m_device.m_floatProperties[vr::Prop_SecondsFromVsyncToPhotons_Float].m_value;
    m_flDisplayFrequency = hmd.m_device.m_floatProperties[vr::Prop_DisplayFrequency_Float].m_value;
    m_flIPD = hmd.m_device.m_floatProperties[vr::Prop_UserIpdMeters_Float].m_value;

    LOG("HMD Serial Number: %s\n", m_sSerialNumber.c_str());
    LOG("HMD Model Number: %s\n", m_sModelNumber.c_str());

    m_hmdData = hmd;

    m_generateVSync = generateVSync;
    m_vsyncStyle = style;
}

void OpenVR_HMDDriver::Update(const vr::DriverPose_t& pose)
{
    // TODO: do we need a mutex here?
    m_pose = pose;
    if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
    {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_pose, sizeof(vr::DriverPose_t));
    }
}

vr::EVRInitError OpenVR_HMDDriver::Activate(vr::TrackedDeviceIndex_t unObjectId)
{
    LOG("OpenVR_HMDDriver::Activate(%i)\n", unObjectId);

    if (!m_renderHelper.Init())
    {
        LOG("OpenVR_HMDDriver: ERROR: Initialization failed, D3D renderer failed.\n");
        return vr::VRInitError_Driver_Failed;
    }

    m_unObjectId = unObjectId;
    m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);
    LOG("HMD Activate %i, %i\n", m_unObjectId, m_ulPropertyContainer);

    vr::VRSettings()->SetInt32(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_Style_Int32, vr::COLLISION_BOUNDS_STYLE_NONE);
    vr::VRSettings()->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_GroundPerimeterOn_Bool, false);

    LOG("HMD Serial Number: %s\n", m_sSerialNumber.c_str());
    LOG("HMD Model Number: %s\n", m_sModelNumber.c_str());
    LOG("HMD Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight);
    LOG("HMD Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons);
    LOG("HMD Display Frequency: %f\n", m_flDisplayFrequency);
    LOG("HMD IPD: %f\n", m_flIPD);

    auto* vrp = vr::VRProperties();

    for (const auto& p : m_hmdData.m_device.m_stringProperties)
    {
        vrp->SetStringProperty(m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value.c_str());
    }
    for (const auto& p : m_hmdData.m_device.m_boolProperties)
    {
        vrp->SetBoolProperty(m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value);
    }
    for (const auto& p : m_hmdData.m_device.m_int32Properties)
    {
        vrp->SetInt32Property(m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value);
    }
    for (const auto& p : m_hmdData.m_device.m_uint64Properties)
    {
        vrp->SetUint64Property(m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value);
    }
    for (const auto& p : m_hmdData.m_device.m_floatProperties)
    {
        vrp->SetFloatProperty(m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value);
    }


    // TODO: maybe need to override some?
    /*
    vrp->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);
    vrp->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
    vrp->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
    vrp->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, true);
    vrp->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceIsWireless_Bool, true);
    */

    if (m_generateVSync)
    {
        vrp->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DriverDirectModeSendsVsyncEvents_Bool, true);
        m_runVSyncThread = true;
        m_vSyncThread = std::thread(&OpenVR_HMDDriver::VSyncThread, this);
    }
    else
    {
        vrp->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DriverDirectModeSendsVsyncEvents_Bool, false);
        m_runVSyncThread = false;
    }

    // set proximity senser to always on, always head present
    auto input = vr::VRDriverInput();
    vr::PropertyContainerHandle_t container = getPropertyContainerHandle();
    vr::VRInputComponentHandle_t prox;
    input->CreateBooleanComponent(container, "/proximity", &prox);
    input->UpdateBooleanComponent(prox, true, 0.0);

    m_isActivated = true;
    return vr::VRInitError_None;
}

void OpenVR_HMDDriver::Deactivate()
{
    m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    if (m_runVSyncThread)
    {
        m_runVSyncThread = false;
        m_vSyncThread.join();
    }
}

void OpenVR_HMDDriver::EnterStandby()
{
}

void* OpenVR_HMDDriver::GetComponent(const char* pchComponentNameAndVersion)
{
    LOG("GetComponent(%s)\n", pchComponentNameAndVersion);

    if (!_stricmp(pchComponentNameAndVersion, vr::IVRDriverDirectModeComponent_Version))
    {
        LOG("  returning IVRDriverDirectModeComponent interface\n");
        return (IVRDriverDirectModeComponent*)this;
    }
    else if (!_stricmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version))
    {
        LOG("  returning IVRDisplayComponent interface\n");
        return (vr::IVRDisplayComponent*)this;
    }

    return NULL;
}


void OpenVR_HMDDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t OpenVR_HMDDriver::GetPose()
{
    // TODO: update m_pose.poseTimeOffset ?
    return m_pose;
}

void OpenVR_HMDDriver::CreateSwapTextureSet(uint32_t unPid, const SwapTextureSetDesc_t* pSwapTextureSetDesc, SwapTextureSet_t* pOutSwapTextureSet)
{
    TextureSet* texSet = new TextureSet;
    int height = pSwapTextureSetDesc->nHeight;
    int width = pSwapTextureSetDesc->nWidth;

    LOG("CreateSwapTextureSet(pid=%u, res=%d x %d)\n", unPid, width, height);

    if (height & 1)
    {
        height++;
    }

    if (width & 1)
    {
        width++;
    }

    for (int i = 0; i < TextureSet::NUM_TEX; i++)
    {
        ID3D11Texture2D* t = m_renderHelper.CreateTexture(width, height, D3D11_RESOURCE_MISC_SHARED, (DXGI_FORMAT)pSwapTextureSetDesc->nFormat);

        IDXGIResource* pDXGIResource = NULL;
        t->QueryInterface(__uuidof(IDXGIResource), (LPVOID*)&pDXGIResource);

        HANDLE h;
        pDXGIResource->GetSharedHandle(&h);
        pDXGIResource->Release();

        if (!h)
        {
            LOG("Failed to retrieve the shared texture handle.\n");
            return;
        }

        pOutSwapTextureSet->rSharedTextureHandles[i] = (vr::SharedTextureHandle_t)h;

        texSet->m_tex[i] = t;
        texSet->m_sharedHandles[i] = h;
    }
    texSet->m_textureFormat = (DXGI_FORMAT)pSwapTextureSetDesc->nFormat;
    texSet->m_srcIdx = 0;
    texSet->m_nextIdx = 0;
    texSet->m_width = width;
    texSet->m_height = height;
    texSet->m_pid = unPid;

    EnterCriticalSection(&m_texSetCS);
    m_texSets.push_back(texSet);
    LeaveCriticalSection(&m_texSetCS);

    LOG("CreateSwapTextureSet done\n");
}

void OpenVR_HMDDriver::DestroySwapTextureSet(vr::SharedTextureHandle_t sharedTextureHandle)
{
    LOG("DestroySwapTextureSet\n");

    EnterCriticalSection(&m_texSetCS);
    for (size_t i = 0; i < m_texSets.size(); i++)
    {
        TextureSet* set = m_texSets[i];

        if (sharedTextureHandle == (vr::SharedTextureHandle_t)set->m_sharedHandles[0] ||
            sharedTextureHandle == (vr::SharedTextureHandle_t)set->m_sharedHandles[1] ||
            sharedTextureHandle == (vr::SharedTextureHandle_t)set->m_sharedHandles[2])
        {
            set->Release();
            delete set;
            m_texSets.erase(m_texSets.begin() + i);

            break;
        }
    }
    LeaveCriticalSection(&m_texSetCS);
}

void OpenVR_HMDDriver::DestroyAllSwapTextureSets(uint32_t unPid)
{
    LOG("DestroyAllSwapTextureSets(pid=%u)\n", unPid);

    std::vector<TextureSet*> newSet;

    EnterCriticalSection(&m_texSetCS);

    for (auto set : m_texSets)
    {
        if (unPid != (~0) && set->m_pid != unPid)
        {
            newSet.push_back(set);
        }
        else
        {
            set->Release();
            delete set;
        }
    }
    m_texSets.swap(newSet);
    LeaveCriticalSection(&m_texSetCS);
}

void OpenVR_HMDDriver::GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t sharedTextureHandles[2], uint32_t(*pIndices)[2])
{
    EnterCriticalSection(&m_texSetCS);

    for (int i = 0; i < 2; i++)
    {
        (*pIndices)[i] = 0;

        int nextSwapChainTexIndex = -1; //enum the tex set to the right index instead of reusing set->m_index
                                        //since the app can request out of order tex handles i.e. instead of 
                                        //passing shared handles indexed @ 0,1,2 the app can come in with 0,2,1. 
        for (auto& set : m_texSets)
        {
            for (int sharedhandleIndex = 0; sharedhandleIndex < TextureSet::NUM_TEX; sharedhandleIndex++)
            {
                if (sharedTextureHandles[i] == (vr::SharedTextureHandle_t)set->m_sharedHandles[sharedhandleIndex])
                {
                    nextSwapChainTexIndex = sharedhandleIndex;
                    break;
                }
            }

            if (nextSwapChainTexIndex != -1)
            {
                set->m_nextIdx = (set->m_nextIdx + 1) % TextureSet::NUM_TEX;
                (*pIndices)[i] = set->m_nextIdx;

                break;
            }
        }
    }

    LeaveCriticalSection(&m_texSetCS);
}

void OpenVR_HMDDriver::SubmitLayer(const SubmitLayerPerEye_t(&perEye)[2])
{
}

void OpenVR_HMDDriver::Present(vr::SharedTextureHandle_t syncTexture)
{
}

void OpenVR_HMDDriver::GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnX = 0;
    *pnY = 0;
    *pnWidth = m_nRenderWidth;
    *pnHeight = m_nRenderHeight;
}

bool OpenVR_HMDDriver::IsDisplayOnDesktop()
{
    return false;
}

bool OpenVR_HMDDriver::IsDisplayRealDisplay()
{
    return false;
}

void OpenVR_HMDDriver::GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnWidth = m_nRenderWidth;
    *pnHeight = m_nRenderHeight;
}

void OpenVR_HMDDriver::GetEyeOutputViewport(vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
    *pnY = 0;
    *pnWidth = m_nRenderWidth;
    *pnHeight = m_nRenderHeight;

    if (eEye == vr::Eye_Left)
    {
        *pnX = 0;
    }
    else
    {
        *pnX = m_nRenderWidth / 2;
    }
}

void OpenVR_HMDDriver::GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
{
    *pfLeft = -1.0;
    *pfRight = 1.0;
    *pfTop = -1.0;
    *pfBottom = 1.0;
}

vr::DistortionCoordinates_t OpenVR_HMDDriver::ComputeDistortion(vr::EVREye eEye, float fU, float fV)
{
    vr::DistortionCoordinates_t coordinates;
    coordinates.rfBlue[0] = fU;
    coordinates.rfBlue[1] = fV;
    coordinates.rfGreen[0] = fU;
    coordinates.rfGreen[1] = fV;
    coordinates.rfRed[0] = fU;
    coordinates.rfRed[1] = fV;
    return coordinates;
}

#define CALC_JITTER 1

void OpenVR_HMDDriver::VSyncThread()
{
    LOG("OpenVR_HMDDriver::VSyncThread thread started, frequency %fHz, id %i, genstyle %i\n", m_flDisplayFrequency, std::this_thread::get_id(), m_vsyncStyle);

    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::duration<double, std::nano>;
    using Timepoint = std::chrono::time_point<Clock, Duration>;

    Duration period = std::chrono::seconds(1) / m_flDisplayFrequency;

    Timepoint last = Clock::now();

#if CALC_JITTER
    size_t n{ 0 }; // number of samples
    double sd{ 0 };  // sum of differences from period (for averaging), in milliseconds
    double ssd{ 0 }; // sum of squares of differences from period (for approximating standard deviation}, in milliseconds
    Timepoint lastVS{ Duration::zero() }; // time of last VSync event
#endif

    while (m_runVSyncThread)
    {
        Timepoint now = Clock::now();
        Timepoint next = last + period;
        Duration wait = next - now;
        if (wait.count() > 0)
        {
            switch (m_vsyncStyle)
            {
            case VSyncStyle::SLEEP:
                // simple sleep - the granularity of sleep_for varies by platform
                // and can be above 1ms - be cautious with higher frequencies
                // 'wait' often is also just a minimum
                std::this_thread::sleep_for(wait);
                break;

            case VSyncStyle::SLEEP_BUSY:
                // sleep one millisecond less, then busy wait
                if (wait > std::chrono::milliseconds(2))
                {
                    // minimum is one millisecond, so check for two in if
                    std::this_thread::sleep_for(wait - std::chrono::milliseconds(1));
                }
                while (Clock::now() < next)
                {
                }
                break;

            case VSyncStyle::BUSY:
                // just busy wait
                while (Clock::now() < next)
                {
                }
                break;
            }

            last = next;
        }
        else
        {
            last = now;
        }

        vr::VRServerDriverHost()->VsyncEvent(0);

#if CALC_JITTER
        if (lastVS != Timepoint(Duration::zero()))
        {
            now = Clock::now();
            double diff{ std::chrono::duration<double, std::micro>(now - lastVS).count() };
            sd += diff;
            double diffsq{ diff * diff };
            ssd += diff;
            ++n;
        }
        lastVS = now;
#endif

    }

    LOG("VSyncThread thread ending\n");

#if CALC_JITTER
    std::stringstream ss;

    double avg = sd / n;
    double stddev = sqrt( ssd / n );

    ss << "Jitter calculation, vsync style " << m_vsyncStyle << "\n";
    ss << "\t# Samples: " << n << "\n";
    ss << std::fixed << std::setprecision(3);
    ss << "\tAverage deviation:  " << avg << " us\n";
    ss << "\tStandard deviation: " << stddev << " us\n";
    LOG(ss.str().c_str());
#endif

}