/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "D3D11_tex.h"

#include <openvr_driver.h>
#include <thread>

#define DRIVER_REPLAY
#include "shared/VRData.h"
#undef DRIVER_REPLAY

class OpenVR_HMDDriver
  : public vr::ITrackedDeviceServerDriver
  , public vr::IVRDriverDirectModeComponent
  , public vr::IVRDisplayComponent
{
public:
  enum VSyncStyle
  {
    SLEEP,
    SLEEP_BUSY,
    BUSY
  };

public:
  OpenVR_HMDDriver();

  virtual ~OpenVR_HMDDriver();

  void setDeviceData( VRData::HMD hmd, bool generateVSync, VSyncStyle style );

  std::string GetSerialNumber() const
  {
    return m_sSerialNumber;
  }

  std::string GetModelNumber() const
  {
    return m_sModelNumber;
  }

  float GetDisplayFrequency() const
  {
    return m_flDisplayFrequency;
  }

  vr::PropertyContainerHandle_t getPropertyContainerHandle() const
  {
    return m_ulPropertyContainer;
  }

  void Update( const vr::DriverPose_t & pose );

  // ITrackedDeviceServerDriver
  virtual vr::EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId ) override;
  virtual void             Deactivate() override;
  virtual void             EnterStandby() override;
  virtual void *           GetComponent( const char * pchComponentNameAndVersion ) override;
  virtual void             DebugRequest( const char * pchRequest, char * pchResponseBuffer, uint32_t unResponseBufferSize ) override;
  virtual vr::DriverPose_t GetPose() override;

  // IVRDriverDirectModeComponent
  virtual void CreateSwapTextureSet( uint32_t unPid, const SwapTextureSetDesc_t * pSwapTextureSetDesc, SwapTextureSet_t * pOutSwapTextureSet ) override;
  virtual void DestroySwapTextureSet( vr::SharedTextureHandle_t sharedTextureHandle ) override;
  virtual void DestroyAllSwapTextureSets( uint32_t unPid ) override;
  virtual void GetNextSwapTextureSetIndex( vr::SharedTextureHandle_t sharedTextureHandles[2], uint32_t ( *pIndices )[2] ) override;
  virtual void SubmitLayer( const SubmitLayerPerEye_t ( &perEye )[2] ) override;
  virtual void Present( vr::SharedTextureHandle_t syncTexture ) override;

  // IVRDisplayComponent
  virtual void                        GetWindowBounds( int32_t * pnX, int32_t * pnY, uint32_t * pnWidth, uint32_t * pnHeight ) override;
  virtual bool                        IsDisplayOnDesktop() override;
  virtual bool                        IsDisplayRealDisplay() override;
  virtual void                        GetRecommendedRenderTargetSize( uint32_t * pnWidth, uint32_t * pnHeight ) override;
  virtual void                        GetEyeOutputViewport( vr::EVREye eEye, uint32_t * pnX, uint32_t * pnY, uint32_t * pnWidth, uint32_t * pnHeight ) override;
  virtual void                        GetProjectionRaw( vr::EVREye eEye, float * pfLeft, float * pfRight, float * pfTop, float * pfBottom ) override;
  virtual vr::DistortionCoordinates_t ComputeDistortion( vr::EVREye eEye, float fU, float fV ) override;

private:
  void VSyncThread();

  bool m_isActivated{ false };

  vr::TrackedDeviceIndex_t      m_unObjectId{ vr::k_unTrackedDeviceIndexInvalid };
  vr::PropertyContainerHandle_t m_ulPropertyContainer{ vr::k_ulInvalidPropertyContainer };

  std::string m_sSerialNumber{ "default_HMD_serial" };
  std::string m_sModelNumber{ "default_HMD_model" };

  int32_t m_nRenderWidth{ 1000 };
  int32_t m_nRenderHeight{ 1000 };
  float   m_flSecondsFromVsyncToPhotons{ 0.01f };
  float   m_flDisplayFrequency{ 90 };
  float   m_flIPD{ 0.06f };
  bool    m_generateVSync{ true };

  RenderHelper              m_renderHelper;
  std::vector<TextureSet *> m_texSets;
  CRITICAL_SECTION          m_texSetCS;

  vr::DriverPose_t m_pose{ 0 };

  VRData::HMD m_hmdData;

  std::thread m_vSyncThread;
  bool        m_runVSyncThread{ true };
  VSyncStyle  m_vsyncStyle{ SLEEP_BUSY };
};