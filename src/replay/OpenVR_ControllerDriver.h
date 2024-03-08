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

#include <openvr_driver.h>

#define DRIVER_REPLAY
#include "shared/VRData.h"
#undef DRIVER_REPLAY

class OpenVR_ControllerDriver : public vr::ITrackedDeviceServerDriver
{
public:
  OpenVR_ControllerDriver( std::string serial = "" );

  virtual ~OpenVR_ControllerDriver();

  void setDeviceData( VRData::Controller controller );

  void Update( const vr::DriverPose_t & pose );

  std::string GetSerialNumber() const
  {
    return m_sSerialNumber;
  }

  std::string GetModelNumber() const
  {
    return m_sModelNumber;
  }

  vr::PropertyContainerHandle_t getPropertyContainerHandle() const
  {
    return m_ulPropertyContainer;
  }

  VRData::Hand getHand() const
  {
    return VRData::Hand( m_controllerData.m_hand );
  }

  virtual vr::EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId ) override;
  virtual void             Deactivate() override;
  virtual void             EnterStandby() override;
  virtual void *           GetComponent( const char * pchComponentNameAndVersion ) override;
  virtual void             DebugRequest( const char * pchRequest, char * pchResponseBuffer, uint32_t unResponseBufferSize ) override;
  virtual vr::DriverPose_t GetPose() override;

private:
  vr::TrackedDeviceIndex_t      m_unObjectId{ vr::k_unTrackedDeviceIndexInvalid };
  vr::PropertyContainerHandle_t m_ulPropertyContainer{ vr::k_ulInvalidPropertyContainer };

  std::string m_sSerialNumber{ "default_CTRL_serial" };
  std::string m_sModelNumber{ "default_CTRL_model" };

  vr::DriverPose_t m_pose;

  VRData::Controller m_controllerData;
};