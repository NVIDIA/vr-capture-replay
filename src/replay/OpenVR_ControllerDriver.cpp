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

#include "OpenVR_ControllerDriver.h"

#include "log.h"

OpenVR_ControllerDriver::OpenVR_ControllerDriver( std::string serial )
{
  if ( !serial.empty() )
  {
    m_sSerialNumber = serial;
  }
  LOG( "OpenVR_HMDDriver::OpenVR_ControllerDriver(%s)\n", serial.c_str() );
}

OpenVR_ControllerDriver::~OpenVR_ControllerDriver()
{
  LOG( "OpenVR_HMDDriver::~OpenVR_ControllerDriver()\n" );
}

void OpenVR_ControllerDriver::setDeviceData( VRData::Controller controller )
{
  m_sSerialNumber = controller.m_device.m_stringProperties[vr::Prop_SerialNumber_String].m_value;
  m_sModelNumber  = controller.m_device.m_stringProperties[vr::Prop_ModelNumber_String].m_value;

  LOG( "Controller Serial Number: %s\n", m_sSerialNumber.c_str() );
  LOG( "Controller Model Number: %s\n", m_sModelNumber.c_str() );

  m_controllerData = controller;
}

void OpenVR_ControllerDriver::Update( const vr::DriverPose_t & pose )
{
  // TODO: do we need a mutex here?
  m_pose = pose;
  if ( m_unObjectId != vr::k_unTrackedDeviceIndexInvalid )
  {
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated( m_unObjectId, m_pose, sizeof( vr::DriverPose_t ) );
  }
}

vr::EVRInitError OpenVR_ControllerDriver::Activate( vr::TrackedDeviceIndex_t unObjectId )
{
  m_unObjectId          = unObjectId;
  m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );
  LOG( "Controller Activate %i, %i\n", m_unObjectId, m_ulPropertyContainer );

  auto * vrp = vr::VRProperties();

  for ( const auto & p : m_controllerData.m_device.m_stringProperties )
  {
    vrp->SetStringProperty( m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value.c_str() );
  }
  for ( const auto & p : m_controllerData.m_device.m_boolProperties )
  {
    vrp->SetBoolProperty( m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value );
  }
  for ( const auto & p : m_controllerData.m_device.m_int32Properties )
  {
    if ( (vr::ETrackedDeviceProperty)p.first == vr::Prop_ControllerRoleHint_Int32 )
    {
      vr::ETrackedControllerRole role{ vr::ETrackedControllerRole( p.second.m_value ) };
      if ( VRData::Hand( m_controllerData.m_hand ) == VRData::Hand::LEFT )
      {
        role = vr::ETrackedControllerRole::TrackedControllerRole_LeftHand;
      }
      else if ( VRData::Hand( m_controllerData.m_hand ) == VRData::Hand::RIGHT )
      {
        role = vr::ETrackedControllerRole::TrackedControllerRole_RightHand;
      }
      vrp->SetInt32Property( m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, role );
    }
    else
    {
      vrp->SetInt32Property( m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value );
    }
  }
  for ( const auto & p : m_controllerData.m_device.m_uint64Properties )
  {
    vrp->SetUint64Property( m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value );
  }
  for ( const auto & p : m_controllerData.m_device.m_floatProperties )
  {
    vrp->SetFloatProperty( m_ulPropertyContainer, (vr::ETrackedDeviceProperty)p.first, p.second.m_value );
  }

  return vr::VRInitError_None;
}

void OpenVR_ControllerDriver::Deactivate()
{
  m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
}

void OpenVR_ControllerDriver::EnterStandby() {}

void * OpenVR_ControllerDriver::GetComponent( const char * pchComponentNameAndVersion )
{
  // override this to add a component to a driver
  return NULL;
}

// debug request from a client

void OpenVR_ControllerDriver::DebugRequest( const char * pchRequest, char * pchResponseBuffer, uint32_t unResponseBufferSize )
{
  if ( unResponseBufferSize >= 1 )
    pchResponseBuffer[0] = 0;
}

vr::DriverPose_t OpenVR_ControllerDriver::GetPose()
{
  return m_pose;
}