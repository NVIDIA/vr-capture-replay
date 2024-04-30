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

#include <assert.h>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <map>
#include <set>
#include <vector>

#if !defined( _OPENVR_DRIVER_API )
// both headers, openvr.h and openvr_driver.h define some data types!
#  include <openvr.h>
#endif

namespace VRData
{
  struct DevicePose
  {
    static const std::uint32_t version = 2;

    float m_pos[3];
    float m_rot[4];

    template <class T>
    void load( T & archive, std::uint32_t const archiveVersion )
    {
      assert( archiveVersion <= version );
      switch ( archiveVersion )
      {
        case 1:
          {
            double pos[3], rot[4];
            archive( pos, rot );
            for ( int i = 0; i < 3; ++i )
            {
              m_pos[i] = static_cast<float>( pos[i] );
              m_rot[i] = static_cast<float>( rot[i] );
            }
            m_rot[3] = static_cast<float>( rot[3] );
          }
          break;
        case 2: archive( m_pos, m_rot ); break;
        default: assert( false );
      }
    }

    template <class T>
    void save( T & archive, std::uint32_t ) const
    {
      archive( m_pos, m_rot );
    }
  };

  struct Device
  {
    static const std::uint32_t version = 1;

    vr::ETrackedDeviceClass m_deviceClass;
    DevicePose              m_initialPose;

    template <typename T>
    struct PropertyData
    {
      static const std::uint32_t version = 1;

      std::string m_propertyName;
      T           m_value;

      template <class Archive>
      void serialize( Archive & archive, std::uint32_t const version )
      {
        archive( CEREAL_NVP( m_propertyName ), CEREAL_NVP( m_value ) );
      };
    };

    std::map<uint32_t, PropertyData<bool>>                           m_boolProperties;
    std::map<uint32_t, PropertyData<float>>                          m_floatProperties;
    std::map<uint32_t, PropertyData<std::vector<float>>>             m_floatArrayProperties;
    std::map<uint32_t, PropertyData<int32_t>>                        m_int32Properties;
    std::map<uint32_t, PropertyData<std::vector<int32_t>>>           m_int32ArrayProperties;
    std::map<uint32_t, PropertyData<vr::HmdMatrix34_t>>              m_matrix34Properties;
    std::map<uint32_t, PropertyData<std::vector<vr::HmdMatrix34_t>>> m_matrix34ArrayProperties;
    std::map<uint32_t, PropertyData<std::string>>                    m_stringProperties;
    std::map<uint32_t, PropertyData<uint64_t>>                       m_uint64Properties;
    std::map<uint32_t, PropertyData<std::vector<vr::HmdVector4_t>>>  m_vector4ArrayProperties;

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( CEREAL_NVP( m_deviceClass ),
               CEREAL_NVP( m_initialPose ),
               CEREAL_NVP( m_stringProperties ),
               CEREAL_NVP( m_boolProperties ),
               CEREAL_NVP( m_int32Properties ),
               CEREAL_NVP( m_uint64Properties ),
               CEREAL_NVP( m_floatProperties ) );
    }
  };

  struct HMD
  {
    static const std::uint32_t version = 1;

    uint32_t m_renderWidth;
    uint32_t m_renderHeight;
    float    m_proj[2][4];
    Device   m_device;

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( CEREAL_NVP( m_renderWidth ), CEREAL_NVP( m_renderHeight ), CEREAL_NVP( m_proj ), CEREAL_NVP( m_device ) );
    }
  };

  enum class Hand
  {
    NONE,
    LEFT,
    RIGHT
  };

  struct Controller
  {
    static const std::uint32_t version = 1;

    Hand   m_hand{ 0 };
    Device m_device;

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( CEREAL_NVP( m_hand ), CEREAL_NVP( m_device ) );
    }
  };

  struct Chaperone
  {
    static const std::uint32_t version = 1;

    uint8_t m_universeOrigin;  // vr::ETrackingUniverseOrigin
    float   m_origin[3][4];
    float   m_playArea[2];

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( m_universeOrigin, m_origin, m_playArea );
    }
  };

  enum class ActionType
  {
    DIG,
    VEC1,
    VEC2
  };

  struct Action
  {
    static const std::uint32_t version = 1;

    ActionType  type;   // dig/vec1/vec2
    std::string name;   // trigger/trackpad/joystick/...
    std::string input;  // click/touch/value/...
    Hand        hand;   // VRData::Hand - none, left, right

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( type, name, input, hand );
    }
  };

  struct HardwareData
  {
    static const std::uint32_t version = 1;

    HMD                     m_hmd;  // OpenVR currently only allows a single HMD
    std::vector<Controller> m_controllers;
    Chaperone               m_chaperone;
    std::vector<Action>     m_actions;
    float                   m_captureFrequency;

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( CEREAL_NVP( m_hmd ), CEREAL_NVP( m_controllers ), m_chaperone, m_actions, m_captureFrequency );
    }
  };

  struct ActionData
  {
    static const std::uint32_t version = 1;

    size_t index;  // index into to HardwareData::m_actions
    float  val[2]{ 0.0f, 0.0f };

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( index, val );
    }
  };

  struct TrackingItem
  {
    static const std::uint32_t version = 2;

    // m_controllerPoses index corresponds with HardwareData::m_controllers
    float                   m_time;
    DevicePose              m_hmdPose;
    std::vector<DevicePose> m_controllerPoses;
    std::vector<ActionData> m_actionDataVec;

    template <class T>
    void load( T & archive, std::uint32_t const archiveVersion )
    {
      assert( archiveVersion <= version );
      switch ( archiveVersion )
      {
        case 1:
          {
            double time;
            archive( time );
            m_time = static_cast<float>( time );
          }
          break;
        case 2: archive( m_time ); break;
        default: assert( false );
      }
      archive( m_hmdPose, m_controllerPoses, m_actionDataVec );
    }

    template <class T>
    void save( T & archive, std::uint32_t ) const
    {
      archive( m_time, m_hmdPose, m_controllerPoses, m_actionDataVec );
    }
  };

  struct TrackingData
  {
    static const std::uint32_t version = 1;
    std::vector<TrackingItem>  m_trackingItems;

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( m_trackingItems );
    }
  };
}  // namespace VRData

// define macro to pull version info out of class itself
#define MAKE_CEREAL_CLASS_VERSION( TYPE ) CEREAL_CLASS_VERSION( TYPE, TYPE## ::version )

MAKE_CEREAL_CLASS_VERSION( VRData::DevicePose );
MAKE_CEREAL_CLASS_VERSION( VRData::Device );
MAKE_CEREAL_CLASS_VERSION( VRData::Device::PropertyData<std::string> );
MAKE_CEREAL_CLASS_VERSION( VRData::Device::PropertyData<bool> );
MAKE_CEREAL_CLASS_VERSION( VRData::Device::PropertyData<int32_t> );
MAKE_CEREAL_CLASS_VERSION( VRData::Device::PropertyData<uint64_t> );
MAKE_CEREAL_CLASS_VERSION( VRData::Device::PropertyData<float> );
MAKE_CEREAL_CLASS_VERSION( VRData::HMD );
MAKE_CEREAL_CLASS_VERSION( VRData::Controller );
MAKE_CEREAL_CLASS_VERSION( VRData::Chaperone );
MAKE_CEREAL_CLASS_VERSION( VRData::Action );
MAKE_CEREAL_CLASS_VERSION( VRData::HardwareData );
MAKE_CEREAL_CLASS_VERSION( VRData::ActionData );
MAKE_CEREAL_CLASS_VERSION( VRData::TrackingItem );
MAKE_CEREAL_CLASS_VERSION( VRData::TrackingData );
