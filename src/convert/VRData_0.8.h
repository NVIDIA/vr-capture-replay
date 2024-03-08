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

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <map>
#include <set>
#include <vector>

namespace VRData_0_8_unversioned
{
  struct DevicePose
  {
    double m_pos[3];
    double m_rot[4];

    template <class T>
    void serialize( T & archive )
    {
      archive( m_pos, m_rot );
    }
  };

  struct Device
  {
    uint32_t   m_deviceClass;
    DevicePose m_initialPose;

    template <typename T>
    struct PropertyData
    {
      std::string m_propertyName;
      T           m_value;

      template <class Archive>
      void serialize( Archive & archive )
      {
        archive( CEREAL_NVP( m_propertyName ), CEREAL_NVP( m_value ) );
      };
    };

    std::map<uint32_t, PropertyData<std::string>> m_stringProperties;
    std::map<uint32_t, PropertyData<bool>>        m_boolProperties;
    std::map<uint32_t, PropertyData<int32_t>>     m_int32Properties;
    std::map<uint32_t, PropertyData<uint64_t>>    m_uint64Properties;
    std::map<uint32_t, PropertyData<float>>       m_floatProperties;

    // missing:
    // GetMatrix34TrackedDeviceProperty
    // GetArrayTrackedDeviceProperty

    template <class Archive>
    void serialize( Archive & archive )
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
    uint32_t m_renderWidth;
    uint32_t m_renderHeight;
    float    m_proj[2][4];
    Device   m_device;

    template <class Archive>
    void serialize( Archive & archive )
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
    Hand   m_hand{ 0 };
    Device m_device;

    template <class Archive>
    void serialize( Archive & archive )
    {
      archive( CEREAL_NVP( m_hand ), CEREAL_NVP( m_device ) );
    }
  };

  struct Chaperone
  {
    uint8_t m_universeOrigin;  // vr::ETrackingUniverseOrigin
    float   m_origin[3][4];
    float   m_playArea[2];

    template <class Archive>
    void serialize( Archive & archive )
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
    ActionType  type;   // dig/vec1/vec2
    std::string name;   // trigger/trackpad/joystick/...
    std::string input;  // click/touch/value/...
    Hand        hand;   // VRData::Hand - none, left, right

    template <class Archive>
    void serialize( Archive & archive )
    {
      archive( type, name, input, hand );
    }
  };

  struct HardwareData
  {
    HMD                     m_hmd;  // OpenVR currently only allows a single HMD
    std::vector<Controller> m_controllers;
    Chaperone               m_chaperone;
    std::vector<Action>     m_actions;
    float                   m_captureFrequency;

    template <class Archive>
    void serialize( Archive & archive )
    {
      archive( CEREAL_NVP( m_hmd ), CEREAL_NVP( m_controllers ), m_chaperone, m_actions, m_captureFrequency );
    }
  };

  struct ActionData
  {
    size_t index;  // index into to HardwareData::m_actions
    float  val[2]{ 0.0f, 0.0f };

    template <class Archive>
    void serialize( Archive & archive )
    {
      archive( index, val );
    }
  };

  struct TrackingItem
  {
    // m_controllerPoses index corresponds with HardwareData::m_controllers

    double                  time;
    DevicePose              m_hmdPose;
    std::vector<DevicePose> m_controllerPoses;
    std::vector<ActionData> m_actionDataVec;

    template <class Archive>
    void serialize( Archive & archive )
    {
      archive( time, m_hmdPose, m_controllerPoses, m_actionDataVec );
    }
  };

  struct TrackingData
  {
    std::vector<TrackingItem> m_trackingItems;

    template <class Archive>
    void serialize( Archive & archive )
    {
      archive( m_trackingItems );
    }
  };

}  // namespace VRData_0_8_unversioned

namespace VRData_0_8_versioned
{
  struct DevicePose
  {
    static const std::uint32_t version = 1;

    double m_pos[3];
    double m_rot[4];

    template <class T>
    void serialize( T & archive, std::uint32_t const version )
    {
      archive( m_pos, m_rot );
    }

    DevicePose() {}

    DevicePose( const VRData_0_8_unversioned::DevicePose & t )
    {
      memcpy( m_pos, t.m_pos, sizeof( m_pos ) );
      memcpy( m_rot, t.m_rot, sizeof( m_rot ) );
    }
  };

  struct Device
  {
    static const std::uint32_t version = 1;

    uint32_t   m_deviceClass;
    DevicePose m_initialPose;

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

      template <typename T>
      PropertyData()
      {
      }

      template <typename T>
      PropertyData( const VRData_0_8_unversioned::Device::PropertyData<T> & t )
      {
        m_propertyName = t.m_propertyName;
        m_value        = t.m_value;
      }
    };

    std::map<uint32_t, PropertyData<std::string>> m_stringProperties;
    std::map<uint32_t, PropertyData<bool>>        m_boolProperties;
    std::map<uint32_t, PropertyData<int32_t>>     m_int32Properties;
    std::map<uint32_t, PropertyData<uint64_t>>    m_uint64Properties;
    std::map<uint32_t, PropertyData<float>>       m_floatProperties;

    // missing:
    // GetMatrix34TrackedDeviceProperty
    // GetArrayTrackedDeviceProperty

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

    Device() {}

    Device( const VRData_0_8_unversioned::Device & t )
    {
      m_deviceClass = t.m_deviceClass;
      m_initialPose = t.m_initialPose;
      m_stringProperties.insert( t.m_stringProperties.begin(), t.m_stringProperties.end() );
      m_boolProperties.insert( t.m_boolProperties.begin(), t.m_boolProperties.end() );
      m_int32Properties.insert( t.m_int32Properties.begin(), t.m_int32Properties.end() );
      m_uint64Properties.insert( t.m_uint64Properties.begin(), t.m_uint64Properties.end() );
      m_floatProperties.insert( t.m_floatProperties.begin(), t.m_floatProperties.end() );
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

    HMD() {}

    HMD( const VRData_0_8_unversioned::HMD & t )
    {
      m_renderWidth  = t.m_renderWidth;
      m_renderHeight = t.m_renderHeight;
      memcpy( m_proj, t.m_proj, sizeof( m_proj ) );
      m_device = t.m_device;
    }
  };

  /*
  enum class Hand
  {
      NONE, LEFT, RIGHT
  };
  */

  struct Controller
  {
    static const std::uint32_t version = 1;

    VRData_0_8_unversioned::Hand m_hand{ 0 };
    Device                       m_device;

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( CEREAL_NVP( m_hand ), CEREAL_NVP( m_device ) );
    }

    Controller() {}

    Controller( const VRData_0_8_unversioned::Controller & t )
    {
      m_hand   = t.m_hand;
      m_device = t.m_device;
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

    Chaperone() {}

    Chaperone( const VRData_0_8_unversioned::Chaperone & t )
    {
      m_universeOrigin = t.m_universeOrigin;
      memcpy( m_origin, t.m_origin, sizeof( m_origin ) );
      memcpy( m_playArea, t.m_playArea, sizeof( m_playArea ) );
    }
  };

  /*
  enum class ActionType
  {
      DIG, VEC1, VEC2
  };
  */

  struct Action
  {
    static const std::uint32_t version = 1;

    VRData_0_8_unversioned::ActionType type;   // dig/vec1/vec2
    std::string                        name;   // trigger/trackpad/joystick/...
    std::string                        input;  // click/touch/value/...
    VRData_0_8_unversioned::Hand       hand;   // VRData::Hand - none, left, right

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( type, name, input, hand );
    }

    Action() {}

    Action( const VRData_0_8_unversioned::Action & t )
    {
      type  = t.type;
      name  = t.name;
      input = t.input;
      hand  = t.hand;
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

    HardwareData() {}

    HardwareData( const VRData_0_8_unversioned::HardwareData & t )
    {
      m_hmd = t.m_hmd;
      m_controllers.assign( t.m_controllers.begin(), t.m_controllers.end() );
      m_chaperone = t.m_chaperone;
      m_actions.assign( t.m_actions.begin(), t.m_actions.end() );
      m_captureFrequency = t.m_captureFrequency;
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

    ActionData() {}

    ActionData( VRData_0_8_unversioned::ActionData t )
    {
      index = t.index;
      memcpy( val, t.val, sizeof( val ) );
    }
  };

  struct TrackingItem
  {
    static const std::uint32_t version = 1;

    // m_controllerPoses index corresponds with HardwareData::m_controllers
    double                  time;
    DevicePose              m_hmdPose;
    std::vector<DevicePose> m_controllerPoses;
    std::vector<ActionData> m_actionDataVec;

    template <class Archive>
    void serialize( Archive & archive, std::uint32_t const version )
    {
      archive( time, m_hmdPose, m_controllerPoses, m_actionDataVec );
    }

    TrackingItem() {}

    TrackingItem( VRData_0_8_unversioned::TrackingItem t )
    {
      time      = t.time;
      m_hmdPose = t.m_hmdPose;
      m_controllerPoses.assign( t.m_controllerPoses.begin(), t.m_controllerPoses.end() );
      m_actionDataVec.assign( t.m_actionDataVec.begin(), t.m_actionDataVec.end() );
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

    TrackingData() {}

    TrackingData( VRData_0_8_unversioned::TrackingData t )
    {
      m_trackingItems.assign( t.m_trackingItems.begin(), t.m_trackingItems.end() );
    }
  };
}  // namespace VRData_0_8_versioned

// define macro to pull version info out of class itself
#define MAKE_CEREAL_CLASS_VERSION( TYPE ) CEREAL_CLASS_VERSION( TYPE, TYPE## ::version )

MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::DevicePose );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Device );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Device::PropertyData<std::string> );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Device::PropertyData<bool> );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Device::PropertyData<int32_t> );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Device::PropertyData<uint64_t> );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Device::PropertyData<float> );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::HMD );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Controller );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Chaperone );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::Action );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::HardwareData );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::ActionData );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::TrackingItem );
MAKE_CEREAL_CLASS_VERSION( VRData_0_8_versioned::TrackingData );
