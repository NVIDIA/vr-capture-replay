/* Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of NVIDIA CORPORATION nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "thirdparty/cereal/archives/binary.hpp"
#include "thirdparty/cereal/archives/json.hpp"
#include "thirdparty/cereal/types/map.hpp"
#include "thirdparty/cereal/types/set.hpp"
#include "thirdparty/cereal/types/string.hpp"
#include "thirdparty/cereal/types/vector.hpp"

#include <map>
#include <set>
#include <vector>

namespace VRData
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
