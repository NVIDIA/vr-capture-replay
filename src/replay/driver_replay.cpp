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

#ifndef NOMINMAX
#  define NOMINMAX
#endif

#include "OpenVR_ControllerDriver.h"
#include "OpenVR_HMDDriver.h"
#include "log.h"
#include "vrutil.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <openvr_driver.h>
#include <thread>
#include <vector>

#if defined( _WINDOWS )
#  include <Psapi.h>
#  include <windows.h>
#endif

#if defined( _WIN32 )
#  define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#  define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined( __GNUC__ ) || defined( COMPILER_GCC ) || defined( __APPLE__ )
#  define HMD_DLL_EXPORT extern "C" __attribute__( ( visibility( "default" ) ) )
#  define HMD_DLL_IMPORT extern "C"
#else
#  error "Unsupported Platform."
#endif

HMD_DLL_EXPORT void * HmdDriverFactory( const char * pInterfaceName, int * pReturnCode );

struct Config
{
  std::string comment_0{ "More documentation for the parameters in readme.txt" };
  std::string comment_1{ "Replay speed as a time factor, e.g. 0.5 replays at half speed" };
  float       replaySpeed{ 1.0 };

  std::string comment_2{ "Frequency at which replayer generates input update events for SteamVR" };
  float       updateFrequency{ -1 };

  std::string comment_3{ "Interpolation between tracking samples" };
  bool        interpolate{ true };

  std::string comment_4{ "VSync generation for SteamVR" };
  bool        generateVSync{ true };

  std::string                  comment_5{ "VSync generation style. 0: Sleep, 1: Sleep&Busy, 2: Busy" };
  OpenVR_HMDDriver::VSyncStyle vSyncStyle{ OpenVR_HMDDriver::VSyncStyle::SLEEP_BUSY };

  std::string comment_6{ "Idle animation for emulated HMD and controllers" };
  bool        playIdleAnimation{ false };

  // TODO: override display resolution and frequency here, and other most used parameters?

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( comment_0 ),
             CEREAL_NVP( comment_1 ),
             CEREAL_NVP( replaySpeed ),
             CEREAL_NVP( comment_2 ),
             CEREAL_NVP( updateFrequency ),
             CEREAL_NVP( comment_3 ),
             CEREAL_NVP( interpolate ),
             CEREAL_NVP( comment_4 ),
             CEREAL_NVP( generateVSync ),
             CEREAL_NVP( comment_5 ),
             CEREAL_NVP( vSyncStyle ),
             CEREAL_NVP( comment_6 ),
             CEREAL_NVP( playIdleAnimation ) );
  }
};

class OpenVR_ServerDriver : public vr::IServerTrackedDeviceProvider
{
public:
  virtual vr::EVRInitError Init( vr::IVRDriverContext * pDriverContext ) override;
  virtual void             Cleanup() override;

  virtual const char * const * GetInterfaceVersions() override
  {
    return vr::k_InterfaceVersions;
  }

  virtual void RunFrame() override;

  virtual bool ShouldBlockStandbyMode() override
  {
    return true;
  }  // according to OpenVR dev, this is not called anymore

  virtual void EnterStandby() override {}

  virtual void LeaveStandby() override {}

private:
  struct ActionReplayData
  {
    VRData::Action               action;
    vr::VRInputComponentHandle_t handle[2]{ vr::k_ulInvalidInputComponentHandle, vr::k_ulInvalidInputComponentHandle };
  };

private:
  VRData::TrackingItem Interpolate( VRData::TrackingItem ti0, VRData::TrackingItem ti1, float seconds );  // function to interpolate between two tracking items
  void                 UpdateDevices();  // separate thread to read session file and call HMD and controller updates
  std::thread          m_updateDevices;
  bool                 m_runUpdateDevices{ true };

  void readConfig();
  void initActions();

  std::string m_tapePath;

  Config m_config;

  VRData::HardwareData                   m_hardwareData;
  bool                                   m_hasHardwareData = false;
  OpenVR_HMDDriver *                     m_hmdDriver       = nullptr;
  std::vector<OpenVR_ControllerDriver *> m_controllerDrivers;
  bool                                   m_actionsInitialized{ false };
  std::vector<ActionReplayData>          m_actionReplayDataVec;
  bool                                   m_chaperoneInitialized{ false };

  VRData::TrackingData                  m_trackingData;
  std::chrono::steady_clock::time_point m_startTime;
};

vr::EVRInitError OpenVR_ServerDriver::Init( vr::IVRDriverContext * pDriverContext )
{
  VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );

  HMODULE     hm = NULL;
  std::string modulePath;
  if ( GetModuleHandleExA( GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)&HmdDriverFactory, &hm ) )
  {
    TCHAR mypath[MAX_PATH];
    GetModuleFileNameA( hm, mypath, sizeof( mypath ) );
    std::string path( mypath );
    modulePath = path.substr( 0, path.find_last_of( '\\' ) );
  }
  else
  {
    // can't write to LOG when we can't look for the logs folder
    DriverLog( "GetModuleHandle failed: %#08x\n", GetLastError() );
    return vr::EVRInitError::VRInitError_Init_NotInitialized;
  }

  m_tapePath = std::filesystem::absolute( modulePath + "\\..\\..\\..\\tape" ).string();

  std::string logPath = std::filesystem::absolute( modulePath + "\\..\\..\\..\\logs" ).string();

  if ( !std::filesystem::exists( logPath ) )
  {
    logPath.clear();
  }

  initLog( logPath );

  LOG( "OpenVR_ServerDriver::Init()\n" );

  // check for NV GPU early to keep the log a little cleaner
  {
    RenderHelper renderHelper;
    if ( !renderHelper.hasGPU() )
    {
      int msgboxID = MessageBox( NULL, "Couldn't find a GPU", "Incompatible Hardware", MB_ICONERROR | MB_OK );
      LOG( "OpenVR_HMDDriver: ERROR: Initialization failed, no GPU found.\n" );
      return vr::VRInitError_Driver_Failed;
    }
  }

  readConfig();

  std::string hardwareFile = m_tapePath + "\\hardware.json";

  LOG( "Looking for %s ...\n", hardwareFile.c_str() );
  std::ifstream ifile( hardwareFile, std::ios::binary );
  if ( ifile )
  {
    try
    {
      LOG( "hardware.json found, loading...\n" );
      cereal::JSONInputArchive jsonInArchive( ifile );
      jsonInArchive( m_hardwareData );
      LOG( "hardware.json successfully loaded!\n" );
      m_hasHardwareData = true;
    }
    catch ( std::exception & e )
    {
      LOG( "Failed to load hardware.bin, exception: %s\n", e.what() );
      LOG( "The file may be outdated, please try the 'convert' tool to update the file\n" );
      LOG( "Using default values\n" );
      m_hardwareData = VRData::HardwareData();
    }
  }
  else
  {
    LOG( "hardware.json not found, using default values\n" );
  }

  if ( m_hasHardwareData )
  {
    // set up hmd data
    LOG( "Set HMD data...\n" );
    m_hmdDriver = new OpenVR_HMDDriver();
    m_hmdDriver->setDeviceData( m_hardwareData.m_hmd, m_config.generateVSync, m_config.vSyncStyle );

    // set up controller data
    LOG( "Set data for %i controllers...\n", m_hardwareData.m_controllers.size() );
    for ( auto & c : m_hardwareData.m_controllers )
    {
      m_controllerDrivers.push_back( new OpenVR_ControllerDriver() );
      m_controllerDrivers.back()->setDeviceData( c );
    }
  }
  else
  {
    m_hmdDriver = new OpenVR_HMDDriver();
    m_controllerDrivers.push_back( new OpenVR_ControllerDriver( "serial1" ) );
    m_controllerDrivers.push_back( new OpenVR_ControllerDriver( "serial2" ) );
  }

  if ( !vr::VRServerDriverHost()->TrackedDeviceAdded( m_hmdDriver->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_hmdDriver ) )
  {
    LOG( "OpenVR_ServerDriver: Failed to add HMD driver %s / %s\n", m_hmdDriver->GetModelNumber().c_str(), m_hmdDriver->GetSerialNumber().c_str() );
    return vr::VRInitError_Driver_Failed;
  }

  for ( const auto & c : m_controllerDrivers )
  {
    if ( !vr::VRServerDriverHost()->TrackedDeviceAdded( c->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, c ) )
    {
      LOG( "OpenVR_ServerDriver: Failed to add controller driver %s / %s\n", c->GetSerialNumber(), c->GetModelNumber() );
      // don't return failed, this is probably not fatal
    }
  }

  m_updateDevices = std::thread( &OpenVR_ServerDriver::UpdateDevices, this );

  return vr::VRInitError_None;
}

void OpenVR_ServerDriver::Cleanup()
{
  LOG( "OpenVR_ServerDriver::Cleanup()\n" );

  // stop device update thread
  m_runUpdateDevices = false;
  m_updateDevices.join();

  // clean up devices
  delete m_hmdDriver;
  m_hmdDriver = NULL;
  for ( auto & c : m_controllerDrivers )
  {
    delete c;
    c = nullptr;
  }
}

void OpenVR_ServerDriver::RunFrame()
{
  static bool first = true;
  if ( first )
  {
    std::string description;
    GPUResult   r = hasGPU( description );

    if ( r == GPUResult::foundGPU )
    {
      LOG( "GPU detected: %s\n", description.c_str() );
    }
    else
    {
      LOG( "No GPU found or SteamVR error, aborting\n" );
      exit( 1 );
    }

    first = false;
  }

  if ( !m_actionsInitialized )
  {
    initActions();
    m_actionsInitialized = true;
  }

  if ( !m_chaperoneInitialized && m_hasHardwareData )
  {
    m_chaperoneInitialized = true;
    setChaperone( m_hardwareData.m_chaperone );
  }

  vr::VREvent_t vrEvent;
  while ( vr::VRServerDriverHost()->PollNextEvent( &vrEvent, sizeof( vrEvent ) ) )
  {
    if ( vrEvent.eventType == vr::VREvent_SceneApplicationChanged )
    {
      vr::VREvent_Process_t pe = vrEvent.data.process;

      uint32_t    pid = pe.pid;
      std::string procname;
      if ( pid > 0 )
      {
        HANDLE handle = OpenProcess( PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, pid );
        if ( !handle )
        {
          LOGE( ( "OpenProcess for pid " + std::to_string( pid ) + " failed!" ).c_str() );
        }
        std::vector<TCHAR> buf( MAX_PATH );
        DWORD              ret = GetModuleFileNameEx( handle, 0, &buf[0], (DWORD)buf.size() );
        if ( !ret )
        {
          LOGE( ( "GetModuleFileNameEx failed: " + std::to_string( GetLastError() ) ).c_str() );
        }
        procname = &buf[0];
      }

      std::stringstream ss;
      ss << "Scene App changed: pid: " << pid << " " << procname << "\n";
      ss << "oldpid: " << pe.oldPid << " forced: " << pe.bForced << " connectionlost: " << pe.bConnectionLost << "\n";
      LOG( ss.str().c_str() );
    }
  }
}

VRData::TrackingItem OpenVR_ServerDriver::Interpolate( VRData::TrackingItem ti0, VRData::TrackingItem ti1, float alpha )
{
  auto lerp_pos = []( float p[3], float p0[3], float p1[3], float alpha )
  {
    p[0] = ( 1 - alpha ) * p0[0] + alpha * p1[0];
    p[1] = ( 1 - alpha ) * p0[1] + alpha * p1[1];
    p[2] = ( 1 - alpha ) * p0[2] + alpha * p1[2];
  };

  auto lerp_vec2 = []( float v[2], float v0[2], float v1[2], double alpha )
  {
    v[0] = (float)( ( 1 - alpha ) * v0[0] + alpha * v1[0] );
    v[1] = (float)( ( 1 - alpha ) * v0[1] + alpha * v1[1] );
  };

  auto quat_dot = []( float q0[4], float q1[4] ) { return q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]; };

  auto quat_normalize = []( float q[4] )
  {
    float d = std::sqrt( q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] );
    if ( d != 0.0 )
    {
      q[0] /= d;
      q[1] /= d;
      q[2] /= d;
      q[3] /= d;
    }
  };

  auto slerp_quat = [quat_dot, quat_normalize]( float q[4], float q0[4], float q1[4], float alpha )
  {
    float dp = quat_dot( q0, q1 );
    float sq0, sq1;

    if ( dp < 1.0 )
    {
      float theta = acos( dp );
      sq0         = sin( ( 1.0f - alpha ) * theta ) / sin( theta );
      sq1         = sin( alpha * theta ) / sin( theta );
    }
    else
    {
      sq0 = 1 - alpha;
      sq1 = alpha;
    }

    q[0] = sq0 * q0[0] + sq1 * q1[0];
    q[1] = sq0 * q0[1] + sq1 * q1[1];
    q[2] = sq0 * q0[2] + sq1 * q1[2];
    q[3] = sq0 * q0[3] + sq1 * q1[3];

    quat_normalize( q );
  };

  VRData::TrackingItem ti;

  // calculate HMD pose as interpolations between t0 and t1 based on alpha
  lerp_pos( ti.m_hmdPose.m_pos, ti0.m_hmdPose.m_pos, ti1.m_hmdPose.m_pos, alpha );
  slerp_quat( ti.m_hmdPose.m_rot, ti0.m_hmdPose.m_rot, ti1.m_hmdPose.m_rot, alpha );

  // interpolate controller poses
  for ( auto i = 0; i < ti0.m_controllerPoses.size(); ++i )
  {
    VRData::DevicePose t;
    lerp_pos( t.m_pos, ti0.m_controllerPoses[i].m_pos, ti1.m_controllerPoses[i].m_pos, alpha );
    slerp_quat( t.m_rot, ti0.m_controllerPoses[i].m_rot, ti1.m_controllerPoses[i].m_rot, alpha );
    ti.m_controllerPoses.push_back( t );
  }

  // don't interpolate inputs for now, just copy from ti0
  // TODO: interpolate? how? every ti has its own set of actions..
  ti.m_actionDataVec = ti0.m_actionDataVec;

  return ti;
}

void OpenVR_ServerDriver::UpdateDevices()
{
  LOG( "UpdateDevices thread started, id %i\n", std::this_thread::get_id() );

  // time is used for idle animation and replay timing
  m_startTime = std::chrono::steady_clock::now();

  // Update frequency from config file or (when <=0) use double capture frequency
  using Duration = std::chrono::duration<double, std::nano>;
  float frequency;
  if ( m_config.updateFrequency > 0 )
  {
    frequency = m_config.updateFrequency;
  }
  else
  {
    frequency = m_hardwareData.m_captureFrequency * 2;
  }

  Duration sleeptime = std::chrono::seconds( 1 ) / ( frequency );

  auto initPose = []()
  {
    vr::DriverPose_t pose         = { 0 };
    pose.poseIsValid              = true;
    pose.result                   = vr::TrackingResult_Running_OK;
    pose.deviceIsConnected        = true;
    pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
    pose.qDriverFromHeadRotation  = { 1, 0, 0, 0 };
    pose.poseTimeOffset           = 0.0;
    return pose;
  };

  auto normalize = []( vr::HmdQuaternion_t & q )
  {
    double d = std::sqrt( q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z );
    if ( d != 0.0 )
    {
      q.w /= d;
      q.x /= d;
      q.y /= d;
      q.z /= d;
    }
  };

  auto quatFromAngleAxis = []( double angle, std::array<double, 3> axis )
  {
    double              s = sin( angle / 2.0 );
    vr::HmdQuaternion_t quat{ cos( angle / 2.0 ), axis[0] * s, axis[1] * s, axis[2] * s };

    return quat;
  };

  auto multiplyQuaternions = []( const vr::HmdQuaternion_t & q1, const vr::HmdQuaternion_t q2 )
  {
    vr::HmdQuaternion_t q;
    q.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    q.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    q.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    q.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
    return q;
  };

  auto exists = []( std::string name )
  {
    std::ifstream file( name );
    return file.good();
  };
  std::string runFile   = m_tapePath + "\\run.bin";
  std::string readyFile = m_tapePath + "\\ready.flag";
  std::string busyFile  = m_tapePath + "\\busy.flag";
  LOG( "Will repeatedly look for %s for tracking data\n", runFile.c_str() );

  while ( m_runUpdateDevices )
  {
    if ( m_trackingData.m_trackingItems.empty() )
    {
      // create ready, delete busy flag file
      if ( !exists( readyFile ) && !static_cast<bool>( std::ofstream( readyFile ).put( ' ' ) ) )
      {
        LOG( std::format( "Error creating ready flag file: {}\n", readyFile ).c_str() );
      }
      if ( exists( busyFile ) && !std::filesystem::remove( busyFile ) )
      {
        LOG( std::format( "Error removing busy flag file: {}\n", busyFile ).c_str() );
      }
    }
    else
    {
      // create busy, delete ready flag file
      if ( !exists( busyFile ) && !static_cast<bool>( std::ofstream( busyFile ).put( ' ' ) ) )
      {
        LOG( std::format( "Error creating busy flag file: {}\n", busyFile ).c_str() );
      }
      if ( exists( readyFile ) && !std::filesystem::remove( readyFile ) )
      {
        LOG( std::format( "Error removing ready flag file: {}\n", readyFile ).c_str() );
      }
    }

    if ( m_trackingData.m_trackingItems.empty() )
    {
      // keep looking for tracking data while there's no data loaded
      std::ifstream ifile( runFile, std::ios::binary );
      if ( ifile )
      {
        LOG( "Loading tracking data from %s\n", runFile.c_str() );
        try
        {
          m_trackingData = VRData::TrackingData();
          cereal::BinaryInputArchive binInArchive( ifile );
          binInArchive( m_trackingData );

          if ( m_trackingData.m_trackingItems.size() < 2 )
          {
            // need two items for interpolation
            LOG( "Tracking data does not have at least two tracking items, ignoring\n" );
            m_trackingData = VRData::TrackingData();
          }
          else
          {
            LOG( "Tracking data loaded, %i tracking items\n", m_trackingData.m_trackingItems.size() );
            m_startTime = std::chrono::steady_clock::now();
          }

          readConfig();
        }
        catch ( std::exception & e )
        {
          LOG( "Failed to load tracking data, exception: %s\n", e.what() );
          LOG( "The file may be outdated, please try the 'convert' tool to update the file\n" );
        }
        ifile.close();
        std::remove( runFile.c_str() );
      }
      else
      {
        // keep updating the devices with initial tracking pose to signal the system is tracking
        double secondsForIdleAnimation = 0.0;
        if ( m_config.playIdleAnimation )
        {
          secondsForIdleAnimation = std::chrono::duration<double>( std::chrono::steady_clock::now() - m_startTime ).count();
        }

        vr::DriverPose_t pose = initPose();
        pose.vecPosition[0]   = 0.0;
        pose.vecPosition[1]   = 1.0;
        pose.vecPosition[2]   = 0.0;
        pose.qRotation        = quatFromAngleAxis( sin( secondsForIdleAnimation * 0.5 ) * 0.02, { 0, 1, 0 } );
        if ( m_hasHardwareData )
        {
          const auto & tracking = m_hardwareData.m_hmd.m_device.m_initialPose;
          const auto & p        = tracking.m_pos;
          const auto & r        = tracking.m_rot;
          pose.vecPosition[0]   = p[0];
          pose.vecPosition[1]   = p[1];
          pose.vecPosition[2]   = p[2];
          pose.qRotation        = multiplyQuaternions( pose.qRotation, { r[0], r[1], r[2], r[3] } );
        }
        m_hmdDriver->Update( pose );

        for ( int i = 0; i < m_controllerDrivers.size(); ++i )
        {
          vr::DriverPose_t pose = initPose();
          pose.vecPosition[0]   = -0.1 + i * 0.2;
          pose.vecPosition[1]   = 0.95;
          pose.vecPosition[2]   = -0.3;
          pose.qRotation        = quatFromAngleAxis( sin( secondsForIdleAnimation ) * 0.02, { 1, 0, 0 } );
          if ( m_hasHardwareData )
          {
            auto         controller = m_controllerDrivers[i];
            auto         tracking   = m_hardwareData.m_controllers[i].m_device.m_initialPose;
            const auto & p          = tracking.m_pos;
            const auto & r          = tracking.m_rot;
            pose.vecPosition[0]     = p[0];
            pose.vecPosition[1]     = p[1];
            pose.vecPosition[2]     = p[2];
            pose.qRotation          = multiplyQuaternions( pose.qRotation, { r[0], r[1], r[2], r[3] } );
          }
          m_controllerDrivers[i]->Update( pose );
        }
      }
    }
    else
    {
      // update using tracking data

      float seconds = m_config.replaySpeed * ( std::chrono::duration<float>( std::chrono::steady_clock::now() - m_startTime ).count() );

      VRData::TrackingItem & ti0 = m_trackingData.m_trackingItems[0];
      VRData::TrackingItem & ti1 = m_trackingData.m_trackingItems[1];

      VRData::TrackingItem ti;
      if ( m_config.interpolate )
      {
        float alpha = ( seconds - ti0.m_time ) / ( ti1.m_time - ti0.m_time );
        ti          = Interpolate( ti0, ti1, alpha );
      }
      else
      {
        ti = ti0;
      }

      // update HMD data
      {
        const auto & tracking = ti.m_hmdPose;
        const auto & p        = tracking.m_pos;
        const auto & r        = tracking.m_rot;

        vr::DriverPose_t pose = initPose();
        std::copy( std::begin( p ), std::end( p ), std::begin( pose.vecPosition ) );
        pose.qRotation = { r[0], r[1], r[2], r[3] };
        m_hmdDriver->Update( pose );
      }

      // update controller data
      for ( int i = 0; i < std::min( m_controllerDrivers.size(), ti.m_controllerPoses.size() ); ++i )
      {
        auto         controller = m_controllerDrivers[i];
        auto         tracking   = ti.m_controllerPoses[i];
        const auto & p          = tracking.m_pos;
        const auto & r          = tracking.m_rot;

        vr::DriverPose_t pose = initPose();
        std::copy( std::begin( p ), std::end( p ), std::begin( pose.vecPosition ) );
        pose.qRotation = { r[0], r[1], r[2], r[3] };
        controller->Update( pose );
      }

      // update action state
      auto input = vr::VRDriverInput();
      for ( const auto & actionData : ti.m_actionDataVec )
      {
        vr::EVRInputError err;

        if ( actionData.index >= m_actionReplayDataVec.size() )
        {
          static int num{ 0 };
          const int  max{ 15 };
          if ( ++num <= max )  // limit the number of times this error is reported
          {
            LOGE( "ERROR: Action Data index out of bounds. This can happen if incompatible tracking and hardware data files are mixed.\n" );
          }
          if ( num == max )
          {
            LOGE( "I'll stop reporting this error now.\n\n" );
          }
          continue;
        }

        const auto & actionReplayData = m_actionReplayDataVec[actionData.index];
        /*
        // depending on the way capture creates input tracking data, this may be a bit verbose
        LOG("action %i %s %s %i %i %i %f %f\n",
            actionData.index,
            actionReplayData.action.name.c_str(),
            actionReplayData.action.input.c_str(),
            actionReplayData.action.hand,
            actionReplayData.handle[0],
            actionReplayData.handle[1],
            actionData.val[0],
            actionData.val[1]
        );
        */

        switch ( VRData::ActionType( actionReplayData.action.type ) )
        {
          case VRData::ActionType::BOOLEAN:
            // LOG("sending bool input %s\n", (actionData.val[0]==0.0f)?"false":"true");
            err = input->UpdateBooleanComponent( actionReplayData.handle[0], actionData.val[0], 0.0 );
            break;
          case VRData::ActionType::SCALAR:
            // LOG("sending vec1 input %f\n", actionData.val[0]);
            err = input->UpdateScalarComponent( actionReplayData.handle[0], actionData.val[0], 0.0 );
            break;
          case VRData::ActionType::VECTOR2D:
            // LOG("sending vec2 input %f %f\n", actionData.val[0], actionData.val[1]);
            input->UpdateScalarComponent( actionReplayData.handle[0], actionData.val[0], 0.0 );
            err = input->UpdateScalarComponent( actionReplayData.handle[1], actionData.val[1], 0.0 );
            break;
        };
      }

      // if we're replaying slower than we captured, we may need to take out several items
      while ( !m_trackingData.m_trackingItems.empty() && seconds > m_trackingData.m_trackingItems[1].m_time )
      {
        // TODO: make sure that inputs are not dropped here?

        m_trackingData.m_trackingItems.erase( m_trackingData.m_trackingItems.begin() );

        // for interpolation, make sure we have >= 2 items or are empty
        // this will discard the last sample if not interpolating
        if ( m_trackingData.m_trackingItems.size() < 2 )
        {
          m_trackingData.m_trackingItems.clear();
          break;
        }
      }
    }

    std::this_thread::sleep_for( sleeptime );
  }

  // clean up state flag files
  if ( exists( readyFile ) && !std::filesystem::remove( readyFile ) )
  {
    LOG( std::format( "Error removing ready flag file: {}\n", readyFile ).c_str() );
  }
  if ( exists( busyFile ) && !std::filesystem::remove( busyFile ) )
  {
    LOG( std::format( "Error removing busy flag file: {}\n", busyFile ).c_str() );
  }

  LOG( "UpdateDevices thread ending\n" );
}

void OpenVR_ServerDriver::readConfig()
{
  std::string configFile = m_tapePath + "\\replay_config.json";

  LOG( "Looking for config file %s ... \n", configFile.c_str() );
  std::ifstream cfile( configFile );
  if ( cfile )
  {
    try
    {
      LOG( "Config found, loading...\n" );
      cereal::JSONInputArchive jsonInArchive( cfile );
      jsonInArchive( m_config );
      LOG( "Config successfully loaded\n" );
    }
    catch ( std::exception & e )
    {
      LOG( "Failed to load config, exception: %s\n", e.what() );
      LOG( "Using default values\n" );
      m_config = Config();
    }
  }
  else
  {
    LOG( "Config not found, using default values and writing file...\n" );
    std::ofstream cfile( configFile );
    m_config = Config();
    if ( cfile )
    {
      cereal::JSONOutputArchive jsonOutArchive( cfile );
      jsonOutArchive( m_config );
      LOG( "Config written\n" );
    }
    else
    {
      LOG( "Could not open config file for writing\n" );
    }
  }
}

void OpenVR_ServerDriver::initActions()
{
  if ( m_controllerDrivers.size() == 0 )
  {
    LOG( "No controllers defined in hardware.json, so no Actions can be defined. Check messages during recording if controllers were present.\n" );
    return;
  }

  if ( m_controllerDrivers[0]->getPropertyContainerHandle() != vr::k_ulInvalidPropertyContainer )
  {
    auto input = vr::VRDriverInput();

    // set up action data for controllers
    LOG( "Set up %i actions....\n", m_hardwareData.m_actions.size() );

    vr::PropertyContainerHandle_t leftContainer{ 0 };
    vr::PropertyContainerHandle_t rightContainer{ 0 };
    for ( auto & c : m_controllerDrivers )
    {
      if ( c->getHand() == VRData::Hand::LEFT )
      {
        leftContainer = c->getPropertyContainerHandle();
      }
      else if ( c->getHand() == VRData::Hand::RIGHT )
      {
        rightContainer = c->getPropertyContainerHandle();
      }
    }

    for ( auto & a : m_hardwareData.m_actions )
    {
      vr::PropertyContainerHandle_t container{ vr::k_ulInvalidPropertyContainer };
      ActionReplayData              ard;
      ard.action = a;

      std::string hand;
      if ( VRData::Hand( a.hand ) == VRData::Hand::LEFT )
      {
        container = leftContainer;
        hand      = "L";
      }
      else
      {
        container = rightContainer;
        hand      = "R";
      }

      std::string       name;
      vr::EVRInputError err;
      switch ( VRData::ActionType( a.type ) )
      {
        case VRData::ActionType::BOOLEAN:
          name = "/input/" + a.name + "/" + a.input;
          err  = input->CreateBooleanComponent( container, name.c_str(), &ard.handle[0] );
          LOG( "Setting up bool action: %s %s -> %i\n", hand.c_str(), name.c_str(), ard.handle[0] );
          if ( err != vr::EVRInputError::VRInputError_None )
          {
            LOG( "\tFailed: %i\n", err );
          }
          break;
        case VRData::ActionType::SCALAR:
          name = "/input/" + a.name + "/" + a.input;
          err  = input->CreateScalarComponent( container, name.c_str(), &ard.handle[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided );
          LOG( "Setting up scalar action: %s %s -> %i\n", hand.c_str(), name.c_str(), ard.handle[0] );
          if ( err != vr::EVRInputError::VRInputError_None )
          {
            LOG( "\tFailed: %i\n", err );
          }
          break;
        case VRData::ActionType::VECTOR2D:
          name = "/input/" + a.name + "/x";
          err  = input->CreateScalarComponent( container, name.c_str(), &ard.handle[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided );
          LOG( "Setting up 2D scalar action: %s %s -> %i\n", hand.c_str(), name.c_str(), ard.handle[0] );
          if ( err != vr::EVRInputError::VRInputError_None )
          {
            LOG( "\tFailed: %i\n", err );
          }
          name = "/input/" + a.name + "/y";
          err  = input->CreateScalarComponent( container, name.c_str(), &ard.handle[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided );
          LOG( "Setting up 2D scalar action: %s %s -> %i\n", hand.c_str(), name.c_str(), ard.handle[1] );
          if ( err != vr::EVRInputError::VRInputError_None )
          {
            LOG( "\tFailed: %i\n", err );
          }
          break;
      };

      m_actionReplayDataVec.push_back( ard );
    }
  }
}

class OpenVR_WatchdogDriver : public vr::IVRWatchdogProvider
{
public:
  OpenVR_WatchdogDriver()
  {
    m_pWatchdogThread = nullptr;
  }

  virtual vr::EVRInitError Init( vr::IVRDriverContext * pDriverContext );
  virtual void             Cleanup();

private:
  std::thread * m_pWatchdogThread;
};

vr::EVRInitError OpenVR_WatchdogDriver::Init( vr::IVRDriverContext * pDriverContext )
{
  VR_INIT_WATCHDOG_DRIVER_CONTEXT( pDriverContext );
  return vr::VRInitError_None;
}

void OpenVR_WatchdogDriver::Cleanup() {}

OpenVR_ServerDriver   g_serverDriver;
OpenVR_WatchdogDriver g_watchdogDriver;

HMD_DLL_EXPORT void * HmdDriverFactory( const char * pInterfaceName, int * pReturnCode )
{
  if ( 0 == strcmp( vr::IServerTrackedDeviceProvider_Version, pInterfaceName ) )
  {
    return &g_serverDriver;
  }
  if ( 0 == strcmp( vr::IVRWatchdogProvider_Version, pInterfaceName ) )
  {
    return &g_watchdogDriver;
  }

  if ( pReturnCode )
    *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;

  return NULL;
}