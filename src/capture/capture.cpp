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

#include "capture.h"

#include "shared/logging.h"

#include <source_location>

class Capture
{
public:
  Capture( bool noNotifyHMD, float sampleFrequency );
  ~Capture();

  VRData::HardwareData &                        getHardwareData();
  vr::TrackedDeviceIndex_t                      getMaxTrackedDeviceIndex() const;
  std::vector<vr::TrackedDeviceIndex_t> const & getTrackedDeviceIndices() const;
  VRData::TrackingData &                        getTrackingData();
  vr::VRActiveActionSet_t &                     getVRActiveActionSet();
  vr::IVRInput *                                getVRInput();
  vr::VROverlayHandle_t                         getVROverlayHandle() const;
  vr::IVRSystem *                               getVRSystem();
  void                                          resetTrackingData();

private:
  VRData::HardwareData                  m_HardwareData          = {};
  vr::TrackedDeviceIndex_t              m_maxTrackedDeviceIndex = {};
  std::vector<vr::TrackedDeviceIndex_t> m_trackedDeviceIndices  = {};
  VRData::TrackingData                  m_TrackingData          = {};
  vr::VRActiveActionSet_t               m_VRActiveActionSet     = {};
  vr::IVRInput *                        m_VRInput               = nullptr;
  vr::VROverlayHandle_t                 m_VROverlayHandle       = vr::k_ulOverlayHandleInvalid;
  vr::IVRSystem *                       m_VRSystem              = nullptr;
};

struct Args : MainArguments<Args>
{
  float                    sampleFreq = option( "sampling_freq", 's', "sampling frequency in Hz, default: 2xHMD display frequency" ) = -1.0f;
  std::vector<std::string> segmentAction = option( "segment_action", 'a', "recording segmenting action, e.g. left,trigger,click" )
                                             .validator( []( auto v ) { return v.size() == 3 || v.size() == 0; } );
  std::vector<std::string> startAction = option( "start_action", 'w', "recording starting action, e.g. left,trigger,click, can be same as segmentation action" )
                                           .validator( []( auto v ) { return v.size() == 3 || v.size() == 0; } );
  bool     noNotifyHMD = option( "noNotifications", 'n', "suppress notifications for start, segment, end in HMD" );
  uint32_t notifyTime = option( "note_time", 't', "for how long to show notifications, default: 1000ms" ) = 1000;
};

// interface and handles for notifications
vr::IVRNotifications * vrNotifications;

vr::VRActionHandle_t segmentActionHandle = vr::k_ulInvalidActionHandle;
vr::VRActionHandle_t startActionHandle   = vr::k_ulInvalidActionHandle;
std::string          startActionString;

struct ActionTemplate
{
  VRData::ActionType type;   // dig/vec1/vec2
  std::string        name;   // trigger/trackpad/joystick/...
  std::string        input;  // click/touch/value/...
};

struct ActionCaptureData
{
  vr::VRActionHandle_t handle;  // handle from OpenVR
  size_t               index;   // index into hardware.m_actions vector
                                // std::string path;             // path used for this handle (for debugging)
};

std::vector<ActionCaptureData> actionCaptureDataVec;

void appendActions( vr::IVRInput * vrInput, std::string modelNumber, VRData::Hand hand, std::vector<VRData::Action> & actions );
void checkFailure( bool success, std::string const & failureMessage = {}, std::source_location const location = std::source_location::current() );
void checkFailure( vr::ETrackedPropertyError  trackedPropertyError,
                   vr::IVRSystem *            vrSystem,
                   std::source_location const location = std::source_location::current() );
void checkFailure( vr::EVRInputError          inputError,
                   std::string const &        failureMessage,
                   std::string const &        successMessage = {},
                   std::source_location const location       = std::source_location::current() );
void checkFailure( vr::EVRInitError initError, std::string const & failureMessage, std::source_location const location = std::source_location::current() );
void checkFailure( vr::EVROverlayError        overlayError,
                   std::string const &        failureMessage,
                   std::source_location const location = std::source_location::current() );
VRData::Device                  collectDeviceProperties( vr::IVRSystem * vrSystem, const vr::TrackedDeviceIndex_t deviceId );
vr::VRActionHandle_t            getActionHandle( vr::IVRInput * vrInput, std::string const & actionString );
std::vector<VRData::DevicePose> getDevicePoses( vr::IVRSystem * vrSystem, uint32_t maxDeviceIndex, std::vector<uint32_t> const & deviceIndices );
vr::HmdVector3_t                get_position( vr::HmdMatrix34_t matrix );
vr::HmdQuaternion_t             get_rotation( vr::HmdMatrix34_t matrix );
void                            initLogging();
void                            notifyHMD( Capture const & capture, bool notify, std::string message, uint32_t notifyTime, uint32_t overrideTime = 0 );
std::unique_ptr<Args>           parseCommandLine( int argc, char * argv[] );
void                            printConfiguration( VRData::HardwareData const & hardwareData );
void                            printDeviceInfo( vr::IVRSystem * vrSystem, vr::TrackedDeviceIndex_t deviceIndex, vr::ETrackedDeviceClass deviceClass );
std::pair<std::string, vr::VRActionHandle_t>
            setupAction( vr::IVRInput * vrInput, std::vector<std::string> const & actionDescription, std::string const & actionKind );
std::string to_string( VRData::Action const & action );
void        waitForStartAction( Capture & capture, bool notifyHMD, uint32_t notifyTime );
void        writeHardwareData( VRData::HardwareData const & hardwareData );
void        writeTrackingData( VRData::TrackingData const & trackingData );

int main( int argc, char * argv[] )
{
  try
  {
    std::unique_ptr<Args> args = parseCommandLine( argc, argv );
    initLogging();
    logCommandLine( argc, argv );

    Capture capture( args->noNotifyHMD, args->sampleFreq );

    std::tie( startActionString, startActionHandle ) = setupAction( capture.getVRInput(), args->startAction, "start recording" );
    std::tie( std::ignore, segmentActionHandle )     = setupAction( capture.getVRInput(), args->segmentAction, "segment recording" );

    printConfiguration( capture.getHardwareData() );

    // wait for the start action, if configured
    if ( startActionHandle != vr::k_ulInvalidActionHandle )
    {
      waitForStartAction( capture, !args->noNotifyHMD, args->notifyTime );
    }

    auto start = std::chrono::steady_clock::now();
    // create hardware data file, contains the pose the driver defaults to
    {
      // set up initial poses
      std::vector<VRData::DevicePose> devicePoses =
        getDevicePoses( capture.getVRSystem(), capture.getMaxTrackedDeviceIndex(), capture.getTrackedDeviceIndices() );
      capture.getHardwareData().m_hmd.m_device.m_initialPose = devicePoses[0];
      for ( int i = 0; i < capture.getHardwareData().m_controllers.size(); ++i )
      {
        // NOTE: offset +1 between controllers and device poses (HMD is pose 0)
        capture.getHardwareData().m_controllers[i].m_device.m_initialPose = devicePoses[i + 1];
      }
      writeHardwareData( capture.getHardwareData() );
    }

    using Duration     = std::chrono::duration<double, std::nano>;
    Duration sleeptime = std::chrono::seconds( 1 ) / ( capture.getHardwareData().m_captureFrequency );

    LOGI( "\n\nRecording events, press ESC to stop, SPACE to write segment...\n\n" );

    notifyHMD( capture, !args->noNotifyHMD, "Starting VCR capture", args->notifyTime );

    while ( true )
    {
      if ( GetAsyncKeyState( VK_ESCAPE ) & 0x8000 )
      {
        // write and quit
        LOGI( "Generating final segment\n" );
        writeTrackingData( capture.getTrackingData() );
        break;
      }
      if ( GetAsyncKeyState( VK_SPACE ) & 0x8000 )
      {
        // write and re-init, continue recording
        LOGI( "Generating segment\n" );
        notifyHMD( capture, !args->noNotifyHMD, "Generating segment", args->notifyTime );
        writeTrackingData( capture.getTrackingData() );
        start = std::chrono::steady_clock::now();
        capture.resetTrackingData();

        while ( GetAsyncKeyState( VK_SPACE ) & 0x8000 )
        {
          // wait for key to be lifted again
        }
      }

      auto t = std::chrono::duration<float>( std::chrono::steady_clock::now() - start ).count();

      VRData::TrackingItem trackingItem;
      trackingItem.time = t;

      // record tracking data for HMD and controllers
      std::vector<VRData::DevicePose> devicePoses =
        getDevicePoses( capture.getVRSystem(), capture.getMaxTrackedDeviceIndex(), capture.getTrackedDeviceIndices() );  // returns HMD, CTR0, CTR1, ....
      // first item into HMD
      auto it                = devicePoses.begin();
      trackingItem.m_hmdPose = *it;
      // all other items into controller poses
      ++it;
      trackingItem.m_controllerPoses.assign( it, devicePoses.end() );

      // update input action state
      capture.getVRInput()->UpdateActionState( &capture.getVRActiveActionSet(), sizeof( vr::VRActiveActionSet_t ), 1 );

      // create segment if configured button was pressed
      if ( segmentActionHandle != vr::k_ulInvalidActionHandle )
      {
        // TODO: allow more than DIG input?
        vr::InputDigitalActionData_t data{ 0 };
        capture.getVRInput()->GetDigitalActionData( segmentActionHandle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
        if ( data.bState )
        {
          // write and re-init, continue recording
          LOGI( "Generating segment\n" );
          notifyHMD( capture, !args->noNotifyHMD, "Generating segment", args->notifyTime );
          writeTrackingData( capture.getTrackingData() );
          start = std::chrono::steady_clock::now();
          capture.resetTrackingData();
          while ( data.bState )
          {
            capture.getVRInput()->UpdateActionState( &capture.getVRActiveActionSet(), sizeof( vr::VRActiveActionSet_t ), 1 );
            capture.getVRInput()->GetDigitalActionData( segmentActionHandle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
          }
          // discard this tracking item - this button will not be logged
          continue;
        }
      }

      // record action data
      for ( auto & actionCaptureData : actionCaptureDataVec )
      {
        vr::VRActionHandle_t   handle = actionCaptureData.handle;
        size_t                 index  = actionCaptureData.index;
        const VRData::Action & action = capture.getHardwareData().m_actions[index];
        VRData::ActionData     actionData{ index };
        bool                   changed{ false };

        if ( VRData::ActionType( action.type ) == VRData::ActionType::DIG )
        {
          vr::InputDigitalActionData_t data{ 0 };
          capture.getVRInput()->GetDigitalActionData( handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
          changed           = data.bChanged;
          actionData.val[0] = data.bState;
        }
        else if ( VRData::ActionType( action.type ) == VRData::ActionType::VEC1 )
        {
          vr::InputAnalogActionData_t data{ 0 };
          capture.getVRInput()->GetAnalogActionData( handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
          changed           = data.deltaX != 0.0f;
          actionData.val[0] = data.x;
        }
        else if ( VRData::ActionType( action.type ) == VRData::ActionType::VEC2 )
        {
          vr::InputAnalogActionData_t data{ 0 };
          capture.getVRInput()->GetAnalogActionData( handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
          changed           = data.deltaX != 0.0f || data.deltaY != 0.0f;
          actionData.val[0] = data.x;
          actionData.val[1] = data.y;
        }

        // We could filter here to only write an action when it has changed, but
        // this creates issues with segment separation (last state may be wrong)
        // and interpolation on playback. I had prematurely optimized by using
        // bool changed here, and that caused issues.
        // File size doesn't seem to high, keeping this until file sizes become a problem.
        // SteamVR also seems to like updates a lot more often.

        trackingItem.m_actionDataVec.push_back( actionData );
      }

      capture.getTrackingData().m_trackingItems.push_back( trackingItem );
      std::this_thread::sleep_for( sleeptime );
    }
    if ( !args->noNotifyHMD )
    {
      notifyHMD( capture, !args->noNotifyHMD, "Shutting down VCR capture", args->notifyTime );
      // sleep to make sure the notification is shown
      std::this_thread::sleep_for( std::chrono::milliseconds( args->notifyTime ) );
    }
    auto t = std::chrono::duration<float>( std::chrono::steady_clock::now() - start ).count();
    LOGE( "\n\nDone, recorded %.1f seconds\n", t );
  }
  catch ( std::exception & e )
  {
    LOGE( "\n\nError: %s\n\n", e.what() );
  }

  return 0;
}

void appendActions( vr::IVRInput * vrInput, std::string modelNumber, VRData::Hand hand, std::vector<VRData::Action> & actions )
{
  std::vector<VRData::Action> newActions;
  if ( ( modelNumber.find( "VIVE" ) != std::string::npos ) || ( modelNumber.find( "Vive" ) != std::string::npos ) )
  {
    newActions = { { VRData::ActionType::DIG, "system", "click", hand },
                   { VRData::ActionType::DIG, "trigger", "click", hand },
                   { VRData::ActionType::DIG, "trigger", "touch", hand },
                   { VRData::ActionType::VEC1, "trigger", "value", hand },
                   { VRData::ActionType::DIG, "grip", "click", hand },
                   { VRData::ActionType::DIG, "trackpad", "click", hand },
                   { VRData::ActionType::DIG, "trackpad", "touch", hand },
                   { VRData::ActionType::VEC2, "trackpad", "value", hand },
                   { VRData::ActionType::DIG, "application_menu", "click", hand } };
  }
  else if ( modelNumber.find( "Oculus" ) != std::string::npos )
  {
    newActions = { { VRData::ActionType::DIG, "system", "click", hand },   { VRData::ActionType::DIG, "trigger", "click", hand },
                   { VRData::ActionType::DIG, "trigger", "touch", hand },  { VRData::ActionType::VEC1, "trigger", "value", hand },
                   { VRData::ActionType::VEC1, "grip", "value", hand },    { VRData::ActionType::DIG, "grip", "touch", hand },
                   { VRData::ActionType::DIG, "joystick", "click", hand }, { VRData::ActionType::DIG, "joystick", "touch", hand },
                   { VRData::ActionType::VEC2, "joystick", "value", hand } };
    if ( hand == VRData::Hand::RIGHT )
    {
      newActions.push_back( { VRData::ActionType::DIG, "a", "click", hand } );
      newActions.push_back( { VRData::ActionType::DIG, "b", "click", hand } );
    }
    else if ( hand == VRData::Hand::LEFT )
    {
      newActions.push_back( { VRData::ActionType::DIG, "x", "click", hand } );
      newActions.push_back( { VRData::ActionType::DIG, "y", "click", hand } );
    }
  }
  else if ( modelNumber.find( "Index" ) != std::string::npos )
  {
    // IMPLEMENT ME
    LOGE( "Unsupported controller, sorry, not setting up any actions.\n\n" );
  }
  else
  {
    LOGE( "Unsupported controller, sorry, not setting up any actions.\n\n" );
  }

  LOGI( "\t   Actions:" );
  std::string actionList;

  // enumerate actions, get handles for them
  for ( const auto & action : newActions )
  {
    actionList += "\n\t           " + action.name + "_" + action.input;

    actions.push_back( action );

    vr::VRActionHandle_t actionHandle = getActionHandle( vrInput, to_string( action ) );
    if ( actionHandle != vr::k_ulInvalidActionHandle )
    {
      actionCaptureDataVec.push_back( { actionHandle, actions.size() - 1 } );
    }
  }
  actionList += "\n";

  LOGI( "%s", actionList.c_str() );
}

void checkFailure( bool success, std::string const & failureMessage, std::source_location const location )
{
  if ( !success )
  {
    throw std::runtime_error( std::format( "\t{}({},{}) {}: {}",
                                           location.file_name(),
                                           location.line(),
                                           location.column(),
                                           location.function_name(),
                                           failureMessage.empty() ? "check failed" : failureMessage ) );
  }
}

void checkFailure( vr::ETrackedPropertyError trackedPropertyError, vr::IVRSystem * vrSystem, std::source_location const location )
{
  if ( trackedPropertyError != vr::TrackedProp_Success )
  {
    throw std::runtime_error( std::format( "\t{}({},{}) {}: {}",
                                           location.file_name(),
                                           location.line(),
                                           location.column(),
                                           location.function_name(),
                                           vrSystem->GetPropErrorNameFromEnum( trackedPropertyError ) ) );
  }
}

void checkFailure( vr::EVRInputError inputError, std::string const & failureMessage, std::string const & successMessage, const std::source_location location )
{
  if ( inputError == vr::VRInputError_None )
  {
    if ( !successMessage.empty() )
    {
      LOGI( "\t%s\n", successMessage.c_str() );
    }
  }
  else
  {
    throw std::runtime_error( std::format( "\t{}({},{}) {}: {} {}",
                                           location.file_name(),
                                           location.line(),
                                           location.column(),
                                           location.function_name(),
                                           failureMessage,
                                           magic_enum::enum_name( inputError ).data() ) );
  }
}

void checkFailure( vr::EVRInitError initError, std::string const & failureMessage, const std::source_location location )
{
  if ( initError != vr::EVRInitError::VRInitError_None )
  {
    throw std::runtime_error( std::format( "\t{}({},{}) {}: {}: {} {}",
                                           location.file_name(),
                                           location.line(),
                                           location.column(),
                                           location.function_name(),
                                           failureMessage,
                                           vr::VR_GetVRInitErrorAsSymbol( initError ),
                                           vr::VR_GetVRInitErrorAsEnglishDescription( initError ) ) );
  }
}

void checkFailure( vr::EVROverlayError overlayError, std::string const & failureMessage, const std::source_location location )
{
  if ( overlayError != vr::EVROverlayError::VROverlayError_None )
  {
    throw std::runtime_error( std::format( "\t{}({},{}) {}: {}: {}",
                                           location.file_name(),
                                           location.line(),
                                           location.column(),
                                           location.function_name(),
                                           failureMessage,
                                           vr::VROverlay()->GetOverlayErrorNameFromEnum( overlayError ) ) );
  }
}

VRData::Device collectDeviceProperties( vr::IVRSystem * vrSystem, const vr::TrackedDeviceIndex_t deviceId )
{
  VRData::Device device;
  device.m_deviceClass = vrSystem->GetTrackedDeviceClass( deviceId );

  // We're not interested in Prop_GraphicsAdapterLuid_Uint64 and Prop_Invalid
  // I don't know anything about Prop_ParentContainer
  // TODO: don't know how to get binary data
  // TODO: don't know how to get vector3 data
  std::set<vr::ETrackedDeviceProperty> skippedProperties{ vr::ETrackedDeviceProperty::Prop_DisplayColorMultLeft_Vector3,
                                                          vr::ETrackedDeviceProperty::Prop_DisplayColorMultRight_Vector3,
                                                          vr::ETrackedDeviceProperty::Prop_DisplayHiddenArea_Binary_End,
                                                          vr::ETrackedDeviceProperty::Prop_DisplayHiddenArea_Binary_Start,
                                                          vr::ETrackedDeviceProperty::Prop_DisplayMCImageData_Binary,
                                                          vr::ETrackedDeviceProperty::Prop_GraphicsAdapterLuid_Uint64,
                                                          vr::ETrackedDeviceProperty::Prop_ImuFactoryAccelerometerBias_Vector3,
                                                          vr::ETrackedDeviceProperty::Prop_ImuFactoryAccelerometerScale_Vector3,
                                                          vr::ETrackedDeviceProperty::Prop_ImuFactoryGyroBias_Vector3,
                                                          vr::ETrackedDeviceProperty::Prop_ImuFactoryGyroScale_Vector3,
                                                          vr::ETrackedDeviceProperty::Prop_Invalid,
                                                          vr::ETrackedDeviceProperty::Prop_ParentContainer };

  vr::ETrackedPropertyError error;
  for ( auto deviceProperty : magic_enum::enum_entries<vr::ETrackedDeviceProperty>() )
  {
    if ( !skippedProperties.contains( deviceProperty.first ) )
    {
      if ( deviceProperty.second.ends_with( "Bool" ) )
      {
        bool valueBool = vrSystem->GetBoolTrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_boolProperties[deviceProperty.first] = { std::string( deviceProperty.second ), valueBool };
        }
      }
      else if ( deviceProperty.second.ends_with( "Float" ) )
      {
        float valueFloat = vrSystem->GetFloatTrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_floatProperties[deviceProperty.first] = { std::string( deviceProperty.second ), valueFloat };
        }
      }
      else if ( deviceProperty.second.ends_with( "Float_Array" ) )
      {
        uint32_t byteSize = vrSystem->GetArrayTrackedDeviceProperty( deviceId, deviceProperty.first, vr::k_unFloatPropertyTag, nullptr, 0, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          assert( byteSize % sizeof( float ) == 0 );
          std::vector<float> valueFloatArray( byteSize / sizeof( float ) );
          vrSystem->GetArrayTrackedDeviceProperty( deviceId, deviceProperty.first, vr::k_unFloatPropertyTag, valueFloatArray.data(), byteSize, &error );
          checkFailure( error, vrSystem );
          device.m_floatArrayProperties[deviceProperty.first] = { std::string( deviceProperty.second ), valueFloatArray };
        }
      }
      else if ( deviceProperty.second.ends_with( "Int32" ) )
      {
        int32_t valueInt32 = vrSystem->GetInt32TrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_int32Properties[deviceProperty.first] = { std::string( deviceProperty.second ), valueInt32 };
        }
      }
      else if ( deviceProperty.second.ends_with( "Int32_Array" ) )
      {
        uint32_t byteSize = vrSystem->GetArrayTrackedDeviceProperty( deviceId, deviceProperty.first, vr::k_unInt32PropertyTag, nullptr, 0, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          assert( byteSize % sizeof( int32_t ) == 0 );
          std::vector<int32_t> valueInt32Array( byteSize / sizeof( int32_t ) );
          vrSystem->GetArrayTrackedDeviceProperty( deviceId, deviceProperty.first, vr::k_unInt32PropertyTag, valueInt32Array.data(), byteSize, &error );
          checkFailure( error, vrSystem );
          device.m_int32ArrayProperties[deviceProperty.first] = { std::string( deviceProperty.second ), valueInt32Array };
        }
      }
      else if ( deviceProperty.second.ends_with( "Matrix34" ) )
      {
        vr::HmdMatrix34_t valueMatrix34 = vrSystem->GetMatrix34TrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_matrix34Properties[deviceProperty.first] = { std::string( deviceProperty.second ), valueMatrix34 };
        }
      }
      else if ( deviceProperty.second.ends_with( "Matrix34_Array" ) )
      {
        uint32_t byteSize = vrSystem->GetArrayTrackedDeviceProperty( deviceId, deviceProperty.first, vr::k_unHmdMatrix34PropertyTag, nullptr, 0, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          assert( byteSize % sizeof( vr::HmdMatrix34_t ) == 0 );
          std::vector<vr::HmdMatrix34_t> valueMatrix34Array( byteSize / sizeof( vr::HmdMatrix34_t ) );
          vrSystem->GetArrayTrackedDeviceProperty(
            deviceId, deviceProperty.first, vr::k_unHmdMatrix34PropertyTag, valueMatrix34Array.data(), byteSize, &error );
          checkFailure( error, vrSystem );
          device.m_matrix34ArrayProperties[deviceProperty.first] = { std::string( deviceProperty.second ), valueMatrix34Array };
        }
      }
      else if ( deviceProperty.second.ends_with( "String" ) )
      {
        std::vector<char> buf( vr::k_unMaxPropertyStringSize );
        vrSystem->GetStringTrackedDeviceProperty( deviceId, deviceProperty.first, buf.data(), vr::k_unMaxPropertyStringSize, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_stringProperties[deviceProperty.first] = { std::string( deviceProperty.second ), std::string( buf.data() ) };
        }
      }
      else if ( deviceProperty.second.ends_with( "Uint64" ) )
      {
        uint64_t valueUint64 = vrSystem->GetUint64TrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_uint64Properties[deviceProperty.first] = { std::string( deviceProperty.second ), valueUint64 };
        }
      }
      else if ( deviceProperty.second.ends_with( "Vector4_Array" ) )
      {
        uint32_t byteSize = vrSystem->GetArrayTrackedDeviceProperty( deviceId, deviceProperty.first, vr::k_unHmdVector4PropertyTag, nullptr, 0, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          assert( byteSize % sizeof( vr::HmdVector4_t ) == 0 );
          std::vector<vr::HmdVector4_t> valueVector4Array( byteSize / sizeof( vr::HmdVector4_t ) );
          vrSystem->GetArrayTrackedDeviceProperty( deviceId, deviceProperty.first, vr::k_unHmdVector4PropertyTag, valueVector4Array.data(), byteSize, &error );
          checkFailure( error, vrSystem );
          device.m_vector4ArrayProperties[deviceProperty.first] = { std::string( deviceProperty.second ), valueVector4Array };
        }
      }
      else
      {
        checkFailure( false, "Encountered unknown property ending at " + std::string( deviceProperty.second ) );
      }
    }
  }
  return device;
}

vr::VRActionHandle_t getActionHandle( vr::IVRInput * vrInput, std::string const & actionString )
{
  // assemble input path of form /actions/record/in/actionString
  // these need to match the definitions in the controller json files
  vr::VRActionHandle_t actionHandle = vr::k_ulInvalidActionHandle;
  auto                 error        = vrInput->GetActionHandle( ( "/actions/record/in/" + actionString ).c_str(), &actionHandle );
  if ( error != vr::EVRInputError::VRInputError_None )
  {
    LOGE( "\tError getting action on %s: %i / %s\n", actionString.c_str(), error, magic_enum::enum_name( error ).data() );
  }
  return actionHandle;
}

std::vector<VRData::DevicePose> getDevicePoses( vr::IVRSystem * vrSystem, uint32_t maxDeviceIndex, std::vector<uint32_t> const & deviceIndices )
{
  std::vector<vr::TrackedDevicePose_t> trackedDevicePose( maxDeviceIndex + 1 );
  vrSystem->GetDeviceToAbsoluteTrackingPose( vr::TrackingUniverseRawAndUncalibrated, 0, trackedDevicePose.data(), (uint32_t)trackedDevicePose.size() );

  std::vector<VRData::DevicePose> devicePoses;
  for ( uint32_t i = 0; i < deviceIndices.size(); ++i )
  {
    uint32_t     deviceIndex = deviceIndices[i];
    const auto & pose        = trackedDevicePose[deviceIndex];

    auto               pos = get_position( pose.mDeviceToAbsoluteTracking );
    auto               rot = get_rotation( pose.mDeviceToAbsoluteTracking );
    VRData::DevicePose devicePose{ { pos.v[0], pos.v[1], pos.v[2] }, { rot.w, rot.x, rot.y, rot.z } };
    devicePoses.push_back( devicePose );
  }
  return devicePoses;
}

vr::HmdVector3_t get_position( vr::HmdMatrix34_t matrix )
{
  vr::HmdVector3_t vector;
  vector.v[0] = matrix.m[0][3];
  vector.v[1] = matrix.m[1][3];
  vector.v[2] = matrix.m[2][3];
  return vector;
};

vr::HmdQuaternion_t get_rotation( vr::HmdMatrix34_t matrix )
{
  vr::HmdQuaternion_t q;
  q.w = sqrt( fmax( 0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = sqrt( fmax( 0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.y = sqrt( fmax( 0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.z = sqrt( fmax( 0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = copysign( q.x, matrix.m[2][1] - matrix.m[1][2] );
  q.y = copysign( q.y, matrix.m[0][2] - matrix.m[2][0] );
  q.z = copysign( q.z, matrix.m[1][0] - matrix.m[0][1] );
  return q;
};

void initLogging()
{
  std::string logPath = std::filesystem::absolute( "..\\logs" ).string();
  if ( !std::filesystem::exists( logPath ) )
  {
    logPath.clear();
  }
  initLog( logPath );
}

void notifyHMD( Capture const & capture, bool notify, std::string message, uint32_t notifyTime, uint32_t overrideTime )
{
  if ( notify && vrNotifications )
  {
    vr::VRNotificationId       notificationId;
    vr::NotificationBitmap_t * messageIconPtr = nullptr;

    vr::EVRNotificationError nerr = vrNotifications->CreateNotification(
      capture.getVROverlayHandle(), 42, vr::EVRNotificationType_Transient, message.c_str(), vr::EVRNotificationStyle_None, messageIconPtr, &notificationId );
    if ( nerr == vr::VRNotificationError_OK )
    {
      // showing notification succeeded, start a thread to kill it later to not block
      auto killNotification = []( vr::VRNotificationId notificationId, uint32_t time )
      {
        std::this_thread::sleep_for( std::chrono::milliseconds( time ) );
        vr::EVRNotificationError nerr = vrNotifications->RemoveNotification( notificationId );
        if ( nerr != vr::VRNotificationError_OK )
        {
          LOGE( "Error removing notification: %i", nerr );
        }
      };

      uint32_t    time = overrideTime > 0 ? overrideTime : notifyTime;
      std::thread t( killNotification, notificationId, time );
      t.detach();
    }
    else
    {
      LOGE( "Error creating notification: %i", nerr );
    }
  }
}

std::unique_ptr<Args> parseCommandLine( int argc, char * argv[] )
{
  try
  {
    return std::make_unique<Args>( Args{ { argc, argv } } );
  }
  catch ( std::exception & e )
  {
    std::cerr << "Error parsing command line: " << e.what() << '\n';
    exit( 1 );
  }
}

void printConfiguration( VRData::HardwareData const & hardwareData )
{
  std::stringstream ss;

  size_t numControllers = hardwareData.m_controllers.size();
  size_t numDevices     = numControllers + 1;  // controllers + HMD

  ss << "\nTracking " << numDevices << " devices: \n";
  auto &                                                                        dev             = hardwareData.m_hmd.m_device;
  uint32_t                                                                      model           = vr::Prop_ModelNumber_String;
  uint32_t                                                                      serial          = vr::Prop_SerialNumber_String;
  std::map<uint32_t, VRData::Device::PropertyData<std::string>>::const_iterator modelPropertyIt = dev.m_stringProperties.find( model );
  assert( modelPropertyIt != dev.m_stringProperties.end() );
  std::map<uint32_t, VRData::Device::PropertyData<std::string>>::const_iterator serialPropertyIt = dev.m_stringProperties.find( serial );
  assert( serialPropertyIt != dev.m_stringProperties.end() );
  ss << "\tHMD: " << modelPropertyIt->second.m_value << " / " << serialPropertyIt->second.m_value << std::endl;
  for ( auto & c : hardwareData.m_controllers )
  {
    auto & dev      = c.m_device;
    modelPropertyIt = dev.m_stringProperties.find( model );
    assert( modelPropertyIt != dev.m_stringProperties.end() );
    serialPropertyIt = dev.m_stringProperties.find( serial );
    assert( serialPropertyIt != dev.m_stringProperties.end() );
    ss << "\tController: " << modelPropertyIt->second.m_value << " / " << serialPropertyIt->second.m_value << "  ";
    if ( VRData::Hand( c.m_hand ) == VRData::Hand::LEFT )
    {
      ss << "LEFT";
    }
    else if ( VRData::Hand( c.m_hand ) == VRData::Hand::RIGHT )
    {
      ss << "RIGHT";
    }
    ss << std::endl;
  }
  ss << "\nConfiguration:\n";
  ss << "\tSample Frequency [Hz]: \t" << hardwareData.m_captureFrequency << "\n";
  ss << "\n\n";

  LOGI( ss.str().c_str() );

  if ( hardwareData.m_actions.empty() )
  {
    LOGW( "\nWarning! Not recording any actions for attached controllers!\n\n" );
  }
}

void printDeviceInfo( vr::IVRSystem * vrSystem, vr::TrackedDeviceIndex_t deviceIndex, vr::ETrackedDeviceClass deviceClass )
{
  std::vector<char>         buf( vr::k_unMaxPropertyStringSize );
  vr::ETrackedPropertyError trackedPropertyError;

  std::stringstream ss;
  ss << "\nDevice " << deviceIndex << " Class " << deviceClass << " / " << magic_enum::enum_name( deviceClass ) << "\n";
  vrSystem->GetStringTrackedDeviceProperty( deviceIndex, vr::Prop_ModelNumber_String, buf.data(), vr::k_unMaxPropertyStringSize, &trackedPropertyError );
  checkFailure( trackedPropertyError, vrSystem );
  ss << "\t     Model: " << buf.data() << "\n";
  vrSystem->GetStringTrackedDeviceProperty( deviceIndex, vr::Prop_SerialNumber_String, buf.data(), vr::k_unMaxPropertyStringSize, &trackedPropertyError );
  ss << "\t    Serial: " << buf.data() << "\n";
  vrSystem->GetStringTrackedDeviceProperty( deviceIndex, vr::Prop_TrackingSystemName_String, buf.data(), vr::k_unMaxPropertyStringSize, &trackedPropertyError );
  ss << "\t  Tracking: " << buf.data() << "\n";

  if ( deviceClass == vr::TrackedDeviceClass_Controller )
  {
    vr::ETrackedControllerRole role = vrSystem->GetControllerRoleForTrackedDeviceIndex( deviceIndex );
    ss << "\t      Role: " << magic_enum::enum_name( role ) << "\n";
  }

  LOGI( "%s", ss.str().c_str() );
}

std::pair<std::string, vr::VRActionHandle_t>
  setupAction( vr::IVRInput * vrInput, std::vector<std::string> const & actionDescription, std::string const & actionKind )
{
  std::string          actionString;
  vr::VRActionHandle_t actionHandle = vr::k_ulInvalidActionHandle;

  if ( !actionDescription.empty() )
  {
    assert( 3 == actionDescription.size() );

    VRData::Hand hand =
      ( actionDescription[0] == "left" ) ? VRData::Hand::LEFT : ( ( actionDescription[0] == "right" ) ? VRData::Hand::RIGHT : VRData::Hand::NONE );
    VRData::Action action = { VRData::ActionType::DIG, actionDescription[1], actionDescription[2], hand };
    actionString          = to_string( action );
    LOGI( "\nSet up %s action... %s\n", actionKind.c_str(), actionString.c_str() );

    if ( hand != VRData::Hand::NONE )
    {
      actionHandle = getActionHandle( vrInput, actionString );
      if ( actionHandle != vr::k_ulInvalidActionHandle )
      {
        LOGI( "\tSuccess, use %s to %s\n", actionString.c_str(), actionKind.c_str() );
      }
    }
    else
    {
      LOGE( "\tInvalid hand string %s\n", actionDescription[0].c_str() );
    }
  }
  return std::make_pair( actionString, actionHandle );
}

std::string to_string( VRData::Action const & action )
{
  assert( action.hand != VRData::Hand::NONE );
  return std::string( ( action.hand == VRData::Hand::LEFT ) ? "left" : "right" ) + "_" + action.name + "_" + action.input;
}

void waitForStartAction( Capture & capture, bool notify, uint32_t notifyTime )
{
  LOGI( "Ready - use %s to start recording\n", startActionString.c_str() );
  notifyHMD( capture, notify, "Ready - use " + startActionString + " to start recording", notifyTime, 3000 );
  while ( true )
  {
    capture.getVRInput()->UpdateActionState( &capture.getVRActiveActionSet(), sizeof( capture.getVRActiveActionSet() ), 1 );
    vr::InputDigitalActionData_t data{ 0 };
    capture.getVRInput()->GetDigitalActionData( startActionHandle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
    if ( data.bState )
    {
      // wait for action to end
      while ( data.bState )
      {
        capture.getVRInput()->UpdateActionState( &capture.getVRActiveActionSet(), sizeof( capture.getVRActiveActionSet() ), 1 );
        capture.getVRInput()->GetDigitalActionData( segmentActionHandle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
      }
      break;
    }
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }
}

void writeHardwareData( VRData::HardwareData const & hardwareData )
{
  std::string hardwarePath = std::filesystem::absolute( "..\\tape\\hardware.json" ).string();
  LOGI( "Writing hardware file: %s\n", hardwarePath.c_str() );
  std::ofstream ofile( "..\\tape\\hardware.json", std::ios::binary );
  if ( !ofile )
  {
    throw std::runtime_error( std::format( "Failed to open for writing: {}", hardwarePath ) );
  }
  cereal::JSONOutputArchive jsonOutArchive( ofile );
  jsonOutArchive( hardwareData );
}

void writeTrackingData( VRData::TrackingData const & trackingData )
{
  // log currently active VR process
  uint32_t pid = vr::VRCompositor()->GetCurrentSceneFocusProcess();
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

    std::string pname = &buf[0];

    LOGI( ( "Current Scene Focus Process: " + std::to_string( pid ) + ", " + pname + "\n" ).c_str() );
  }
  else
  {
    LOGI( ( "Current Scene Focus Process: " + std::to_string( pid ) + ", no process currently presenting\n" ).c_str() );
  }

  // find first tracking_i that's free...
  int i = 0;
  while ( std::filesystem::exists( "..\\tape\\tracking_" + std::to_string( i ) + ".bin" ) )
  {
    ++i;
  }
  std::string trackingPath = std::filesystem::absolute( "..\\tape\\tracking_" + std::to_string( i ) + ".bin" ).string();

  // ...and write tracking file
  LOGI( "Writing tracking file: %s\n", trackingPath.c_str() );
  std::ofstream ofile( trackingPath, std::ios::binary );
  if ( !ofile )
  {
    throw std::runtime_error( std::format( "Failed to open for writing: {}", trackingPath ) );
  }
  cereal::BinaryOutputArchive binOutArchive( ofile );
  binOutArchive( trackingData );
}

Capture::Capture( bool noNotifyHMD, float sampleFrequency )
{
  // init the VRSystem
  vr::EVRInitError initError;
  m_VRSystem = vr::VR_Init( &initError, vr::VRApplication_Background );
  checkFailure( initError, "Failed to init VRSystem" );
  m_VRInput = vr::VRInput();

  // init active action set
  TCHAR path[MAX_PATH * 2];
  if ( GetModuleFileName( NULL, path, MAX_PATH ) )
  {
    strcpy_s( strrchr( path, '\\' ), MAX_PATH, "\\record_controller_actions.json" );
  }
  else
  {
    LOGE( "GetModuleFileName failed\n" );
  }

  LOGI( "\nSetting action manifest file: %s\n", path );
  checkFailure( m_VRInput->SetActionManifestPath( path ), "Failed", "Success" );

  LOGI( "\nGetting action set: %s\n", path );
  checkFailure( m_VRInput->GetActionSetHandle( "/actions/record", &m_VRActiveActionSet.ulActionSet ), "Failed", "Success" );

  // init chaperone
  vr::IVRCompositor *     vrCompositor        = vr::VRCompositor();
  vr::IVRChaperoneSetup * vrChaperoneSetup    = vr::VRChaperoneSetup();
  m_HardwareData.m_chaperone.m_universeOrigin = vrCompositor->GetTrackingSpace();
  checkFailure( vrChaperoneSetup->GetWorkingPlayAreaSize( &m_HardwareData.m_chaperone.m_playArea[0], &m_HardwareData.m_chaperone.m_playArea[1] ) );
  if ( vrCompositor->GetTrackingSpace() == vr::ETrackingUniverseOrigin::TrackingUniverseSeated )
  {
    checkFailure( vrChaperoneSetup->GetWorkingSeatedZeroPoseToRawTrackingPose( (vr::HmdMatrix34_t *)&m_HardwareData.m_chaperone.m_origin ) );
  }
  else
  {
    checkFailure( vrChaperoneSetup->GetWorkingStandingZeroPoseToRawTrackingPose( (vr::HmdMatrix34_t *)&m_HardwareData.m_chaperone.m_origin ) );
  }

  // init overlays
  if ( !noNotifyHMD )
  {
    vr::IVROverlay *      overlay = vr::VROverlay();
    vr::VROverlayHandle_t thumbnailHandle;
    vr::EVROverlayError   overlayError = overlay->CreateDashboardOverlay( "VCR", "VCR", &m_VROverlayHandle, &thumbnailHandle );
    checkFailure( overlayError, "Failed to create Overlay" );
    checkFailure( overlay->SetOverlayWidthInMeters( m_VROverlayHandle, 2.5f ), "Failed to set Overlay width in meters" );

    vr::EVRInitError initError;
    vrNotifications = (vr::IVRNotifications *)vr::VR_GetGenericInterface( vr::IVRNotifications_Version, &initError );
    checkFailure( initError, "Failed to get Notifications interface" );
  }

  // get the tracked devices
  LOGI( "\nEnumerating OpenVR devices...\n" );
  for ( uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i )
  {
    vr::TrackedDeviceIndex_t deviceIndex = static_cast<vr::TrackedDeviceIndex_t>( i );
    auto                     deviceClass = m_VRSystem->GetTrackedDeviceClass( deviceIndex );
    if ( deviceClass != vr::TrackedDeviceClass_Invalid )
    {
      printDeviceInfo( m_VRSystem, deviceIndex, deviceClass );

      switch ( deviceClass )
      {
        case vr::TrackedDeviceClass_HMD:
          checkFailure( deviceIndex == 0, "Encountered HMD Device at deviceIndex " + std::to_string( deviceIndex ) );

          m_trackedDeviceIndices.push_back( deviceIndex );
          m_VRSystem->GetRecommendedRenderTargetSize( &m_HardwareData.m_hmd.m_renderWidth, &m_HardwareData.m_hmd.m_renderHeight );
          {
            auto & p = m_HardwareData.m_hmd.m_proj;
            for ( int i = 0; i < 2; ++i )
            {
              m_VRSystem->GetProjectionRaw( (vr::EVREye)i, &p[i][0], &p[i][1], &p[i][2], &p[i][3] );
            }
          }
          m_HardwareData.m_hmd.m_device = collectDeviceProperties( m_VRSystem, deviceIndex );
          m_HardwareData.m_captureFrequency =
            ( 0 < sampleFrequency ) ? sampleFrequency : 2.0f * m_HardwareData.m_hmd.m_device.m_floatProperties[vr::Prop_DisplayFrequency_Float].m_value;
          break;

        case vr::TrackedDeviceClass_Controller:
          checkFailure( deviceIndex != 0, "Encountered Controller Device at deviceIndex 0, expecting an HMD there" );

          m_trackedDeviceIndices.push_back( deviceIndex );
          m_HardwareData.m_controllers.push_back( VRData::Controller() );
          VRData::Controller & c          = m_HardwareData.m_controllers.back();
          c.m_device                      = collectDeviceProperties( m_VRSystem, deviceIndex );
          vr::ETrackedControllerRole role = m_VRSystem->GetControllerRoleForTrackedDeviceIndex( deviceIndex );
          if ( ( role == vr::TrackedControllerRole_LeftHand ) || ( role == vr::TrackedControllerRole_RightHand ) )
          {
            c.m_hand = ( role == vr::TrackedControllerRole_LeftHand ) ? VRData::Hand::LEFT : VRData::Hand::RIGHT;
            appendActions( m_VRInput, c.m_device.m_stringProperties[vr::Prop_ModelNumber_String].m_value, c.m_hand, m_HardwareData.m_actions );
          }
          else if ( role == vr::TrackedControllerRole_Invalid )
          {
            LOGW( "\nWarning! This controller has an invalid role. Capturing only pose data, no action (e.g. button) inputs!\n\n" );
          }
          break;
      }
    }
  }
  checkFailure( !m_trackedDeviceIndices.empty(), "No tracked devices found" );
  checkFailure( std::ranges::count_if( m_trackedDeviceIndices,
                                       [this]( auto const & device )
                                       { return m_VRSystem->GetControllerRoleForTrackedDeviceIndex( device ) == vr::TrackedControllerRole_LeftHand; } ) <= 1,
                "More than one Left Hand controller tracked" );
  checkFailure( std::ranges::count_if( m_trackedDeviceIndices,
                                       [this]( auto const & device )
                                       { return m_VRSystem->GetControllerRoleForTrackedDeviceIndex( device ) == vr::TrackedControllerRole_RightHand; } ) <= 1,
                "More than one Right Hand controller tracked" );
  if ( std::ranges::find_if( m_HardwareData.m_controllers, []( auto const & controller ) { return controller.m_hand == VRData::Hand::LEFT; } ) ==
       m_HardwareData.m_controllers.end() )
  {
    LOGW( "\nWarning! No left hande controller found. This might cause broken capture data.\n\n" );
  }
  if ( std::ranges::find_if( m_HardwareData.m_controllers, []( auto const & controller ) { return controller.m_hand == VRData::Hand::RIGHT; } ) ==
       m_HardwareData.m_controllers.end() )
  {
    LOGW( "\nWarning! No right hand controller found. This might cause broken capture data.\n\n" );
  }
  m_maxTrackedDeviceIndex = *std::ranges::max_element( m_trackedDeviceIndices );
}

Capture::~Capture()
{
  LOGI( "Shutting down...\n" );
  vr::VR_Shutdown();
}

VRData::HardwareData & Capture::getHardwareData()
{
  return m_HardwareData;
}

vr::TrackedDeviceIndex_t Capture::getMaxTrackedDeviceIndex() const
{
  return m_maxTrackedDeviceIndex;
}

std::vector<vr::TrackedDeviceIndex_t> const & Capture::getTrackedDeviceIndices() const
{
  return m_trackedDeviceIndices;
}

VRData::TrackingData & Capture::getTrackingData()
{
  return m_TrackingData;
}

vr::VRActiveActionSet_t & Capture::getVRActiveActionSet()
{
  return m_VRActiveActionSet;
}

vr::IVRInput * Capture::getVRInput()
{
  return m_VRInput;
}

vr::VROverlayHandle_t Capture::getVROverlayHandle() const
{
  return m_VROverlayHandle;
}

vr::IVRSystem * Capture::getVRSystem()
{
  return m_VRSystem;
}

void Capture::resetTrackingData()
{
  m_TrackingData = {};
}