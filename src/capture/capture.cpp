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

#include <cereal/types/optional.hpp>
#include <source_location>

using namespace std::literals::string_literals;

//=======================
// Command line arguments
struct Args : MainArguments<Args>
{
  bool                     logActions  = option( "logActions", 'l', "log detected actions" );
  bool                     noNotifyHMD = option( "noNotifications", 'n', "suppress notifications for start, segment, end in HMD" );
  uint32_t                 notifyTime = option( "note_time", 't', "for how long to show notifications, default: 1000ms" ) = 1000;
  float                    sampleFreq = option( "sampling_freq", 's', "sampling frequency in Hz, default: 2xHMD display frequency" ) = -1.0f;
  std::vector<std::string> segmentAction = option( "segment_action", 'a', "recording segmenting action, e.g. left,trigger,click" )
                                             .validator( []( auto v ) { return v.size() == 3 || v.size() == 0; } );
  std::vector<std::string> startAction = option( "start_action", 'w', "recording starting action, e.g. left,trigger,click, can be same as segmentation action" )
                                           .validator( []( auto v ) { return v.size() == 3 || v.size() == 0; } );
};

#define SERIALIZE_OPTIONAL( field ) \
  try                               \
  {                                 \
    archive( CEREAL_NVP( field ) ); \
  }                                 \
  catch ( ... )                     \
  {                                 \
  }

#define SERIALIZE_PSEUDO_MAP( Type, values )                    \
  template <class Archive>                                      \
  void save( Archive & archive ) const                          \
  {                                                             \
    for ( auto const & value : values )                         \
    {                                                           \
      archive( cereal::make_nvp( value.first, value.second ) ); \
    }                                                           \
  }                                                             \
                                                                \
  template <class Archive>                                      \
  void load( Archive & archive )                                \
  {                                                             \
    char const * name;                                          \
    do                                                          \
    {                                                           \
      name = archive.getNodeName();                             \
      if ( name )                                               \
      {                                                         \
        Type value;                                             \
        archive( value );                                       \
        values.emplace( name, std::move( value ) );             \
      }                                                         \
    } while ( name );                                           \
  }

//==============================================
// Representation of an action manifest file
struct DefaultBinding
{
  std::string controller_type;
  std::string binding_url;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( controller_type ), CEREAL_NVP( binding_url ) );
  }
};

struct Action
{
  std::string name;
  std::string type;

  template <class Archive>
  void serialize( Archive & archive ) const
  {
    archive( CEREAL_NVP( name ), CEREAL_NVP( type ) );
  }
};

struct ActionSet
{
  std::string name;
  std::string usage;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( name ), CEREAL_NVP( usage ) );
  }
};

struct ActionManifest
{
  std::vector<DefaultBinding> default_bindings;
  std::vector<Action>         actions;
  std::vector<ActionSet>      action_sets;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( default_bindings ), CEREAL_NVP( actions ), CEREAL_NVP( action_sets ) );
  }
};

void epilogue( cereal::JSONOutputArchive &, const ActionManifest & ) {}

void prologue( cereal::JSONOutputArchive &, const ActionManifest & ) {}

//==============================================
// Representation of a capture bindings file
struct Haptic
{
  std::string output;
  std::string path;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( output ), CEREAL_NVP( path ) );
  }
};

struct Pose
{
  std::string output;
  std::string path;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( output ), CEREAL_NVP( path ) );
  }
};

struct Input
{
  std::string output;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( output ) );
  }
};

struct Inputs
{
  std::map<std::string, Input> inputs;

  SERIALIZE_PSEUDO_MAP( Input, inputs );
};

struct Source
{
  Inputs      inputs;
  std::string mode;
  std::string path;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( inputs ), CEREAL_NVP( mode ), CEREAL_NVP( path ) );
  }
};

struct Actions
{
  std::vector<Haptic> haptics;
  std::vector<Pose>   poses;
  std::vector<Source> sources;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( haptics ), CEREAL_NVP( poses ), CEREAL_NVP( sources ) );
  }
};

struct Bindings
{
  Actions actions;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( ::cereal::make_nvp( "/actions/capture", actions ) );
  }
};

struct ControllerBindings
{
  Bindings    bindings;
  std::string category;
  std::string controller_type;
  std::string description;
  std::string name;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( bindings ), CEREAL_NVP( category ), CEREAL_NVP( controller_type ), CEREAL_NVP( description ), CEREAL_NVP( name ) );
  }
};

void epilogue( cereal::JSONOutputArchive &, const ControllerBindings & ) {}

void prologue( cereal::JSONOutputArchive &, const ControllerBindings & ) {}

//==============================================
// Representation of the file *_profile.json
struct InputBlock
{
  std::string type;
  std::string side;
  std::string skeleton;
  uint32_t    order = ~0;
  bool        click = false;
  bool        force = false;
  bool        touch = false;
  bool        value = false;

  template <class Archive>
  void serialize( Archive & archive )
  {
    // required fields
    archive( CEREAL_NVP( type ) );

    // optional fields
    SERIALIZE_OPTIONAL( click );
    SERIALIZE_OPTIONAL( force );
    SERIALIZE_OPTIONAL( order );
    SERIALIZE_OPTIONAL( side );
    SERIALIZE_OPTIONAL( skeleton );
    SERIALIZE_OPTIONAL( touch );
    SERIALIZE_OPTIONAL( value );
  }
};

struct InputSource
{
  std::map<std::string, InputBlock> inputs;

  SERIALIZE_PSEUDO_MAP( InputBlock, inputs );
};

struct Profile
{
  std::string controller_type;
  InputSource input_source;

  template <class Archive>
  void serialize( Archive & archive )
  {
    archive( CEREAL_NVP( controller_type ), CEREAL_NVP( input_source ) );
  }
};

void epilogue( cereal::JSONInputArchive &, const Profile & ) {}

void prologue( cereal::JSONInputArchive &, const Profile & ) {}

//==========================
class Capture
{
public:
  struct ActionCapture
  {
    vr::VRActionHandle_t handle;  // handle from OpenVR
    size_t               index;   // index into hardware.m_actions vector
  };

public:
  Capture( Args const & args );
  ~Capture();

  VRData::TrackingItem               createTrackingItem( std::chrono::time_point<std::chrono::steady_clock> const & start );
  void                               finishSegmentAction();
  std::vector<ActionCapture> const & getActionCaptures() const;
  VRData::HardwareData const &       getHardwareData() const;
  VRData::TrackingData &             getTrackingData();
  vr::IVRInput *                     getVRInput();
  void                               notifyHMD( bool notify, std::string message, uint32_t notifyTime, uint32_t overrideTime = 0 );
  bool                               segmentActionTriggered();
  void                               setupInitialPose();
  void                               waitForStartAction( bool notify, uint32_t notifyTime );
  void                               writeTrackingData();

private:
  bool                            addAction( std::string const & actionString, std::vector<Action> & actions );
  void                            addBindingSource( VRData::Action const & action, std::vector<Source> & sources );
  void                            appendActions( std::vector<VRData::Action> const & actions );
  std::vector<VRData::Action>     collectActions( VRData::Hand const & hand ) const;
  VRData::Device                  collectDeviceProperties( const vr::TrackedDeviceIndex_t deviceId ) const;
  void                            gatherTrackedDevices( float sampleFrequency );
  void                            generateActionManifestFile( std::string const & path );
  void                            generateControllerBindingsFile( std::filesystem::path const & path );
  vr::VRActionHandle_t            getActionHandle( std::string const & actionString ) const;
  std::vector<VRData::DevicePose> getDevicePoses();
  template <typename ValueType>
  void                                                          getDevicePropertyValueArray( vr::TrackedDeviceIndex_t                                                   deviceId,
                                                                                             std::pair<vr::ETrackedDeviceProperty, std::string_view> const &            deviceProperty,
                                                                                             std::map<uint32_t, VRData::Device::PropertyData<std::vector<ValueType>>> & properties ) const;
  void                                                          initActiveSectionSet();
  void                                                          initChaperone();
  void                                                          initOverlays( bool notifyHMD );
  void                                                          initVRSystem();
  void                                                          printConfiguration() const;
  void                                                          printConfiguration( std::stringstream & ss, VRData::Device const & device ) const;
  std::pair<std::string, Profile>                               readProfile( vr::TrackedDeviceIndex_t deviceIndex, VRData::Device const & device ) const;
  std::tuple<VRData::Action, std::string, vr::VRActionHandle_t> setupAction( std::vector<std::string> const & actionDescription,
                                                                             std::string const &              actionKind );
  void                                                          writeHardwareData();

private:
  struct ActionData
  {
    VRData::Action       action  = {};
    bool                 isAdded = {};
    vr::VRActionHandle_t handle  = vr::k_ulInvalidActionHandle;
    std::string          name    = {};
  };

private:
  std::vector<ActionCapture>            m_actionCaptures        = {};
  VRData::HardwareData                  m_hardwareData          = {};
  vr::TrackedDeviceIndex_t              m_maxTrackedDeviceIndex = {};
  Profile                               m_profile               = {};
  std::string                           m_profilePath           = {};
  ActionData                            m_segmentAction         = {};
  ActionData                            m_startAction           = {};
  std::vector<vr::TrackedDeviceIndex_t> m_trackedDeviceIndices  = {};
  VRData::TrackingData                  m_trackingData          = {};
  vr::VRActiveActionSet_t               m_vrActiveActionSet     = {};
  vr::IVRInput *                        m_vrInput               = nullptr;
  vr::IVRNotifications *                m_vrNotifications       = nullptr;
  vr::VROverlayHandle_t                 m_vrOverlayHandle       = vr::k_ulOverlayHandleInvalid;
  vr::IVRSystem *                       m_vrSystem              = nullptr;
};

void checkFailure( bool success, std::string const & failureMessage = {}, std::source_location const location = std::source_location::current() );
void checkFailure( vr::EVRApplicationError    applicationError,
                   vr::IVRApplications *      applications,
                   std::source_location const location = std::source_location::current() );
void checkFailure( vr::EVRSettingsError settingsError, vr::CVRSettingHelper & setting, std::source_location const location = std::source_location::current() );
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
vr::HmdVector3_t      get_position( vr::HmdMatrix34_t matrix );
vr::HmdQuaternionf_t  get_rotation( vr::HmdMatrix34_t matrix );
void                  initLogging();
std::unique_ptr<Args> parseCommandLine( int argc, char * argv[] );
void                  printDeviceInfo( vr::IVRSystem * vrSystem, vr::TrackedDeviceIndex_t deviceIndex, vr::ETrackedDeviceClass deviceClass );
std::string           to_string( VRData::Action const & action );
std::string           to_string( VRData::ActionType actionType );

int main( int argc, char * argv[] )
{
  try
  {
    std::unique_ptr<Args> args = parseCommandLine( argc, argv );
    initLogging();
    logCommandLine( argc, argv );

    Capture capture( *args );

    // wait for the start action, if configured
    capture.waitForStartAction( !args->noNotifyHMD, args->notifyTime );

    capture.setupInitialPose();

    using Duration           = std::chrono::duration<double, std::nano>;
    Duration const sleeptime = std::chrono::seconds( 1 ) / ( capture.getHardwareData().m_captureFrequency );

    LOGI( "\n\nRecording events, press ESC to stop, SPACE to write segment...\n\n" );

    capture.notifyHMD( !args->noNotifyHMD, "Starting VCR capture", args->notifyTime );

    auto start = std::chrono::steady_clock::now();
    while ( true )
    {
      if ( GetAsyncKeyState( VK_ESCAPE ) & 0x8000 )
      {
        // write and quit
        LOGI( "Generating final segment\n" );
        capture.notifyHMD( !args->noNotifyHMD, "Generating final segment", args->notifyTime );
        capture.writeTrackingData();
        break;
      }
      else if ( GetAsyncKeyState( VK_SPACE ) & 0x8000 )
      {
        // write and re-init, continue recording
        LOGI( "Generating segment\n" );
        capture.notifyHMD( !args->noNotifyHMD, "Generating segment", args->notifyTime );
        capture.writeTrackingData();

        while ( GetAsyncKeyState( VK_SPACE ) & 0x8000 )
        {
          // wait for key to be lifted again
        }
        start = std::chrono::steady_clock::now();
      }

      // create segment if configured button was pressed
      if ( capture.segmentActionTriggered() )
      {
        // write and re-init, continue recording
        LOGI( "Generating segment\n" );
        capture.notifyHMD( !args->noNotifyHMD, "Generating segment", args->notifyTime );
        capture.writeTrackingData();
        capture.finishSegmentAction();

        // discard this tracking item - this button will not be logged
        start = std::chrono::steady_clock::now();
        continue;
      }

      VRData::TrackingItem trackingItem = capture.createTrackingItem( start );

      // capture action data
      for ( auto & actionCapture : capture.getActionCaptures() )
      {
        vr::VRActionHandle_t   handle = actionCapture.handle;
        size_t                 index  = actionCapture.index;
        VRData::Action const & action = capture.getHardwareData().m_actions[index];
        VRData::ActionData     actionData{ index };

        vr::EVRInputError            inputError;
        vr::InputDigitalActionData_t digitalData{ 0 };
        vr::InputAnalogActionData_t  analogData{ 0 };

        switch ( action.type )
        {
          case VRData::ActionType::BOOLEAN:
            inputError = capture.getVRInput()->GetDigitalActionData( handle, &digitalData, sizeof( digitalData ), vr::k_ulInvalidInputValueHandle );
            assert( inputError == vr::EVRInputError::VRInputError_None );
            actionData.val[0] = digitalData.bState;
            if ( args->logActions && digitalData.bChanged )
            {
              LOGI( "%s: %s\n", to_string( action ).c_str(), digitalData.bState ? "pressed" : "released" );
            }
            break;
          case VRData::ActionType::SCALAR:
            inputError = capture.getVRInput()->GetAnalogActionData( handle, &analogData, sizeof( analogData ), vr::k_ulInvalidInputValueHandle );
            assert( inputError == vr::EVRInputError::VRInputError_None );
            actionData.val[0] = analogData.x;
            if ( args->logActions && ( analogData.deltaX != 0.0f ) )
            {
              LOGI( "%s: %f\n", to_string( action ).c_str(), analogData.x );
            }
            break;
          case VRData::ActionType::VECTOR2D:
            inputError = capture.getVRInput()->GetAnalogActionData( handle, &analogData, sizeof( analogData ), vr::k_ulInvalidInputValueHandle );
            assert( inputError == vr::EVRInputError::VRInputError_None );
            actionData.val[0] = analogData.x;
            actionData.val[1] = analogData.y;
            if ( args->logActions && ( ( analogData.deltaX != 0.0f ) || ( analogData.deltaY != 0.0f ) ) )
            {
              LOGI( "%s: %f, %f\n", to_string( action ).c_str(), analogData.x, analogData.y );
            }
            break;
          case VRData::ActionType::VECTOR3D: assert( false ); break;
          case VRData::ActionType::UNKNOWN: assert( false ); break;
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
      capture.notifyHMD( !args->noNotifyHMD, "Shutting down VCR capture", args->notifyTime );
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

void checkFailure( vr::EVRApplicationError applicationError, vr::IVRApplications * applications, std::source_location const location )
{
  if ( ( applicationError != vr::VRApplicationError_None ) && ( applicationError != vr::VRApplicationError_PropertyNotSet ) )
  {
    throw std::runtime_error( std::format( "\t{}({},{}) {}: {}",
                                           location.file_name(),
                                           location.line(),
                                           location.column(),
                                           location.function_name(),
                                           applications->GetApplicationsErrorNameFromEnum( applicationError ) ) );
  }
}

void checkFailure( vr::EVRSettingsError settingsError, vr::CVRSettingHelper & setting, std::source_location const location )
{
  if ( ( settingsError != vr::VRSettingsError_None ) && ( settingsError != vr::VRSettingsError_UnsetSettingHasNoDefault ) )
  {
    throw std::runtime_error( std::format( "\t{}({},{}) {}: {}",
                                           location.file_name(),
                                           location.line(),
                                           location.column(),
                                           location.function_name(),
                                           setting.GetSettingsErrorNameFromEnum( settingsError ) ) );
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

vr::HmdVector3_t get_position( vr::HmdMatrix34_t matrix )
{
  vr::HmdVector3_t vector;
  vector.v[0] = matrix.m[0][3];
  vector.v[1] = matrix.m[1][3];
  vector.v[2] = matrix.m[2][3];
  return vector;
};

vr::HmdQuaternionf_t get_rotation( vr::HmdMatrix34_t matrix )
{
  vr::HmdQuaternionf_t q;
  q.w = sqrt( std::max( 0.0f, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
  q.x = sqrt( std::max( 0.0f, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.y = sqrt( std::max( 0.0f, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2] ) ) / 2;
  q.z = sqrt( std::max( 0.0f, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2] ) ) / 2;
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

std::string to_string( VRData::Action const & action )
{
  assert( action.hand != VRData::Hand::NONE );
  return std::string( ( action.hand == VRData::Hand::LEFT ) ? "left" : "right" ) + "_" + action.name + "_" + action.input;
}

std::string to_string( VRData::ActionType actionType )
{
  switch ( actionType )
  {
    case VRData::ActionType::BOOLEAN: return "boolean";
    case VRData::ActionType::SCALAR: return "vector1";
    case VRData::ActionType::VECTOR2D: return "vector2";
    case VRData::ActionType::VECTOR3D: return "vector3";
    case VRData::ActionType::UNKNOWN:
    default: assert( false ); return "";
  }
}

Capture::Capture( Args const & args )
{
  initVRSystem();
  initChaperone();
  initOverlays( !args.noNotifyHMD );
  gatherTrackedDevices( args.sampleFreq );
  std::tie( m_startAction.action, m_startAction.name, m_startAction.handle )       = setupAction( args.startAction, "start recording" );
  std::tie( m_segmentAction.action, m_segmentAction.name, m_segmentAction.handle ) = setupAction( args.segmentAction, "segment recording" );
  initActiveSectionSet();
  printConfiguration();
}

Capture::~Capture()
{
  LOGI( "Shutting down...\n" );
  vr::VR_Shutdown();
}

VRData::TrackingItem Capture::createTrackingItem( std::chrono::time_point<std::chrono::steady_clock> const & start )
{
  VRData::TrackingItem trackingItem;
  trackingItem.m_time = std::chrono::duration<float>( std::chrono::steady_clock::now() - start ).count();

  // capture tracking data for HMD and controllers
  std::vector<VRData::DevicePose> devicePoses = getDevicePoses();  // returns HMD, CTR0, CTR1, ....
  // first item into HMD
  auto it                = devicePoses.begin();
  trackingItem.m_hmdPose = *it;
  // all other items into controller poses
  ++it;
  trackingItem.m_controllerPoses.assign( it, devicePoses.end() );

  return trackingItem;
}

void Capture::finishSegmentAction()
{
  assert( m_segmentAction.handle != vr::k_ulInvalidActionHandle );

  vr::InputDigitalActionData_t data{ 0 };
  do
  {
    m_vrInput->UpdateActionState( &m_vrActiveActionSet, sizeof( vr::VRActiveActionSet_t ), 1 );
    m_vrInput->GetDigitalActionData( m_segmentAction.handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
  } while ( data.bState );
}

std::vector<Capture::ActionCapture> const & Capture::getActionCaptures() const
{
  return m_actionCaptures;
}

VRData::HardwareData const & Capture::getHardwareData() const
{
  return m_hardwareData;
}

VRData::TrackingData & Capture::getTrackingData()
{
  return m_trackingData;
}

vr::IVRInput * Capture::getVRInput()
{
  return m_vrInput;
}

void Capture::notifyHMD( bool notify, std::string message, uint32_t notifyTime, uint32_t overrideTime )
{
  if ( notify && m_vrNotifications )
  {
    vr::VRNotificationId       notificationId;
    vr::NotificationBitmap_t * messageIconPtr = nullptr;

    vr::EVRNotificationError nerr = m_vrNotifications->CreateNotification(
      m_vrOverlayHandle, 42, vr::EVRNotificationType_Transient, message.c_str(), vr::EVRNotificationStyle_None, messageIconPtr, &notificationId );
    if ( nerr == vr::VRNotificationError_OK )
    {
      // showing notification succeeded, start a thread to kill it later to not block
      auto killNotification = []( vr::IVRNotifications * notifications, vr::VRNotificationId notificationId, uint32_t time )
      {
        std::this_thread::sleep_for( std::chrono::milliseconds( time ) );
        vr::EVRNotificationError nerr = notifications->RemoveNotification( notificationId );
        if ( nerr != vr::VRNotificationError_OK )
        {
          LOGE( "Error removing notification: %i", nerr );
        }
      };

      uint32_t    time = overrideTime > 0 ? overrideTime : notifyTime;
      std::thread t( killNotification, m_vrNotifications, notificationId, time );
      t.detach();
    }
    else
    {
      LOGE( "Error creating notification: %i", nerr );
    }
  }
}

bool Capture::segmentActionTriggered()
{
  m_vrInput->UpdateActionState( &m_vrActiveActionSet, sizeof( vr::VRActiveActionSet_t ), 1 );

  if ( m_segmentAction.handle != vr::k_ulInvalidActionHandle )
  {
    // TODO: allow more than BOOLEAN input?
    vr::InputDigitalActionData_t data{ 0 };
    m_vrInput->GetDigitalActionData( m_segmentAction.handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
    return data.bState;
  }
  return false;
}

void Capture::setupInitialPose()
{
  std::vector<VRData::DevicePose> devicePoses = getDevicePoses();
  assert( devicePoses.size() == 1 + m_hardwareData.m_controllers.size() );

  auto devicePoseIt                           = devicePoses.begin();
  m_hardwareData.m_hmd.m_device.m_initialPose = *devicePoseIt++;
  for ( auto & controller : m_hardwareData.m_controllers )
  {
    controller.m_device.m_initialPose = *devicePoseIt++;
  }
  writeHardwareData();
}

void Capture::waitForStartAction( bool notify, uint32_t notifyTime )
{
  if ( m_startAction.handle != vr::k_ulInvalidActionHandle )
  {
    LOGI( "Ready - use %s to start recording\n", m_startAction.name.c_str() );
    notifyHMD( notify, "Ready - use " + m_startAction.name + " to start recording", notifyTime, 3000 );
    while ( true )
    {
      checkFailure( m_vrInput->UpdateActionState( &m_vrActiveActionSet, sizeof( m_vrActiveActionSet ), 1 ),
                    "Failed to update action state while waiting for " + m_startAction.name,
                    "" );
      vr::InputDigitalActionData_t data{ 0 };
      checkFailure( m_vrInput->GetDigitalActionData( m_startAction.handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle ),
                    "Failed to get action data for " + m_startAction.name,
                    "" );
      if ( data.bState )
      {
        // wait for action to end
        while ( data.bState )
        {
          m_vrInput->UpdateActionState( &m_vrActiveActionSet, sizeof( m_vrActiveActionSet ), 1 );
          m_vrInput->GetDigitalActionData( m_segmentAction.handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
        }
        break;
      }
      std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
    }
  }
}

void Capture::writeTrackingData()
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
  LOGI( "\ttracked %d poses in %.1f seconds", m_trackingData.m_trackingItems.size(), m_trackingData.m_trackingItems.back().m_time );
  std::ofstream ofile( trackingPath, std::ios::binary );
  if ( !ofile )
  {
    throw std::runtime_error( std::format( "Failed to open for writing: {}", trackingPath ) );
  }
  cereal::BinaryOutputArchive binOutArchive( ofile );
  binOutArchive( m_trackingData );

  // reset the tracking data after they've been written
  m_trackingData = {};
}

bool Capture::addAction( std::string const & actionString, std::vector<Action> & actions )
{
  bool added = false;
  if ( !actionString.empty() &&
       std::find_if( actions.begin(), actions.end(), [&actionString]( Action const & action ) { return action.name.ends_with( actionString ); } ) ==
         actions.end() )
  {
    actions.push_back( { "/actions/capture/in/" + actionString, "boolean" } );
    added = true;
  }
  return added;
}

void Capture::addBindingSource( VRData::Action const & action, std::vector<Source> & sources )
{
  assert( ( action.hand == VRData::Hand::LEFT ) || ( action.hand == VRData::Hand::RIGHT ) );
  std::string hand = ( action.hand == VRData::Hand::LEFT ) ? "left" : "right";
  std::string path = "/user/hand/" + hand + "/input/" + action.name;

  auto sourcesIt = std::find_if( sources.begin(), sources.end(), [&path]( Source const & source ) { return source.path == path; } );
  if ( sourcesIt == sources.end() )
  {
    auto inputIt = m_profile.input_source.inputs.find( "/input/" + action.name );
    assert( inputIt != m_profile.input_source.inputs.end() );
    sources.emplace_back( Inputs(), inputIt->second.type, path );
    sourcesIt = std::prev( sources.end() );
  }
  assert( !sourcesIt->inputs.inputs.contains( action.input ) );
  std::string key               = ( action.input == "value" )
                                  ? ( ( sourcesIt->mode == "joystick" ) ? "position" : ( ( sourcesIt->mode == "trigger" ) ? "pull" : action.input ) )
                                  : action.input;
  sourcesIt->inputs.inputs[key] = { "/actions/capture/in/" + to_string( action ) };
}

void Capture::appendActions( std::vector<VRData::Action> const & actions )
{
  LOGI( "\t   Actions:" );
  std::string actionList;

  // enumerate actions, get handles for them
  for ( const auto & action : actions )
  {
    actionList += "\n\t           " + action.name + "_" + action.input;

    vr::VRActionHandle_t actionHandle = getActionHandle( to_string( action ) );
    if ( actionHandle != vr::k_ulInvalidActionHandle )
    {
      m_actionCaptures.push_back( { actionHandle, m_hardwareData.m_actions.size() } );
      m_hardwareData.m_actions.push_back( action );
    }
    else
    {
      actionList += " (not handled!)";
    }
  }
  actionList += "\n";

  LOGI( "%s", actionList.c_str() );
}

std::vector<VRData::Action> Capture::collectActions( VRData::Hand const & hand ) const
{
  assert( ( hand == VRData::Hand::LEFT ) || ( hand == VRData::Hand::RIGHT ) );
  std::string side = ( hand == VRData::Hand::LEFT ) ? "left" : "right";

  std::vector<VRData::Action> actions;
  for ( auto const & input : m_profile.input_source.inputs )
  {
    if ( ( input.second.side.empty() || ( input.second.side == side ) ) &&
         ( input.second.click || input.second.force || input.second.touch || input.second.value ) )
    {
      assert( input.first.starts_with( "/input/" ) );
      std::string name = input.first.substr( strlen( "/input/" ) );
      if ( input.second.type == "button" )
      {
        // click is automatic
        actions.push_back( { VRData::ActionType::BOOLEAN, name, "click", hand } );
        if ( input.second.touch )
        {
          actions.push_back( { VRData::ActionType::BOOLEAN, name, "touch", hand } );
        }
        if ( input.second.force || input.second.value )
        {
          throw std::runtime_error( std::format( "Encountered unsupported component <{}> for input type <{}> named <{}>",
                                                 input.second.force ? "force" : "value",
                                                 input.second.type.c_str(),
                                                 name.c_str() ) );
        }
      }
      else if ( input.second.type == "haptic" )
      {
        // TODO: determine what to do with such an input!
      }
      else if ( input.second.type == "joystick" )
      {
        if ( input.second.click )
        {
          actions.push_back( { VRData::ActionType::BOOLEAN, name, "click", hand } );
        }
        if ( input.second.touch )
        {
          actions.push_back( { VRData::ActionType::BOOLEAN, name, "touch", hand } );
        }
        // value is automatic
        actions.push_back( { VRData::ActionType::VECTOR2D, name, "value", hand } );
        if ( input.second.force )
        {
          throw std::runtime_error(
            std::format( "Encountered unsupported component <force> for input type <{}> named <{}>", input.second.type.c_str(), name.c_str() ) );
        }
      }
      else if ( input.second.type == "pose" )
      {
        // TODO: determine what to do with such an input!
      }
      else if ( input.second.type == "skeleton" )
      {
        // TODO: determine what to do with such an input!
      }
      else if ( input.second.type == "trackpad" )
      {
        if ( input.second.click )
        {
          actions.push_back( { VRData::ActionType::BOOLEAN, name, "click", hand } );
        }
        if ( input.second.force )
        {
          actions.push_back( { VRData::ActionType::SCALAR, name, "force", hand } );
        }
        // position and touch are automatic
        actions.push_back( { VRData::ActionType::VECTOR2D, name, "position", hand } );
        actions.push_back( { VRData::ActionType::BOOLEAN, name, "touch", hand } );
        if ( input.second.value )
        {
          throw std::runtime_error(
            std::format( "Encountered unsupported component <value> for input type <{}> named <{}>", input.second.type.c_str(), name.c_str() ) );
        }
      }
      else if ( input.second.type == "trigger" )
      {
        if ( input.second.click )
        {
          actions.push_back( { VRData::ActionType::BOOLEAN, name, "click", hand } );
        }
        if ( input.second.force )
        {
          actions.push_back( { VRData::ActionType::SCALAR, name, "force", hand } );
        }
        if ( input.second.touch )
        {
          actions.push_back( { VRData::ActionType::BOOLEAN, name, "touch", hand } );
        }
        // value is automatic
        actions.push_back( { VRData::ActionType::SCALAR, name, "value", hand } );
      }
      else
      {
        throw std::runtime_error( std::format( "Encountered unknown input type <{}> in file <{}>", input.second.type, m_profilePath ) );
      }
    }
  }
  return actions;
}

VRData::Device Capture::collectDeviceProperties( const vr::TrackedDeviceIndex_t deviceId ) const
{
  VRData::Device device;
  device.m_deviceClass = m_vrSystem->GetTrackedDeviceClass( deviceId );

  // TODO: don't know anything about Prop_Invalid or Prop_ParentContainer
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
        bool valueBool = m_vrSystem->GetBoolTrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_boolProperties[deviceProperty.first] = { std::string( deviceProperty.second ), valueBool };
        }
      }
      else if ( deviceProperty.second.ends_with( "Float" ) )
      {
        float valueFloat = m_vrSystem->GetFloatTrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_floatProperties[deviceProperty.first] = { std::string( deviceProperty.second ), valueFloat };
        }
      }
      else if ( deviceProperty.second.ends_with( "Float_Array" ) )
      {
        getDevicePropertyValueArray<float>( deviceId, deviceProperty, device.m_floatArrayProperties );
      }
      else if ( deviceProperty.second.ends_with( "Int32" ) )
      {
        int32_t valueInt32 = m_vrSystem->GetInt32TrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_int32Properties[deviceProperty.first] = { std::string( deviceProperty.second ), valueInt32 };
        }
      }
      else if ( deviceProperty.second.ends_with( "Int32_Array" ) )
      {
        getDevicePropertyValueArray<int32_t>( deviceId, deviceProperty, device.m_int32ArrayProperties );
      }
      else if ( deviceProperty.second.ends_with( "Matrix34" ) )
      {
        vr::HmdMatrix34_t valueMatrix34 = m_vrSystem->GetMatrix34TrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_matrix34Properties[deviceProperty.first] = { std::string( deviceProperty.second ), valueMatrix34 };
        }
      }
      else if ( deviceProperty.second.ends_with( "Matrix34_Array" ) )
      {
        getDevicePropertyValueArray<vr::HmdMatrix34_t>( deviceId, deviceProperty, device.m_matrix34ArrayProperties );
      }
      else if ( deviceProperty.second.ends_with( "String" ) )
      {
        std::vector<char> buf( vr::k_unMaxPropertyStringSize );
        m_vrSystem->GetStringTrackedDeviceProperty( deviceId, deviceProperty.first, buf.data(), vr::k_unMaxPropertyStringSize, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_stringProperties[deviceProperty.first] = { std::string( deviceProperty.second ), std::string( buf.data() ) };
        }
      }
      else if ( deviceProperty.second.ends_with( "Uint64" ) )
      {
        uint64_t valueUint64 = m_vrSystem->GetUint64TrackedDeviceProperty( deviceId, deviceProperty.first, &error );
        assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_UnknownProperty ) );
        if ( error == vr::TrackedProp_Success )
        {
          device.m_uint64Properties[deviceProperty.first] = { std::string( deviceProperty.second ), valueUint64 };
        }
      }
      else if ( deviceProperty.second.ends_with( "Vector4_Array" ) )
      {
        getDevicePropertyValueArray<float>( deviceId, deviceProperty, device.m_vector4ArrayProperties );
      }
      else
      {
        checkFailure( false, "Encountered unknown property ending at " + std::string( deviceProperty.second ) );
      }
    }
  }
  return device;
}

void Capture::gatherTrackedDevices( float sampleFrequency )
{
  LOGI( "\nEnumerating OpenVR devices...\n" );
  for ( uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i )
  {
    vr::TrackedDeviceIndex_t deviceIndex = static_cast<vr::TrackedDeviceIndex_t>( i );
    auto                     deviceClass = m_vrSystem->GetTrackedDeviceClass( deviceIndex );
    if ( deviceClass != vr::TrackedDeviceClass_Invalid )
    {
      printDeviceInfo( m_vrSystem, deviceIndex, deviceClass );

      switch ( deviceClass )
      {
        case vr::TrackedDeviceClass_HMD:
          checkFailure( deviceIndex == 0, "Encountered HMD Device at deviceIndex " + std::to_string( deviceIndex ) );

          m_trackedDeviceIndices.push_back( deviceIndex );
          m_vrSystem->GetRecommendedRenderTargetSize( &m_hardwareData.m_hmd.m_renderWidth, &m_hardwareData.m_hmd.m_renderHeight );
          {
            auto & p = m_hardwareData.m_hmd.m_proj;
            for ( int i = 0; i < 2; ++i )
            {
              m_vrSystem->GetProjectionRaw( (vr::EVREye)i, &p[i][0], &p[i][1], &p[i][2], &p[i][3] );
            }
          }
          m_hardwareData.m_hmd.m_device = collectDeviceProperties( deviceIndex );
          m_hardwareData.m_captureFrequency =
            ( 0 < sampleFrequency ) ? sampleFrequency : 2.0f * m_hardwareData.m_hmd.m_device.m_floatProperties[vr::Prop_DisplayFrequency_Float].m_value;
          break;

        case vr::TrackedDeviceClass_Controller:
          checkFailure( deviceIndex != 0, "Encountered Controller Device at deviceIndex 0, expecting an HMD there" );

          m_trackedDeviceIndices.push_back( deviceIndex );
          m_hardwareData.m_controllers.push_back( VRData::Controller() );
          VRData::Controller & c = m_hardwareData.m_controllers.back();
          c.m_device             = collectDeviceProperties( deviceIndex );

          auto propertyIt = c.m_device.m_stringProperties.find( vr::ETrackedDeviceProperty::Prop_ControllerType_String );
          checkFailure( ( propertyIt != c.m_device.m_stringProperties.end() ) && !propertyIt->second.m_value.empty(),
                        "Could not find ControllerType information for device " + std::to_string( deviceIndex ) );
          if ( m_profile.controller_type.empty() )
          {
            assert( m_profilePath.empty() );
            std::tie( m_profilePath, m_profile ) = readProfile( deviceIndex, c.m_device );
          }
          else
          {
            checkFailure( m_profile.controller_type == propertyIt->second.m_value,
                          "Encountered unexpected controller type " + propertyIt->second.m_value + ", expected " + m_profile.controller_type );
          }

          vr::ETrackedControllerRole role = m_vrSystem->GetControllerRoleForTrackedDeviceIndex( deviceIndex );
          if ( ( role == vr::TrackedControllerRole_LeftHand ) || ( role == vr::TrackedControllerRole_RightHand ) )
          {
            c.m_hand                            = ( role == vr::TrackedControllerRole_LeftHand ) ? VRData::Hand::LEFT : VRData::Hand::RIGHT;
            std::vector<VRData::Action> actions = collectActions( c.m_hand );
            appendActions( actions );
          }
          else if ( role == vr::TrackedControllerRole_Invalid )
          {
            LOGW( "\nWarning! This controller has an invalid role. Capturing only pose data, no action (e.g. button) inputs!\n\n" );
          }
          else
          {
            LOGW( "\nWarning! Encountered unhandled controller role %s", magic_enum::enum_name( role ).data() );
          }
          break;
      }
    }
  }
  checkFailure( !m_trackedDeviceIndices.empty(), "No tracked devices found" );
  checkFailure( std::ranges::count_if( m_trackedDeviceIndices,
                                       [this]( auto const & device )
                                       { return m_vrSystem->GetControllerRoleForTrackedDeviceIndex( device ) == vr::TrackedControllerRole_LeftHand; } ) <= 1,
                "More than one Left Hand controller tracked" );
  checkFailure( std::ranges::count_if( m_trackedDeviceIndices,
                                       [this]( auto const & device )
                                       { return m_vrSystem->GetControllerRoleForTrackedDeviceIndex( device ) == vr::TrackedControllerRole_RightHand; } ) <= 1,
                "More than one Right Hand controller tracked" );
  if ( std::ranges::find_if( m_hardwareData.m_controllers, []( auto const & controller ) { return controller.m_hand == VRData::Hand::LEFT; } ) ==
       m_hardwareData.m_controllers.end() )
  {
    LOGW( "\nWarning! No left hande controller found. This might cause broken capture data.\n\n" );
  }
  if ( std::ranges::find_if( m_hardwareData.m_controllers, []( auto const & controller ) { return controller.m_hand == VRData::Hand::RIGHT; } ) ==
       m_hardwareData.m_controllers.end() )
  {
    LOGW( "\nWarning! No right hand controller found. This might cause broken capture data.\n\n" );
  }
  m_maxTrackedDeviceIndex = *std::ranges::max_element( m_trackedDeviceIndices );
}

void Capture::generateActionManifestFile( std::string const & path )
{
  std::string controllerType = m_hardwareData.m_controllers.begin()->m_device.m_stringProperties.find( vr::Prop_ControllerType_String )->second.m_value;
  for ( auto const & controller : m_hardwareData.m_controllers )
  {
    checkFailure( controller.m_device.m_stringProperties.find( vr::Prop_ControllerType_String )->second.m_value == controllerType,
                  "Controllers of different type are not supported" );
  }

  ActionManifest actionManifest;
  actionManifest.default_bindings.push_back( { controllerType, "capture_" + controllerType + "_bindings.json" } );

  for ( auto const & a : m_hardwareData.m_actions )
  {
    actionManifest.actions.push_back( { "/actions/capture/in/" + to_string( a ), to_string( a.type ) } );
  }
  m_startAction.isAdded   = addAction( m_startAction.name, actionManifest.actions );
  m_segmentAction.isAdded = addAction( m_segmentAction.name, actionManifest.actions );

  actionManifest.action_sets.push_back( { "/actions/capture", "single" } );

  LOGI( "Writing capture_action_manifest.json file: %s\n", path.c_str() );
  std::ofstream ofile( path );
  if ( !ofile )
  {
    throw std::runtime_error( std::format( "Failed to open for writing: {}", path ) );
  }
  cereal::JSONOutputArchive jsonOutArchive( ofile );
  jsonOutArchive( actionManifest );
}

void Capture::generateControllerBindingsFile( std::filesystem::path const & modulePath )
{
  ControllerBindings controllerBindings;

  controllerBindings.bindings.actions.haptics.push_back( { "/actions/capture/out/left_haptic", "/user/hand/left/output" } );
  controllerBindings.bindings.actions.haptics.push_back( { "/actions/capture/out/right_haptic", "/user/hand/right/output" } );
  controllerBindings.bindings.actions.poses.push_back( { "/actions/capture/in/left_pose", "/user/hand/left/pose/raw" } );
  controllerBindings.bindings.actions.poses.push_back( { "/actions/capture/in/right_pose", "/user/hand/right/pose/raw" } );

  for ( auto const & action : m_hardwareData.m_actions )
  {
    addBindingSource( action, controllerBindings.bindings.actions.sources );
  }
  if ( m_startAction.isAdded )
  {
    addBindingSource( m_startAction.action, controllerBindings.bindings.actions.sources );
  }
  if ( m_segmentAction.isAdded )
  {
    addBindingSource( m_segmentAction.action, controllerBindings.bindings.actions.sources );
  }

  controllerBindings.category        = "steamvr_input";
  controllerBindings.controller_type = m_hardwareData.m_controllers.begin()->m_device.m_stringProperties.find( vr::Prop_ControllerType_String )->second.m_value;
  controllerBindings.name            = "Capture " + controllerBindings.controller_type + " Controller Config";

  std::string fileName = "capture_" + controllerBindings.controller_type + "_bindings.json";
  LOGI( "Writing file: %s\n", ( modulePath / fileName ).string().c_str() );
  std::ofstream ofile( fileName );
  if ( !ofile )
  {
    throw std::runtime_error( std::format( "Failed to open for writing: {}", fileName ) );
  }

  cereal::JSONOutputArchive jsonOutArchive( ofile );
  jsonOutArchive( controllerBindings );
}

vr::VRActionHandle_t Capture::getActionHandle( std::string const & actionString ) const
{
  // assemble input path of form /actions/capture/in/actionString
  // these need to match the definitions in the controller json files
  vr::VRActionHandle_t actionHandle = vr::k_ulInvalidActionHandle;
  auto                 error        = m_vrInput->GetActionHandle( ( "/actions/capture/in/" + actionString ).c_str(), &actionHandle );
  if ( error != vr::EVRInputError::VRInputError_None )
  {
    LOGE( "\tError getting action on %s: %i / %s\n", actionString.c_str(), error, magic_enum::enum_name( error ).data() );
  }
  return actionHandle;
}

std::vector<VRData::DevicePose> Capture::getDevicePoses()
{
  std::vector<vr::TrackedDevicePose_t> trackedDevicePose( m_maxTrackedDeviceIndex + 1 );
  m_vrSystem->GetDeviceToAbsoluteTrackingPose( vr::TrackingUniverseRawAndUncalibrated, 0, trackedDevicePose.data(), (uint32_t)trackedDevicePose.size() );

  std::vector<VRData::DevicePose> devicePoses;
  for ( uint32_t i = 0; i < m_trackedDeviceIndices.size(); ++i )
  {
    const auto & pose = trackedDevicePose[m_trackedDeviceIndices[i]];
    auto         pos  = get_position( pose.mDeviceToAbsoluteTracking );
    auto         rot  = get_rotation( pose.mDeviceToAbsoluteTracking );
    devicePoses.push_back( { { pos.v[0], pos.v[1], pos.v[2] }, { rot.w, rot.x, rot.y, rot.z } } );
  }
  return devicePoses;
}

template <typename ValueType>
struct PropertyTag
{
  static constexpr vr::PropertyTypeTag_t value = vr::k_unInvalidPropertyTag;
};

template <>
struct PropertyTag<float>
{
  static constexpr vr::PropertyTypeTag_t value = vr::k_unFloatPropertyTag;
};

template <>
struct PropertyTag<int32_t>
{
  static constexpr vr::PropertyTypeTag_t value = vr::k_unInt32PropertyTag;
};

template <>
struct PropertyTag<vr::HmdMatrix34_t>
{
  static constexpr vr::PropertyTypeTag_t value = vr::k_unHmdMatrix34PropertyTag;
};

template <>
struct PropertyTag<vr::HmdVector4_t>
{
  static constexpr vr::PropertyTypeTag_t value = vr::k_unHmdVector4PropertyTag;
};

template <typename ValueType>
void Capture::getDevicePropertyValueArray( vr::TrackedDeviceIndex_t                                                   deviceId,
                                           std::pair<vr::ETrackedDeviceProperty, std::string_view> const &            deviceProperty,
                                           std::map<uint32_t, VRData::Device::PropertyData<std::vector<ValueType>>> & properties ) const
{
  vr::ETrackedPropertyError error;
  std::vector<ValueType>    values( 2 );
  uint32_t                  byteSize = m_vrSystem->GetArrayTrackedDeviceProperty(
    deviceId, deviceProperty.first, PropertyTag<ValueType>::value, values.data(), static_cast<uint32_t>( values.size() ) * sizeof( ValueType ), &error );
  if ( error != vr::TrackedProp_UnknownProperty )
  {
    assert( ( error == vr::TrackedProp_Success ) || ( error == vr::TrackedProp_BufferTooSmall ) );
    while ( error == vr::TrackedProp_BufferTooSmall )
    {
      values.resize( 2 * values.size() );
      byteSize = m_vrSystem->GetArrayTrackedDeviceProperty(
        deviceId, deviceProperty.first, PropertyTag<ValueType>::value, values.data(), static_cast<uint32_t>( values.size() ) * sizeof( ValueType ), &error );
    }
    checkFailure( error, m_vrSystem );
    assert( byteSize % sizeof( ValueType ) == 0 );
    values.resize( byteSize / sizeof( ValueType ) );
    properties[deviceProperty.first] = { std::string( deviceProperty.second ), values };
  }
}

void Capture::initActiveSectionSet()
{
  TCHAR                 path[MAX_PATH * 2];
  std::filesystem::path modulePath;
  if ( GetModuleFileName( NULL, path, MAX_PATH ) )
  {
    modulePath = std::string( path, strrchr( path, '\\' ) );
  }
  else
  {
    LOGE( "GetModuleFileName failed\n" );
  }

  std::string actionManifestPath = ( modulePath / "capture_action_manifest.json" ).string();
  generateActionManifestFile( actionManifestPath );

  LOGI( "\nSetting action manifest file: %s\n", actionManifestPath.c_str() );
  checkFailure( m_vrInput->SetActionManifestPath( actionManifestPath.c_str() ), "Failed", "Success" );

  LOGI( "\nGetting action set: %s\n", actionManifestPath.c_str() );
  checkFailure( m_vrInput->GetActionSetHandle( "/actions/capture", &m_vrActiveActionSet.ulActionSet ), "Failed", "Success" );

  generateControllerBindingsFile( modulePath );
}

void Capture::initChaperone()
{
  vr::IVRCompositor *     vrCompositor        = vr::VRCompositor();
  vr::IVRChaperoneSetup * vrChaperoneSetup    = vr::VRChaperoneSetup();
  m_hardwareData.m_chaperone.m_universeOrigin = vrCompositor->GetTrackingSpace();
  checkFailure( vrChaperoneSetup->GetWorkingPlayAreaSize( &m_hardwareData.m_chaperone.m_playArea[0], &m_hardwareData.m_chaperone.m_playArea[1] ) );
  if ( vrCompositor->GetTrackingSpace() == vr::ETrackingUniverseOrigin::TrackingUniverseSeated )
  {
    checkFailure( vrChaperoneSetup->GetWorkingSeatedZeroPoseToRawTrackingPose( (vr::HmdMatrix34_t *)&m_hardwareData.m_chaperone.m_origin ) );
  }
  else
  {
    checkFailure( vrChaperoneSetup->GetWorkingStandingZeroPoseToRawTrackingPose( (vr::HmdMatrix34_t *)&m_hardwareData.m_chaperone.m_origin ) );
  }
}

void Capture::initOverlays( bool notify )
{
  if ( notify )
  {
    vr::IVROverlay *      overlay = vr::VROverlay();
    vr::VROverlayHandle_t thumbnailHandle;
    vr::EVROverlayError   overlayError = overlay->CreateDashboardOverlay( "VCR", "VCR", &m_vrOverlayHandle, &thumbnailHandle );
    checkFailure( overlayError, "Failed to create Overlay" );
    checkFailure( overlay->SetOverlayWidthInMeters( m_vrOverlayHandle, 2.5f ), "Failed to set Overlay width in meters" );

    vr::EVRInitError initError;
    m_vrNotifications = (vr::IVRNotifications *)vr::VR_GetGenericInterface( vr::IVRNotifications_Version, &initError );
    checkFailure( initError, "Failed to get Notifications interface" );
  }
}

void Capture::initVRSystem()
{
  vr::EVRInitError initError;
  m_vrSystem = vr::VR_Init( &initError, vr::VRApplication_Background );
  checkFailure( initError, "Failed to init VRSystem" );
  m_vrInput = vr::VRInput();
}

void Capture::printConfiguration() const
{
  std::stringstream ss;

  size_t numControllers = m_hardwareData.m_controllers.size();
  size_t numDevices     = numControllers + 1;  // controllers + HMD

  ss << "\nTracking " << numDevices << " devices: \n";
  printConfiguration( ss, m_hardwareData.m_hmd.m_device );
  for ( auto & c : m_hardwareData.m_controllers )
  {
    printConfiguration( ss, c.m_device );
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
  ss << "\tSample Frequency [Hz]: \t" << m_hardwareData.m_captureFrequency << "\n";
  ss << "\n\n";

  LOGI( ss.str().c_str() );

  if ( m_hardwareData.m_actions.empty() )
  {
    LOGW( "\nWarning! Not recording any actions for attached controllers!\n\n" );
  }
}

void Capture::printConfiguration( std::stringstream & ss, VRData::Device const & device ) const
{
  std::map<uint32_t, VRData::Device::PropertyData<std::string>>::const_iterator modelPropertyIt = device.m_stringProperties.find( vr::Prop_ModelNumber_String );
  assert( modelPropertyIt != device.m_stringProperties.end() );
  std::map<uint32_t, VRData::Device::PropertyData<std::string>>::const_iterator serialPropertyIt =
    device.m_stringProperties.find( vr::Prop_SerialNumber_String );
  assert( serialPropertyIt != device.m_stringProperties.end() );

  assert( ( device.m_deviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_HMD ) ||
          ( device.m_deviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_Controller ) );
  ss << "\t" << ( ( device.m_deviceClass == vr::ETrackedDeviceClass::TrackedDeviceClass_HMD ) ? "HMD" : "Controller" ) << ": "
     << modelPropertyIt->second.m_value << " / " << serialPropertyIt->second.m_value << std::endl;
}

std::pair<std::string, Profile> Capture::readProfile( vr::TrackedDeviceIndex_t deviceIndex, VRData::Device const & device ) const
{
  auto propertyIt = device.m_stringProperties.find( vr::ETrackedDeviceProperty::Prop_InputProfilePath_String );
  checkFailure( propertyIt != device.m_stringProperties.end(), "Could not find InputProfilePath information for device " + std::to_string( deviceIndex ) );

  auto     vrResources = vr::VRResources();
  uint32_t len         = vrResources->GetResourceFullPath( propertyIt->second.m_value.c_str(), "", nullptr, 0 );
  assert( 0 < len );
  std::string profilePath;
  profilePath.resize( len );
  len = vrResources->GetResourceFullPath( propertyIt->second.m_value.c_str(), "", profilePath.data(), len );
  assert( len == profilePath.length() );

  std::ifstream resourcesStream( profilePath );
  if ( !resourcesStream )
  {
    throw std::runtime_error( std::format( "Failed to open file <{}> for reading", profilePath ) );
  }

  cereal::JSONInputArchive resourcesArchive( resourcesStream );

  Profile profile;
  resourcesArchive( profile );

  return { std::move( profilePath ), std::move( profile ) };
}

std::tuple<VRData::Action, std::string, vr::VRActionHandle_t> Capture::setupAction( std::vector<std::string> const & actionDescription,
                                                                                    std::string const &              actionKind )
{
  VRData::Action       action;
  std::string          actionString;
  vr::VRActionHandle_t actionHandle = vr::k_ulInvalidActionHandle;

  if ( !actionDescription.empty() )
  {
    assert( 3 == actionDescription.size() );

    VRData::Hand hand =
      ( actionDescription[0] == "left" ) ? VRData::Hand::LEFT : ( ( actionDescription[0] == "right" ) ? VRData::Hand::RIGHT : VRData::Hand::NONE );
    action       = { VRData::ActionType::BOOLEAN, actionDescription[1], actionDescription[2], hand };
    actionString = to_string( action );
    LOGI( "\nSet up %s action... %s\n", actionKind.c_str(), actionString.c_str() );

    if ( hand != VRData::Hand::NONE )
    {
      actionHandle = getActionHandle( actionString );
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
  return { action, actionString, actionHandle };
}

void Capture::writeHardwareData()
{
  std::string hardwarePath = std::filesystem::absolute( "..\\tape\\hardware.json" ).string();
  LOGI( "Writing hardware file: %s\n", hardwarePath.c_str() );
  std::ofstream ofile( "..\\tape\\hardware.json", std::ios::binary );
  if ( !ofile )
  {
    throw std::runtime_error( std::format( "Failed to open for writing: {}", hardwarePath ) );
  }
  cereal::JSONOutputArchive jsonOutArchive( ofile );
  jsonOutArchive( m_hardwareData );
}
