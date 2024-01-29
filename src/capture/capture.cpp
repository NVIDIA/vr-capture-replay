/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "capture.h"

#include "shared/logging.h"

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

std::shared_ptr<Args> args;

vr::IVRSystem *         vrSystem;
vr::IVRInput *          vrInput;
vr::VRActiveActionSet_t activeActionSet;

VRData::HardwareData hardwareData;
VRData::TrackingData trackingData;

// index of our devices in tracked device position
// devices can be mixed with non-tracked stuff like lighthouses
std::vector<uint32_t> deviceIndices;

// highest index of a device we're capturing
// needed to ask OpenVR for the right amount of data
uint32_t maxDeviceIndex;

// interface and handles for notifications
vr::IVRNotifications * vrNotifications;
vr::VROverlayHandle_t  overlayHandle          = vr::k_ulOverlayHandleInvalid;
vr::VROverlayHandle_t  overlayThumbnailHandle = vr::k_ulOverlayHandleInvalid;

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

int main( int argc, char * argv[] )
{
  try
  {
    args = std::make_shared<Args>( Args{ { argc, argv } } );
  }
  catch ( std::exception & e )
  {
    std::cerr << "Error parsing command line: " << e.what() << '\n';
    exit( 1 );
  }

  std::string logPath = std::filesystem::absolute( "..\\logs" ).string();
  if ( !std::filesystem::exists( logPath ) )
  {
    logPath.clear();
  }
  initLog( logPath );
  logCommandLine( argc, argv );

  try
  {
    initVR();
    initChaperone();
    initOverlay();
  }
  catch ( std::exception & e )
  {
    LOGE( "Error initialzing: %s", e.what() );
  }

  VRData::Controller              leftController, rightController;
  uint32_t                        leftIndex, rightIndex;
  bool                            leftValid{ false };
  bool                            rightValid{ false };
  std::vector<VRData::Controller> tmpControllers;
  std::vector<uint32_t>           tmpIndices;

  LOGI( "\nEnumerating OpenVR devices...\n" );
  for ( uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i )
  {
    vr::TrackedDeviceIndex_t deviceId = (vr::TrackedDeviceIndex_t)i;

    auto deviceClass = vrSystem->GetTrackedDeviceClass( deviceId );
    if ( deviceClass == vr::TrackedDeviceClass_Invalid )
    {
      continue;
    }

    printDeviceInfo( deviceId );

    std::shared_ptr<char[]> buf( new char[vr::k_unMaxPropertyStringSize] );
    if ( deviceClass == vr::TrackedDeviceClass_HMD )
    {
      uint32_t width, height;
      vrSystem->GetRecommendedRenderTargetSize( &width, &height );
      hardwareData.m_hmd.m_renderWidth  = width;
      hardwareData.m_hmd.m_renderHeight = height;

      auto & p = hardwareData.m_hmd.m_proj;
      for ( int i = 0; i < 2; ++i )
      {
        vrSystem->GetProjectionRaw( (vr::EVREye)i, &p[i][0], &p[i][1], &p[i][2], &p[i][3] );
      }
    }
    else if ( deviceClass != vr::TrackedDeviceClass_Controller )
    {
      // not an HMD or a controller, skip
      // TODO: add support for generic trackers?
      continue;
    }

    VRData::Device device;
    device.m_deviceClass = deviceClass;

    collectDeviceProperties( deviceId, device );

    if ( deviceClass == vr::TrackedDeviceClass_HMD )
    {
      hardwareData.m_hmd.m_device = device;
      if ( args->sampleFreq > 0 )
      {
        hardwareData.m_captureFrequency = args->sampleFreq;
      }
      else
      {
        hardwareData.m_captureFrequency = 2.0f * hardwareData.m_hmd.m_device.m_floatProperties[vr::Prop_DisplayFrequency_Float].m_value;
      }
    }
    else
    {
      VRData::Controller c;
      c.m_device                      = device;
      vr::ETrackedControllerRole role = vrSystem->GetControllerRoleForTrackedDeviceIndex( deviceId );
      if ( role == vr::TrackedControllerRole_LeftHand )
      {
        c.m_hand = VRData::Hand::LEFT;
        setupActions( c.m_device.m_stringProperties[vr::Prop_ModelNumber_String].m_value, VRData::Hand::LEFT );
        leftController = c;
        leftIndex      = i;
        leftValid      = true;
      }
      else if ( role == vr::TrackedControllerRole_RightHand )
      {
        c.m_hand = VRData::Hand::RIGHT;
        setupActions( c.m_device.m_stringProperties[vr::Prop_ModelNumber_String].m_value, VRData::Hand::RIGHT );
        rightController = c;
        rightIndex      = i;
        rightValid      = true;
      }
      else
      {
        if ( role == vr::TrackedControllerRole_Invalid )
        {
          LOGW( "\nWarning! This device has an invalid role. Capturing only pose data, no action (e.g. button) inputs!\n\n" );
        }
        tmpControllers.push_back( c );
        tmpIndices.push_back( i );
      }
    }
    maxDeviceIndex = i;
  }

  // assemble indices and devices so that we have HMD, LCTRL, RCTRL, ....
  deviceIndices.push_back( 0 );  // HMD is always 0 and separate in HardwareData
  if ( leftValid )
  {
    deviceIndices.push_back( leftIndex );                    // left controller index
    hardwareData.m_controllers.push_back( leftController );  // and controller
  }
  else
  {
    LOGW( "\nWarning! No left controller found. This might cause broken capture data.\n\n" );
  }
  if ( rightValid )
  {
    deviceIndices.push_back( rightIndex );                    // right controller index
    hardwareData.m_controllers.push_back( rightController );  // and controller
  }
  else
  {
    LOGW( "\nWarning! No right controller found. This might cause broken capture data.\n\n" );
  }

  // insert any other tracked controllers
  deviceIndices.insert( deviceIndices.end(), tmpIndices.begin(), tmpIndices.end() );
  hardwareData.m_controllers.insert( hardwareData.m_controllers.end(), tmpControllers.begin(), tmpControllers.end() );

  startActionString = setupStartAction();
  setupSegmentationAction();

  printConfiguration();

  // wait for the start action, if configured
  if ( startActionHandle != vr::k_ulInvalidActionHandle )
  {
    waitForStartAction();
  }

  auto start = std::chrono::steady_clock::now();
  try
  {
    // create hardware data file, contains the pose the driver defaults to
    {
      // set up initial poses
      std::vector<VRData::DevicePose> devicePoses = getDevicePoses();
      hardwareData.m_hmd.m_device.m_initialPose   = devicePoses[0];
      for ( int i = 0; i < hardwareData.m_controllers.size(); ++i )
      {
        // NOTE: offset +1 between controllers and device poses (HMD is pose 0)
        hardwareData.m_controllers[i].m_device.m_initialPose = devicePoses[i + 1];
      }
      writeHardwareData();
    }

    using Duration     = std::chrono::duration<double, std::nano>;
    Duration sleeptime = std::chrono::seconds( 1 ) / ( hardwareData.m_captureFrequency );

    LOGI( "\n\nRecording events, press ESC to stop, SPACE to write segment...\n\n" );

    notifyHMD( "Starting VCR capture" );

    while ( true )
    {
      if ( GetAsyncKeyState( VK_ESCAPE ) & 0x8000 )
      {
        // write and quit
        LOGI( "Generating final segment\n" );
        writeTrackingData();
        break;
      }
      if ( GetAsyncKeyState( VK_SPACE ) & 0x8000 )
      {
        // write and re-init, continue recording
        LOGI( "Generating segment\n" );
        notifyHMD( "Generating segment" );
        writeTrackingData();
        start        = std::chrono::steady_clock::now();
        trackingData = VRData::TrackingData{};

        while ( GetAsyncKeyState( VK_SPACE ) & 0x8000 )
        {
          // wait for key to be lifted again
        }
      }

      auto t = std::chrono::duration<float>( std::chrono::steady_clock::now() - start ).count();

      VRData::TrackingItem trackingItem;
      trackingItem.time = t;

      // record tracking data for HMD and controllers
      std::vector<VRData::DevicePose> devicePoses = getDevicePoses();  // returns HMD, CTR0, CTR1, ....
      // first item into HMD
      auto it                = devicePoses.begin();
      trackingItem.m_hmdPose = *it;
      // all other items into controller poses
      ++it;
      trackingItem.m_controllerPoses.assign( it, devicePoses.end() );

      // update input action state
      vrInput->UpdateActionState( &activeActionSet, sizeof( activeActionSet ), 1 );

      // create segment if configured button was pressed
      if ( segmentActionHandle != vr::k_ulInvalidActionHandle )
      {
        // TODO: allow more than DIG input?
        vr::InputDigitalActionData_t data{ 0 };
        vrInput->GetDigitalActionData( segmentActionHandle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
        if ( data.bState )
        {
          // write and re-init, continue recording
          LOGI( "Generating segment\n" );
          notifyHMD( "Generating segment" );
          writeTrackingData();
          start        = std::chrono::steady_clock::now();
          trackingData = VRData::TrackingData{};
          while ( data.bState )
          {
            vrInput->UpdateActionState( &activeActionSet, sizeof( activeActionSet ), 1 );
            vrInput->GetDigitalActionData( segmentActionHandle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
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
        const VRData::Action & action = hardwareData.m_actions[index];
        VRData::ActionData     actionData{ index };
        bool                   changed{ false };

        if ( VRData::ActionType( action.type ) == VRData::ActionType::DIG )
        {
          vr::InputDigitalActionData_t data{ 0 };
          vrInput->GetDigitalActionData( handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
          changed           = data.bChanged;
          actionData.val[0] = data.bState;
        }
        else if ( VRData::ActionType( action.type ) == VRData::ActionType::VEC1 )
        {
          vr::InputAnalogActionData_t data{ 0 };
          vrInput->GetAnalogActionData( handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
          changed           = data.deltaX != 0.0f;
          actionData.val[0] = data.x;
        }
        else if ( VRData::ActionType( action.type ) == VRData::ActionType::VEC2 )
        {
          vr::InputAnalogActionData_t data{ 0 };
          vrInput->GetAnalogActionData( handle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
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

      trackingData.m_trackingItems.push_back( trackingItem );
      std::this_thread::sleep_for( sleeptime );
    }
  }
  catch ( std::exception & e )
  {
    LOGE( "\n\nError: %s\n\n", e.what() );
  }
  if ( !args->noNotifyHMD )
  {
    notifyHMD( "Shutting down VCR capture" );
    // sleep to make sure the notification is shown
    std::this_thread::sleep_for( std::chrono::milliseconds( args->notifyTime ) );
  }
  LOGI( "Shutting down...\n" );
  vr::VR_Shutdown();

  auto t = std::chrono::duration<float>( std::chrono::steady_clock::now() - start ).count();
  LOGE( "\n\nDone, recorded %.1f seconds\n", t );
  return 0;
}

void initVR()
{
  vr::HmdError hmdErr;
  vrSystem = vr::VR_Init( &hmdErr, vr::VRApplication_Background );
  if ( hmdErr != vr::EVRInitError::VRInitError_None )
  {
    LOGE( "Failed to initialize: %s %s \n\n", vr::VR_GetVRInitErrorAsSymbol( hmdErr ), vr::VR_GetVRInitErrorAsEnglishDescription( hmdErr ) );
    exit( hmdErr );
  }

  vr::EVRInputError inputErr;
  TCHAR             path[MAX_PATH * 2];
  if ( GetModuleFileName( NULL, path, MAX_PATH ) )
  {
    strcpy_s( strrchr( path, '\\' ), MAX_PATH, "\\record_controller_actions.json" );
  }
  else
  {
    LOGE( "GetModuleFileName failed\n" );
  }
  LOGI( "\nSetting action manifest file: %s\n", path );
  inputErr = vr::VRInput()->SetActionManifestPath( path );
  if ( inputErr == vr::VRInputError_None )
  {
    LOGI( "\tSuccess\n" )
  }
  else
  {
    LOGE( "\tError loading manifest file!\n" );
  }

  vrInput = vr::VRInput();

  activeActionSet = { 0 };
  inputErr        = vrInput->GetActionSetHandle( "/actions/record", &activeActionSet.ulActionSet );
}

void initChaperone()
{
  // collect chaperone / universe information
  auto & chap           = hardwareData.m_chaperone;
  chap.m_universeOrigin = vr::VRCompositor()->GetTrackingSpace();
  vr::VRChaperoneSetup()->GetWorkingPlayAreaSize( &chap.m_playArea[0], &chap.m_playArea[1] );
  if ( vr::VRCompositor()->GetTrackingSpace() == vr::ETrackingUniverseOrigin::TrackingUniverseSeated )
  {
    vr::VRChaperoneSetup()->GetWorkingSeatedZeroPoseToRawTrackingPose( (vr::HmdMatrix34_t *)&chap.m_origin );
  }
  else
  {
    vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose( (vr::HmdMatrix34_t *)&chap.m_origin );
  }
}

void initOverlay()
{
  if ( args->noNotifyHMD )
  {
    return;
  }

  vr::VROverlayError overlayError = vr::VROverlay()->CreateDashboardOverlay( "VCR", "VCR", &overlayHandle, &overlayThumbnailHandle );
  if ( overlayError != vr::VROverlayError_None )
  {
    throw std::runtime_error( std::string( "Failed to create Overlay: " + std::string( vr::VROverlay()->GetOverlayErrorNameFromEnum( overlayError ) ) ) );
  }
  vr::VROverlay()->SetOverlayWidthInMeters( overlayHandle, 2.5f );

  vr::EVRInitError ierr;
  vrNotifications = (vr::IVRNotifications *)vr::VR_GetGenericInterface( vr::IVRNotifications_Version, &ierr );
  if ( ierr != vr::VRInitError_None )
  {
    LOGE( "Error while getting IVRNotifications interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription( ierr ) );
    vrNotifications = nullptr;
  }
}

void printDeviceInfo( const vr::TrackedDeviceIndex_t deviceId )
{
  auto                      deviceClass = vrSystem->GetTrackedDeviceClass( deviceId );
  std::shared_ptr<char[]>   buf( new char[vr::k_unMaxPropertyStringSize] );
  vr::ETrackedPropertyError error;

  std::stringstream ss;
  ss << "\nDevice " << deviceId << " Class " << deviceClass << " / " << magic_enum::enum_name( deviceClass ) << "\n";
  vrSystem->GetStringTrackedDeviceProperty( deviceId, vr::Prop_ModelNumber_String, buf.get(), vr::k_unMaxPropertyStringSize, &error );
  ss << "\t     Model: " << buf.get() << "\n";
  vrSystem->GetStringTrackedDeviceProperty( deviceId, vr::Prop_SerialNumber_String, buf.get(), vr::k_unMaxPropertyStringSize, &error );
  ss << "\t    Serial: " << buf.get() << "\n";
  vrSystem->GetStringTrackedDeviceProperty( deviceId, vr::Prop_TrackingSystemName_String, buf.get(), vr::k_unMaxPropertyStringSize, &error );
  ss << "\t  Tracking: " << buf.get() << "\n";

  if ( deviceClass == vr::TrackedDeviceClass_Controller )
  {
    vr::ETrackedControllerRole role = vrSystem->GetControllerRoleForTrackedDeviceIndex( deviceId );
    ss << "\t      Role: " << magic_enum::enum_name( role ) << "\n";
  }

  LOGI( "%s", ss.str().c_str() );
}

void collectDeviceProperties( const vr::TrackedDeviceIndex_t deviceId, VRData::Device & device )
{
  vr::ETrackedPropertyError error;

  for ( uint32_t p = 1; p < 20000; ++p )
  {  // Can't take Prop_TrackedDeviceProperty_Max here, takes too long
    vr::ETrackedDeviceProperty propertyId = (vr::ETrackedDeviceProperty)p;

    // skip properties that should not be collected/persisted
    std::vector<vr::ETrackedDeviceProperty> skip{
      vr::ETrackedDeviceProperty::Prop_GraphicsAdapterLuid_Uint64,
    };
    if ( std::ranges::find( skip, propertyId ) != skip.end() )
    {
      continue;
    }

    {  // string
      std::shared_ptr<char[]> buf( new char[vr::k_unMaxPropertyStringSize] );
      vrSystem->GetStringTrackedDeviceProperty( deviceId, propertyId, buf.get(), vr::k_unMaxPropertyStringSize, &error );
      if ( error == vr::TrackedProp_Success )
      {
        device.m_stringProperties[propertyId] = { std::string( magic_enum::enum_name( propertyId ) ), std::string( buf.get() ) };
        continue;
      }
    }
    {  // int32
      int32_t valueInt32 = vrSystem->GetInt32TrackedDeviceProperty( deviceId, propertyId, &error );
      if ( error == vr::TrackedProp_Success )
      {
        device.m_int32Properties[propertyId] = { std::string( magic_enum::enum_name( propertyId ) ), valueInt32 };
        continue;
      }
    }
    {  // uint64
      uint64_t valueUint64 = vrSystem->GetUint64TrackedDeviceProperty( deviceId, propertyId, &error );
      if ( error == vr::TrackedProp_Success )
      {
        device.m_uint64Properties[propertyId] = { std::string( magic_enum::enum_name( propertyId ) ), valueUint64 };
        continue;
      }
    }
    {  // bool
      bool valueBool = vrSystem->GetBoolTrackedDeviceProperty( deviceId, propertyId, &error );
      if ( error == vr::TrackedProp_Success )
      {
        device.m_boolProperties[propertyId] = { std::string( magic_enum::enum_name( propertyId ) ), valueBool };
        continue;
      }
    }
    {  // float
      float valueFloat = vrSystem->GetFloatTrackedDeviceProperty( deviceId, propertyId, &error );
      if ( error == vr::TrackedProp_Success )
      {
        device.m_floatProperties[propertyId] = { std::string( magic_enum::enum_name( propertyId ) ), valueFloat };
        continue;
      }
    }
  }
}

vr::EVRInputError getActionHandle( VRData::Action action, vr::VRActionHandle_t & a )
{
  // assemble input path of form /actions/record/in/left_system_click
  // these need to match the definitions in the controller json files
  std::string prefix = "/actions/record/in/";
  std::string h      = ( action.hand == VRData::Hand::LEFT ) ? "left" : "right";
  std::string path   = prefix + h + "_" + action.name + "_" + action.input;
  return vrInput->GetActionHandle( path.c_str(), &a );
}

void setupActions( std::string modelNumber, VRData::Hand hand )
{
  std::vector<VRData::Action> actions;
  if ( ( modelNumber.find( "VIVE" ) != std::string::npos ) || ( modelNumber.find( "Vive" ) != std::string::npos ) )
  {
    actions = { { VRData::ActionType::DIG, "system", "click", hand },
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
    actions = { { VRData::ActionType::DIG, "system", "click", hand },   { VRData::ActionType::DIG, "trigger", "click", hand },
                { VRData::ActionType::DIG, "trigger", "touch", hand },  { VRData::ActionType::VEC1, "trigger", "value", hand },
                { VRData::ActionType::VEC1, "grip", "value", hand },    { VRData::ActionType::DIG, "grip", "touch", hand },
                { VRData::ActionType::DIG, "joystick", "click", hand }, { VRData::ActionType::DIG, "joystick", "touch", hand },
                { VRData::ActionType::VEC2, "joystick", "value", hand } };
    if ( hand == VRData::Hand::RIGHT )
    {
      actions.push_back( { VRData::ActionType::DIG, "a", "click", hand } );
      actions.push_back( { VRData::ActionType::DIG, "b", "click", hand } );
    }
    else if ( hand == VRData::Hand::LEFT )
    {
      actions.push_back( { VRData::ActionType::DIG, "x", "click", hand } );
      actions.push_back( { VRData::ActionType::DIG, "y", "click", hand } );
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
  std::stringstream ss;

  // enumerate actions, get handles for them
  size_t i = 0;
  for ( const auto & a : actions )
  {
    hardwareData.m_actions.push_back( a );
    size_t                 index  = hardwareData.m_actions.size() - 1;
    const VRData::Action & action = hardwareData.m_actions[index];

    vr::VRActionHandle_t a;
    vr::EVRInputError    err = getActionHandle( action, a );

    ss << " " << action.name + "_" + action.input;
    if ( ++i >= 3 )
    {
      i = 0;
      ss << "\n\t           ";
    }

    if ( err == vr::EVRInputError::VRInputError_None )
    {
      actionCaptureDataVec.push_back( { a, index } );
    }
    else
    {
      ss << "Error getting handle for action " << action.name << "_" << action.input << ": " << err << " / " << magic_enum::enum_name( err ) << "\n";
    }
  }
  ss << "\n";

  LOGI( "%s", ss.str().c_str() );
}

std::string setupStartAction()
{
  const auto & a = args->startAction;
  if ( a.empty() )
  {
    return {};
  }

  std::string startActionString = a[0] + "_" + a[1] + "_" + a[2];

  LOGI( "\nSet up start action... %s\n", startActionString.c_str() );
  VRData::Hand hand;
  if ( a[0] == "left" )
  {
    hand = VRData::Hand::LEFT;
  }
  else if ( a[0] == "right" )
  {
    hand = VRData::Hand::RIGHT;
  }
  else
  {
    LOGE( "\tInvalid hand string %s\n", a[0].c_str() );
    return {};
  }

  VRData::Action startAction = { VRData::ActionType::DIG, a[1], a[2], hand };

  vr::EVRInputError err = getActionHandle( startAction, startActionHandle );
  if ( err == vr::EVRInputError::VRInputError_None )
  {
    // TODO: make this smarter, maybe the get returns the path?
    LOGI( "\tSuccess, use %s to start recording\n", startActionString.c_str() );
  }
  else
  {
    LOGE( "\tError getting handle for start action %s_%s: %i / %s\n",
          startAction.name.c_str(),
          startAction.input.c_str(),
          err,
          std::string( magic_enum::enum_name( err ) ).c_str() );
  }
  return startActionString;
}

void waitForStartAction()
{
  LOGI( "Ready - use %s to start recording\n", startActionString.c_str() );
  notifyHMD( "Ready - use " + startActionString + " to start recording", 3000 );
  while ( true )
  {
    vrInput->UpdateActionState( &activeActionSet, sizeof( activeActionSet ), 1 );
    vr::InputDigitalActionData_t data{ 0 };
    vrInput->GetDigitalActionData( startActionHandle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
    if ( data.bState )
    {
      // wait for action to end
      while ( data.bState )
      {
        vrInput->UpdateActionState( &activeActionSet, sizeof( activeActionSet ), 1 );
        vrInput->GetDigitalActionData( segmentActionHandle, &data, sizeof( data ), vr::k_ulInvalidInputValueHandle );
      }
      break;
    }
    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
  }
}

void setupSegmentationAction()
{
  const auto & a = args->segmentAction;
  if ( a.empty() )
  {
    return;
  }

  LOGI( "\nSet up segmenting action... %s_%s_%s\n", a[0].c_str(), a[1].c_str(), a[2].c_str() );
  VRData::Hand hand;
  if ( a[0] == "left" )
  {
    hand = VRData::Hand::LEFT;
  }
  else if ( a[0] == "right" )
  {
    hand = VRData::Hand::RIGHT;
  }
  else
  {
    LOGE( "\tInvalid hand string %s\n", a[0].c_str() );
    return;
  }

  VRData::Action segmentAction = { VRData::ActionType::DIG, a[1], a[2], hand };

  vr::EVRInputError err = getActionHandle( segmentAction, segmentActionHandle );
  if ( err == vr::EVRInputError::VRInputError_None )
  {
    // TODO: make this smarter, maybe the get returns the path
    LOGI( "\tSuccess, use %s_%s_%s to segment recording\n", a[0].c_str(), a[1].c_str(), a[2].c_str() );
  }
  else
  {
    LOGE( "\tError getting handle for segmenting action %s_%s: %i / %s\n",
          segmentAction.name.c_str(),
          segmentAction.input.c_str(),
          err,
          std::string( magic_enum::enum_name( err ) ).c_str() );
  }
}

void printConfiguration()
{
  std::stringstream ss;

  size_t numControllers = hardwareData.m_controllers.size();
  size_t numDevices     = numControllers + 1;  // controllers + HMD

  ss << "\nTracking " << numDevices << " devices: \n";
  auto &   dev    = hardwareData.m_hmd.m_device;
  uint32_t model  = vr::Prop_ModelNumber_String;
  uint32_t serial = vr::Prop_SerialNumber_String;
  ss << "\tHMD: " << dev.m_stringProperties[model].m_value << " / " << dev.m_stringProperties[serial].m_value << std::endl;
  for ( auto & c : hardwareData.m_controllers )
  {
    auto & dev = c.m_device;
    ss << "\tController: " << dev.m_stringProperties[model].m_value << " / " << dev.m_stringProperties[serial].m_value << "  ";
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

void writeHardwareData()
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

void writeTrackingData()
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

void notifyHMD( std::string message, uint32_t overrideTime )
{
  if ( args->noNotifyHMD || !vrNotifications )
  {
    return;
  }

  vr::VRNotificationId       notificationId;
  vr::NotificationBitmap_t * messageIconPtr = nullptr;

  vr::EVRNotificationError nerr = vrNotifications->CreateNotification(
    overlayHandle, 42, vr::EVRNotificationType_Transient, message.c_str(), vr::EVRNotificationStyle_None, messageIconPtr, &notificationId );
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

    uint32_t    time = overrideTime > 0 ? overrideTime : args->notifyTime;
    std::thread t( killNotification, notificationId, time );
    t.detach();
  }
  else
  {
    LOGE( "Error creating notification: %i", nerr );
  }
}

auto get_position( vr::HmdMatrix34_t matrix )
{
  vr::HmdVector3_t vector;
  vector.v[0] = matrix.m[0][3];
  vector.v[1] = matrix.m[1][3];
  vector.v[2] = matrix.m[2][3];
  return vector;
};

auto get_rotation( vr::HmdMatrix34_t matrix )
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

std::vector<VRData::DevicePose> getDevicePoses()
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