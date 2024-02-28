/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "thirdparty/openvr/headers/openvr.h"

#pragma warning( push )
#pragma warning( disable : 4267 )
#include "quick_arg_parser/quick_arg_parser.hpp"
#pragma warning( pop )

#include <windows.h>  // windows.h needs to be included before psapi.h !
/**/
#include <filesystem>
#include <iostream>
#include <psapi.h>
#include <stdexcept>

struct Args : MainArguments<Args>
{
  bool installDir     = option( "installDir", 'i', "return OpenVR installation directory" );
  bool currentProcess = option( "currentProcess", 'c', "return PID and name of process currently rendering the scene" );
};

std::shared_ptr<Args> args;

vr::IVRSystem * vrSystem;

void initVR( vr::EVRApplicationType type )
{
  vr::EVRInitError err;
  vrSystem = vr::VR_Init( &err, type );
  if ( err != vr::EVRInitError::VRInitError_None )
  {
    std::stringstream ss;
    ss << "Failed to initialize: " << vr::VR_GetVRInitErrorAsSymbol( err ) << " " << vr::VR_GetVRInitErrorAsEnglishDescription( err ) << "\n";
    throw std::runtime_error( ss.str() );
  }
}

void check( bool expression, std::string message )
{
  if ( !expression )
  {
    throw std::runtime_error( message );
  }
}

int main( int argc, char * argv[] )
{
  try
  {
    args = std::make_shared<Args>( Args{ { argc, argv } } );
  }
  catch ( std::exception & e )
  {
    std::cerr << "Error parsing command line: " << e.what() << std::endl;
    exit( 1 );
  }

  try
  {
    if ( args->installDir )
    {
      initVR( vr::VRApplication_Utility );
      uint32_t          reqSize = MAX_PATH;
      std::vector<char> buf( reqSize, '\0' );
      check( vr::VR_GetRuntimePath( &buf[0], reqSize, &reqSize ), "VR_GetRuntimePath failed" );
      std::string path( &buf[0] );
      std::cout << path << std::endl;
    }
    else if ( args->currentProcess )
    {
      initVR( vr::VRApplication_Background );
      vr::EVRInitError err;
      auto             c = (vr::IVRCompositor *)vr::VR_GetGenericInterface( vr::IVRCompositor_Version, &err );
      check( err == vr::EVRInitError::VRInitError_None, "VR_GetGenericInterface failed: " + std::to_string( err ) );
      uint32_t pid = vr::VRCompositor()->GetCurrentSceneFocusProcess();

      if ( pid > 0 )
      {
        HANDLE handle = OpenProcess( PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, pid );
        check( handle, "OpenProcess for pid " + std::to_string( pid ) + " failed!" );

        std::vector<TCHAR> buf( MAX_PATH );
        DWORD              ret = GetModuleFileNameEx( handle, 0, &buf[0], (DWORD)buf.size() );
        check( ret, "GetModuleFileNameEx failed: " + std::to_string( GetLastError() ) );

        std::string pname = &buf[0];

        std::cout << pid << "," << pname << std::endl;
      }
      else
      {
        std::cout << pid << std::endl;
      }
    }
    else
    {
      std::cerr << "no arguments passed, try \"-?\"" << std::endl;
    }
  }
  catch ( std::exception & e )
  {
    std::cerr << "Error: " << e.what() << std::endl;
    exit( 1 );
  }
  return 0;
}