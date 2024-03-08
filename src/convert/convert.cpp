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

#include "VRData_0.8.h"
#include "shared/VRData.h"
#include "shared/logging.h"

#pragma warning( push )
#pragma warning( disable : 4267 )
#include "quick_arg_parser/quick_arg_parser.hpp"

#pragma warning( pop )

#include <filesystem>
#include <fstream>

struct Args : MainArguments<Args>
{
  std::string mode    = option( "mode", 'm', "(T)racking or (H)ardware" );
  std::string infile  = option( "infile", 'i', "input file" );
  std::string outfile = option( "outfile", 'o', "output file" );
};

Args args;

bool readTrackingData( VRData::TrackingData & trackingData, std::string file );
bool readHardwareData( VRData::HardwareData & hardwareData, std::string file );

template <typename T>
bool readData( T & data, std::string file );

template <typename T>
bool writeData( const T & data, std::string file );

bool atEnd( const std::string & str, const std::string & end )
{
  return str.compare( str.size() - end.size(), end.size(), end ) == 0;
}

int main( char argc, char * argv[] )
{
  // command line parameter handling
  try
  {
    args = Args{ { argc, argv } };
    // this should be feasible through =argument(0) in quick_arg_parser but it causes runtime errors
    if ( ( args.mode != "T" ) && ( args.mode != "H" ) || args.infile.empty() || args.outfile.empty() )
    {
      std::cerr << "Invalid arguments, try --help\n";
      exit( 1 );
    }
  }
  catch ( std::exception & e )
  {
    std::cerr << "Error parsing command line: " << e.what() << std::endl;
    exit( 1 );
  }

  std::string logPath = std::filesystem::absolute( "..\\logs" ).string();
  if ( !std::filesystem::exists( logPath ) )
  {
    logPath.clear();
  }
  initLog( logPath );
  logCommandLine( argc, argv );

  // read (and implicitly convert) - write
  if ( args.mode == "T" )
  {
    VRData::TrackingData trackingData;
    if ( readTrackingData( trackingData, args.infile ) )
    {
      if ( !writeData( trackingData, args.outfile ) )
      {
        return 1;
      }
    }
    else
    {
      return 1;
    }
  }
  else if ( args.mode == "H" )
  {
    VRData::HardwareData hardwareData;
    if ( readHardwareData( hardwareData, args.infile ) )
    {
      if ( !writeData( hardwareData, args.outfile ) )
      {
        return 1;
      }
    }
    else
    {
      return 1;
    }
  }
  return 0;
}

bool readTrackingData( VRData::TrackingData & trackingData, std::string file )
{
  bool        useTmpFile  = false;
  std::string tmpFileName = "tmp.bin";

  LOGI( "Trying to read unversioned file type to convert to versioned... \n" );
  VRData_0_8_unversioned::TrackingData trackingData_0_8_unversioned;
  if ( readData( trackingData_0_8_unversioned, file ) )
  {
    LOGI( "Unversioned tracking data loaded, " );
    LOGI( "%i tracking items\n", trackingData_0_8_unversioned.m_trackingItems.size() );

    LOGI( "Converting unversioned file to v0.8 versioned file...\n" );

    VRData_0_8_versioned::TrackingData trackingData_0_8_versioned( trackingData_0_8_unversioned );

    LOGI( "Saving as versioned v0.8 file...\n" );
    if ( !writeData( trackingData_0_8_versioned, tmpFileName ) )
    {
      return false;
    }
    useTmpFile = true;
  }

  std::string fileName = useTmpFile ? tmpFileName : file;
  LOGI( "Trying to read versioned file type...\n" );
  bool success = readData( trackingData, fileName );

  // clean up tmp file before handling success or failure of read
  if ( useTmpFile )
  {
    LOGI( "Deleting file: %s\n", fileName.c_str() );
    std::remove( fileName.c_str() );
  }

  if ( success )
  {
    LOGI( "Tracking data loaded, %i tracking items\n", trackingData.m_trackingItems.size() );
  }
  else
  {
    LOGE( "Reading failed, aborting\n" );
    LOGE( "Please make sure the file is from VCR version 0.7 or newer\n" );
    return false;
  }

  return true;
}

bool readHardwareData( VRData::HardwareData & hardwareData, std::string file )
{
  bool        useTmpFile  = false;
  std::string tmpFileName = "tmp.json";

  LOGI( "Trying to read unversioned file type to convert to versioned... \n" );
  VRData_0_8_unversioned::HardwareData hardwareData_0_8_unversioned;
  if ( readData( hardwareData_0_8_unversioned, file ) )
  {
    LOGI( "Unversioned hardware data loaded\n" );
    LOGI( "Converting unversioned file to v0.8 versioned file...\n" );

    VRData_0_8_versioned::HardwareData hardwareData_0_8_versioned( hardwareData_0_8_unversioned );

    LOGI( "Saving as versioned v0.8 file...\n" );
    if ( !writeData( hardwareData_0_8_versioned, tmpFileName ) )
    {
      return false;
    }
    useTmpFile = true;
  }

  std::string fileName = useTmpFile ? tmpFileName : file;
  LOGI( "Trying to read versioned file type...\n" );
  bool success = readData( hardwareData, fileName );

  // clean up tmp file before handling success or failure of read
  if ( useTmpFile )
  {
    LOGI( "Deleting file: %s\n", fileName.c_str() );
    std::remove( fileName.c_str() );
  }

  if ( success )
  {
    LOGI( "Hardware data loaded\n" );
  }
  else
  {
    LOGE( "Reading failed, aborting\n" );
    return false;
  }

  return true;
}

template <typename T>
bool readData( T & data, std::string file )
{
  LOGI( "Reading file: %s\n", file.c_str() );

  if ( !std::filesystem::exists( file ) )
  {
    LOGE( "File %s does not exist!\n", file.c_str() );
    return false;
  }

  std::ifstream ifile( file, std::ios::binary );
  if ( ifile )
  {
    try
    {
      if ( atEnd( file, "json" ) )
      {
        cereal::JSONInputArchive jsonInArchive( ifile );
        jsonInArchive( data );
      }
      else
      {
        cereal::BinaryInputArchive binInArchive( ifile );
        binInArchive( data );
      }
      LOGI( "Data successfully read\n" );
    }
    catch ( std::exception & e )
    {
      LOGE( "Exception: %s\n\n", e.what() );
      return false;
    }
  }
  else
  {
    LOGE( "Could not open %s for reading\n", file.c_str() );
    return false;
  }
  return true;
}

template <typename T>
bool writeData( const T & data, std::string file )
{
  LOGI( "Writing file: %s\n", file.c_str() );

  if ( std::filesystem::exists( file ) )
  {
    LOGE( "File %s already exists, aborting write\n", file.c_str() );
    return false;
  }

  std::ofstream ofile( file, std::ios::binary );
  if ( ofile )
  {
    try
    {
      if ( atEnd( file, "json" ) )
      {
        cereal::JSONOutputArchive jsonOutArchive( ofile );
        jsonOutArchive( data );
      }
      else
      {
        cereal::BinaryOutputArchive binOutArchive( ofile );
        binOutArchive( data );
      }
      LOGI( "Data successfully written\n" );
      ;
    }
    catch ( std::exception & e )
    {
      LOGE( "Exception: %s\n\n", e.what() );
      return false;
    }
  }
  else
  {
    LOGE( "Could not open %s for writing\n", file.c_str() );
    return false;
  }
  return true;
}