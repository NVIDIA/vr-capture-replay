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

#include "shared/VRData.h"

#include <filesystem>
#include <fstream>

VRData::TrackingData trackingData;

bool readTrackingData( std::string file )
{
  std::ifstream ifile( file, std::ios::binary );
  if ( ifile )
  {
    std::cout << "Reading data from " << file << "\n";
    try
    {
      cereal::BinaryInputArchive binInArchive( ifile );
      binInArchive( trackingData );
      std::cout << "Tracking data loaded, " << trackingData.m_trackingItems.size() << " tracking items\n";
    }
    catch ( std::exception & e )
    {
      std::cerr << "Exception: " << e.what();
      return false;
    }
  }
  else
  {
    std::cerr << "Could not open " << file << " for reading\n";
    return false;
  }
  return true;
}

bool writeTrackingData( std::string file )
{
  int i = 0;
  if ( std::filesystem::exists( file ) )
  {
    std::cerr << "File already exists, aborting write\n";
    return false;
  }
  std::ofstream ofile( file, std::ios::binary );
  if ( ofile )
  {
    std::cout << "Writing tracking file: " << file << "\n";
    try
    {
      cereal::BinaryOutputArchive binOutArchive( ofile );
      binOutArchive( trackingData );
      std::cout << "Tracking data written\n";
    }
    catch ( std::exception & e )
    {
      std::cerr << "Exception: " << e.what();
      return false;
    }
  }
  else
  {
    std::cerr << "Could not open " << file << " for writing\n";
    return false;
  }
  return true;
}

/*
 * This filter shows a simple averaging of positional data.
 * It first determines what neighborhood can be averaged,
 * the filter needs a symmetrical amount of items around i.
 * E.g. for i = 0 there's no items to the left,
 * for n-2 there's only one item to the right.
 * Then the data in this neighborhood is summed and divided
 * by the number of items participating.
 *
 * Note that this filter only touches positional data.
 * Other data like rotation can be filtered as well,
 * but this is not the scope of this demonstration.
 */
void filterTrackingData()
{
  auto average = []( const auto & in, auto & out, const size_t index, const size_t size )
  {
    double num = 2 * (double)size + 1;

    VRData::TrackingItem & ti_out = out[index];

    {
      // average HMD pose into ti
      auto & p = ti_out.m_hmdPose.m_pos;
      p[0] = p[1] = p[2] = 0.0;
      for ( size_t i = index - size; i < index + size + 1; ++i )
      {
        p[0] += in[i].m_hmdPose.m_pos[0];
        p[1] += in[i].m_hmdPose.m_pos[1];
        p[2] += in[i].m_hmdPose.m_pos[2];
      }
      p[0] /= num;
      p[1] /= num;
      p[2] /= num;
    }

    for ( size_t j = 0; j < ti_out.m_controllerPoses.size(); ++j )
    {
      // average controller poses into ti
      auto & p = ti_out.m_controllerPoses[j].m_pos;
      p[0] = p[1] = p[2] = 0.0;
      for ( size_t i = index - size; i < index + size + 1; ++i )
      {
        p[0] += in[i].m_controllerPoses[j].m_pos[0];
        p[1] += in[i].m_controllerPoses[j].m_pos[1];
        p[2] += in[i].m_controllerPoses[j].m_pos[2];
      }
      p[0] /= num;
      p[1] /= num;
      p[2] /= num;
    }
  };

  const size_t s = 4;  // average up to s to left and the right, i.e. 2s+1 items

  const auto & in = trackingData.m_trackingItems;
  // copy input data to out to keep all untouched data intact
  auto         out = trackingData.m_trackingItems;
  const size_t n   = in.size();

  for ( size_t i = 0; i < n; ++i )
  {
    const size_t d    = std::min( i, ( n - 1 ) - i );  // determine distance to ends
    const size_t size = std::min( s, d );              // determine filter size
    average( in, out, i, size );                       // average <i>-th item of <in> using filter size <size>, store in <i>-th item of <out>
  }
  trackingData.m_trackingItems = out;
}

int main( char argc, char * argv[] )
{
  if ( argc != 4 )
  {
    std::cerr << "Usage: " << argv[0] << " <input.bin> <iterations> <output.bin>\n";
    return 1;
  }
  size_t iterations{ 0 };
  try
  {
    iterations = std::stoi( argv[2] );
  }
  catch ( std::exception & e )
  {
    std::cerr << "Exception: " << e.what();
    return 1;
  }

  readTrackingData( argv[1] );
  for ( size_t i = 0; i < iterations; ++i )
  {
    filterTrackingData();
  }
  writeTrackingData( argv[3] );
  return 0;
}
