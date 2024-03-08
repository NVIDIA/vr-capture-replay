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

#include "nvprint.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

inline void initLog( std::string logPath = "" )
{
  auto              t = std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() );
  std::stringstream tmp;
#pragma warning( push )
#pragma warning( disable : 4996 )
  tmp << std::put_time( std::localtime( &t ), "%Y-%m-%d_%H%M%S" );
#pragma warning( pop )
  std::string timeString = tmp.str();

  std::stringstream ss;
  if ( !logPath.empty() )
  {
    ss << logPath << "\\";
  }
  ss << "log_" << PROJECT_NAME << "_";
  ss << timeString;
  ss << ".txt";
  nvprintSetLogFileName( ss.str().c_str() );
  LOGI( "NVIDIA VCR " PROJECT_NAME " version " VCR_VERSION "\n\n" );
  LOGI( "%s\n\n", timeString.c_str() );
  LOGI( "current path: %s\n\n", std::filesystem::current_path().string().c_str() );
}

inline void logCommandLine( char argc, char * argv[] )
{
  LOGI( "Command Line\n" );
  for ( char i = 0; i < argc; ++i )
  {
    LOGI( "%s ", argv[i] );
  }
  LOGI( "\n\n" );
}
