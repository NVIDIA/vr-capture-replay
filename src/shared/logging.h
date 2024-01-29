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



#include "nvprint.hpp"


#include <chrono>
#include <ctime>
#include <filesystem>
#include <sstream>
#include <string>
#include <iomanip>

inline void initLog(std::string logPath = "")
{
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream tmp;
#pragma warning( push )
#pragma warning( disable : 4996 )
    tmp << std::put_time(std::localtime(&t), "%Y-%m-%d_%H%M%S");
#pragma warning(pop)
    std::string timeString = tmp.str();

    std::stringstream ss;
    if (!logPath.empty())
    {
        ss << logPath << "\\";
    }
    ss << "log_" << PROJECT_NAME << "_";
    ss << timeString;
    ss << ".txt";
    nvprintSetLogFileName(ss.str().c_str());
    LOGI("NVIDIA VCR " PROJECT_NAME " version " VCR_VERSION "\n\n");
    LOGI("%s\n\n", timeString.c_str());
    LOGI("current path: %s\n\n", std::filesystem::current_path().string().c_str());
}

inline void logCommandLine(char argc, char* argv[])
{
    LOGI("Command Line\n");
    for (char i = 0; i < argc; ++i)
    {
        LOGI("%s ", argv[i]);
    }
    LOGI("\n\n");
}

