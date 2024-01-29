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


#include "shared/VRData.h"

#include <filesystem>
#include <fstream>

VRData::TrackingData trackingData;

bool readTrackingData(std::string file)
{
    std::ifstream ifile(file, std::ios::binary);
    if (ifile)
    {
        std::cout << "Reading data from " << file << "\n";
        try
        {
            cereal::BinaryInputArchive binInArchive(ifile);
            binInArchive(trackingData);
            std::cout << "Tracking data loaded, " << trackingData.m_trackingItems.size() << " tracking items\n";
        }
        catch (std::exception& e)
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

bool writeTrackingData(std::string file)
{
    int i = 0;
    if (std::filesystem::exists(file))
    {
        std::cerr << "File already exists, aborting write\n";
        return false;
    }
    std::ofstream ofile(file, std::ios::binary);
    if (ofile)
    {
        std::cout << "Writing tracking file: " << file << "\n";
        try
        {
            cereal::BinaryOutputArchive binOutArchive(ofile);
            binOutArchive(trackingData);
            std::cout << "Tracking data written\n";
        }
        catch (std::exception& e)
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
    auto average = [](const auto& in, auto& out, const size_t index, const size_t size)
    {
        double num = 2 * (double)size + 1;

        VRData::TrackingItem& ti_out = out[index];

        {
            // average HMD pose into ti
            auto& p = ti_out.m_hmdPose.m_pos;
            p[0] = p[1] = p[2] = 0.0;
            for (size_t i = index - size; i < index + size + 1; ++i)
            {
                p[0] += in[i].m_hmdPose.m_pos[0];
                p[1] += in[i].m_hmdPose.m_pos[1];
                p[2] += in[i].m_hmdPose.m_pos[2];
            }
            p[0] /= num;
            p[1] /= num;
            p[2] /= num;
        }

        for (size_t j = 0; j < ti_out.m_controllerPoses.size(); ++j)
        {
            // average controller poses into ti
            auto& p = ti_out.m_controllerPoses[j].m_pos;
            p[0] = p[1] = p[2] = 0.0;
            for (size_t i = index - size; i < index + size + 1; ++i)
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


    const size_t s = 4; // average up to s to left and the right, i.e. 2s+1 items

    const auto& in = trackingData.m_trackingItems;
    // copy input data to out to keep all untouched data intact
    auto out = trackingData.m_trackingItems;
    const size_t n = in.size();

    for (size_t i = 0; i < n; ++i)
    {
        const size_t d = std::min(i, (n - 1) - i); // determine distance to ends
        const size_t size = std::min(s, d);        // determine filter size
        average(in, out, i, size);                 // average <i>-th item of <in> using filter size <size>, store in <i>-th item of <out>
    }
    trackingData.m_trackingItems = out;
}


int main(char argc, char* argv[])
{
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] << " <input.bin> <iterations> <output.bin>\n";
        return 1;
    }
    size_t iterations{ 0 };
    try
    {
        iterations = std::stoi(argv[2]);
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what();
        return 1;
    }

    readTrackingData(argv[1]);
    for (size_t i = 0; i < iterations; ++i)
    {
        filterTrackingData();
    }
    writeTrackingData(argv[3]);
    return 0;
}
