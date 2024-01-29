.. Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.


NVIDIA VCR Overview
===================

NVIDIA (R) VR Capture & Replay (VCR) is a tool to capture, modify and replay OpenVR sessions.

The capture component records any input (HMD poses, controller poses, controller buttons)
during an OpenVR session and stores it to a set of files.

The replay component uses this set of files to emulate the VR hardware (HMD and controllers)
and reads the pose and button data to replay the recorded session.

It is possible to modify the tracking data (poses, inputs) through a filter. The VCR package
includes a sample in source and binary form that demonstrates how to load, modify and save 
tracking data. The sample does a simple filtering of the positional data only.

For easier automation, there is a helper tool in the helper directory that provides information
about SteamVR, see parameters in the user guide section.

Due to a change in the use of the 'cereal' library, some older tracking and hardware files
are not compatible with VCR anymore. To not lose that data, the convert tool can be used
to update hardware and tracking files of VCR 0.7 and later to the most recent version of VCR.



Third party libraries
---------------------

This project uses the following third party libraries:

* cereal ( https://github.com/USCiLab/cereal )
* Magic Enum C++ ( https://github.com/Neargye/magic_enum )
* OpenVR SDK ( https://github.com/ValveSoftware/openvr )
* Quick Arg Parser ( https://github.com/Dugy/quick_arg_parser )

The licenses for these libraries are included in this package 
under the ``VCR\\3rd_party_licenses`` folder.
