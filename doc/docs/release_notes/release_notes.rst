.. Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.


Release Notes
=============

Version 1.0.1 (Public)
	* Replay script now takes a list of segments, replays them consecutively
	* Fix possible crash when moving a VCR tape to a different system and the VR app is based on Vulkan
	* Fix possible crash and warn when mixing incompatible hardware.json and tracking_*.bin files on replay
	* Fix possible crash and warn when a controller has an invalid role or left or right controller is missing on capture
	* Fix issue with helper tool output not correctly captured in CMD variables on some systems
	
Version 1.0 (Public)
	* Add capture parameter to configure controller button to start recording 
	* Add replay state flag files, generates 'ready.flag' and 'busy.flag' files for reliable automation synchronization

NVIDIA Internal (Version 0.10)
	* Add notifications in HMD on capture events (start, segment, stop), add parameters to control behavior
	* Add helper/helper.exe tool to help with automation
	* Make cmd scripts more automation friendly (use pushd/popd)
	* Add helper/drivers.cmd tool to help with SteamVR registration issues
	* Bugfix in convert tool - didn't clean up tmp files

NVIDIA Internal (Version 0.9)
	* Make idle animation optional (and off by default)
	* Add versioning support for recorded data, so newer VCR versions can read old data
	* Add "convert" tool to update outdated data files (works for Version 0.7 files and newer)
	* Add logging into "logs" folder 
	* Add parameter for replay to control vsync generation
	* Modify capture parameter, action is now freely definable

Version 0.8 (Public)
	* Add capture parameter to set left trigger to segment recordings
	* Fix crash on replay when no controllers were recorded

NVIDIA Internal (Version 0.7)
	* Add capture segmentation - press ESC to stop recording, press SPACE to create capture segment
	* Add sample showing load, modify, store of tracking data. Filter just shows how to smoothe positional data
	* Fix controller inputs, hardcode controller buttons
	* Make hardware description human-readable and -editable
	* Link capture frequency to HMD refresh (2x) and link replay update frequency to capture frequency (2x)

NVIDIA Internal (Version 0.6)
	* Code cleanup

NVIDIA Internal (Version 0.5)
	* Add parameter for sampling rate
	* Add tracking item interpolation
	* Add parameter for update interval

NVIDIA Internal (Version 0.4)
	* Generate VSync depending on recorded HMD refresh rate

