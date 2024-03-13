# VCR

## Overview
VR Capture & Replay (VCR) is a set of tools to capture, modify and replay OpenVR sessions.

The capture component records any input (HMD poses, controller poses, controller buttons) during an
OpenVR session and stores it to a set of files.

The replay component uses this set of files to emulate VR hardware (HMD and controllers) and
reads the pose and button data to replay the recorded session.

It is possible to modify the tracking data (poses, inputs) through a filter. The VCR repository includes a
sample that demonstrates how to load, modify and save tracking data. The
sample does a simple filtering of the positional data only.

For easier automation, there is a helper tool in the helper directory that provides information about
SteamVR, see parameters in the user guide section.

Due to a change in the use of the ‘cereal’ library, some older tracking and hardware files are not compatible
with VCR anymore. To not lose that data, the convert tool can be used to update hardware and
tracking files of VCR 0.7 and later to the most recent version of VCR.

## Third party modules
This project will download and install additional third-party open source software projects. Review the license terms of these open source projects before use.

# Building

Clone the repository with --recurse-submodules so the dependencies are available.

The VCR root directory contains scripts to generate a solution `generate_solution.cmd` and to perform a complete build `build.cmd`.
CMAke 3.5 or higher is required.

Build files will be generated into the `_build` directory, the VCR package will be generated into the `_out` directory.

# VCR User guide

## Workflow

1. Start application and SteamVR on a normal OpenVR system that has an HMD and controllers
1. Use `1_capture.cmd` to start recording the session
	1. Verify devices and input components are recognized correctly - capture will list all devices it finds
	1. Interact with the application through the HMD, perform actions that should be automated
	1. [OPTIONAL] Press <SPACE> to separate recording into segments - each segment is stored in a ``tracking_<i>.bin`` file and will be replayable separately
	1. Stop recording by pressing <ESC>
1. [OPTIONAL] Use a filter to modify the tracking data - please refer to the readme in the filter sample for more information
1. [OPTIONAL] Copy the resulting `tape` directory to a different system if replay happens somewhere else
1. [OPTIONAL] Update hardware and tracking files to newest VCR version if needed
1. Use `2_install_replay.cmd` to install the replay driver on the target system 
	1. Note that this system must not have an HMD attached (replay driver and real hardware would clash)
	1. Note that SteamVR needs to be installed on the target system
1. Start SteamVR, the replay driver will use the recorded data to emulate the HMD and the controllers
1. Replay the session or the session segments
	1. Use `3_replay.cmd` to replay a single session or the `tracking_0.bin` file
	1. Use `3_replay.cmd <i>` to replay `tracking_<i>.bin` file - this can be used to replay segments in an arbitrary order
	1. Use `3_replay.cmd <i_0> ... <i_n>` to replay the corresponding tracking files consecutively
1. [OPTIONAL] Use `4_uninstall_replay.cmd` to uninstall the replay driver - e.g. if a real HMD should be used again

## Parameters

### Capture CMD line parameters

`-s <f>` sampling frequency in Hz, default: 2x HMD display frequency
	
`-a <hand>,<name>,<input>` recording segmenting action, e.g. left,trigger,click

Use this setting to set up a controller action to segment the recording, in addition to pressing <SPACE>.
Run capture without parameters to see the currently available actions, they are listed
for each controller. Best to use an action that is not used or even has no effect in
the application that's being traced.
The input for this action is not recorded.
	
`-w <hand>,<name>,<input>`
	
Use this setting to set up a controller action to start the recording. If this parameter is set,
capture will initialize and then wait for the configured action before starting the recording.
Run capture without parameters to see the currently available actions, they are listed
for each controller.
Since the action only affects the start of the recording, later inputs of this action will be recorded.
	
`-n` Suppress notifications in HMD. Tool will show notifications by default on start, segment, stop of capture
	
`-t <ms>` For how long (in ms) notifications should be shown

### Replay CMD line parameters

After the OpenVR driver has started, begin a replay by running `3_replay.cmd` with the following parameters:

`<i_0> .. <i_n>` feeds the replay driver with `tracking_<i_0>` .. `tracking_<i_n>` segments
	
Use this feature to replay a set of segments consecutively. The script will use the sync files generated
by the replay driver to determine when to launch the next segment. The replay script itself can synchronize
in two different ways with the replay driver, please look at the replay script file for more details.

### Replay configuration in `VCR/tape/replay_config.json`

Default values can be obtained by deleting the file, replay will generate it on launch

`replaySpeed <s>` Controls the replay speed as a time factor, e.g. 0.5 will replay at half speed

`updateFrequency_in_Hz <hz>` Controls the frequency at which the replayer generates input update events for SteamVR. A value of -1.0 allows the replayer to choose the frequency depending on the recording data
		
`interpolate <b>` Controls whether replay should interpolate between tracking samples
		
`generateVSync <b>` Controls whether replay should generate VSync signals to SteamVR. This can be used for tesing applications in free-running mode for performance benchmarking
	
`vSyncStyle <i>` Controls the VSync generation style. 0: Sleep, 1: Sleep&Busy, 2: Busy

- Sleep: Just use OS sleep. This is quite granular and can cause jitter
- Sleep&Busy: Sleep a bit less, then busy loop until the right time is reached. Slight CPU overhead, but better precision
- Busy: Busy loop the whole time to avoid sending the thread to sleep. High CPU overhead, best precision

`playIdleAnimation <b>` Slightly animate HMD and controllers while no tracking data is replaying
	
### Hardware configuration in `VCR/tape/hardware.json`	

A JSON file containing the hardware description of the HMD and the controllers
	
Most properties are of the form key / property name / value
	
`key`: int value from `vr::ETrackedDeviceProperty`
		
`property name`: name as stated in vr::ETrackedDeviceProperty (trying to document keys in a readable form)
		
`value`: value of that property 
		
`m_hmd`: Properties of the Head-Mounted Display
		
`m_renderWidth` / `m_renderHeight`: resolution of the HMD
			
`m_device` / `m_floatProperties` / `2002` ; `Prop_DisplayFrequency_Float`: frequency (in Hz) at which the replayer generates vsync events for SteamVR
		
`m_device`: Properties of a general OpenVR device (HMD, Controllers, other devices)
		
`m_initialPose` is the position and quaternion of the device's pose when the driver doesn't replay tracking data
		
### Convert tool CMD line parameters

`-m [H/T]` upgrade hardware or tracking file
	
`-i <name>` filename of the input file
	
`-o <name>`	filename of the output file
	
There are also two cmd files for your convenience, just drag a hardware 
or a tracking file onto `convert_hardware.cmd` or `convert_tracking.cmd` respectively.
	
### Helper tool CMD line parameters

`-i` Return current OpenVR/SteamVR installation directory
	
`-c` Return PID and path of the process currently rendering the scene, 0 if no process is rendering

### Drivers tool CMD line parameters

`show` Run `vrpathreg show`, listing all installed SteamVR drivers

`cleanup` Uninstall any VCR driver installed on the system, including VCR drilers installed from other folders

## Supported Hardware

VCR captures and replays the poses of any trackable object in SteamVR. This includes any HMD and controllers, 
but also VIVE Trackers and similar devices.
Button inputs can currently only be captured for some controllers. Other controllers will be tracked, i.e.
their motion will be captured and replayed, but button inputs for these controllers are not captured and 
replayed.

Currently, the following controllers are supported:

- Oculus Rift controllers
- Oculus Quest controllers
- Meta Quest 2 controllers
- VIVE and VIVE Pro controllers

If a controller is not supported, the capture tool will report an unsupported controller in its output
and in its log file.

It is possible that a controller is supported, but still refuses to work. Please check the logs - both 
controllers need to have a valid role (i.e. left or right hand) and generate a list of actions.
Sometimes the role is invalid, this can normally be fixed by restarting SteamVR.

## Automation
VCR replay is intended to be useable in automation environments. The replay driver will generate 'ready.flag' and 'busy.flag' files 
in the tape folder to communicate its current state. If a 'ready.flag' file exists, the replay driver is idle and waiting for a data segment to replay.
If a 'busy.flag' file exists, the replay driver has read a data segment and is replaying it.
Synchronizing an automation tool with VCR using the flag files would look like this::

1. Wait for ready.flag
1. replay segment
1. Wait for busy.flag
1. Wait for ready.flag
1. loop

A CMD script that replays a list of tracking segments could look like below.
The list of segments to be replayed is stored in the variable ``list``, and for each item in the list,
the script first waits for the ready flag, then starts replaying the segment. 
It then waits for VCR to pick up the segment (wait for busy), and for VCR to finish playing (wait for ready).

    set list=1 3 5 5 2 8 4
	for %%i in (%list%) do (
	  call :RUN %%i
	)
	GOTO :EOF

	:RUN
	call :WAITREADY
	echo %1
	call runsession tracking_%1.bin 
	call :WAITBUSY
	call :WAITREADY 
	GOTO :EOF

	:SLEEP
	powershell -nop -c "& {sleep -m 100}"
	GOTO :EOF

	:WAITREADY
	if not exist ready.flag (
	  call :SLEEP
	  GOTO :WAITREADY
	)
	GOTO :EOF

	:WAITBUSY
	if not exist busy.flag (
	  call :SLEEP
	  GOTO :WAITBUSY
	)
	GOTO :EOF

For easier management of segments, it is possible to rename the files, it must just be made sure that the replay scripts
are called with the correct parameters.

It is also possible to detect whenever VCR replay has started replaying a segment. The scripts around replay will copy a segment to a 'run.bin' file
which will get read and then deleted by VCR replay. The moment the run.bin file disappears marks the beginning of replay for that segment.
A script could wait for the run.bin file to disappear to launch the next segment, i.e. create another run.bin file. This file will be picked up the moment the
old data has finished replaying, so that no frames are spent idle. This approach is useful for consecutive replay of segments, but it's harder to synchronize when a segment 
has finished playing.

The above cmd script would need to be modified for this behavior:

	:RUN
	echo %1
	call :WAITRUNBIN
	call runsession tracking_%1.bin
	GOTO :EOF

	:WAITRUNBIN
	if exist run.bin (
	  call :SLEEP
	  GOTO :WAITRUNBIN
	)
	GOTO :EOF


The replay driver will re-read its configuration whenever it reads a new segment. Some parameters can be modified while VCR replay is running, most importantly the
replay speed. An automation can copy different configurations into place while it's feeding VCR replay to change its behavior.

## Notes

To make replays able to loop, find a way to reset the app state to a known state at the end of the segment - jump to a defined viewpoint, close menus, etc.

If you want to test with different hardware setups during replay, it may be best to record with the lowest-end hardware configuration.
The problem with slower hardware is that apps tend to react slower, and inputs may get lost because the app is not showing the right state for a recorded input yet.

The logs folder contains logs of any run of ``capture``, ``convert`` and ``replay``. A failing replay may be caused by issues from a broken capture session.

## Troubleshooting 

The `capture` and `replay` tools generate logs in the `logs` folder. If anything goes wrong, please check the logs first, they should hint at issues. Failing replays could be caused by broken capture data.

If SteamVR shows the Dashboard and it interferes with the replay, disable SteamVR Settings / Dashboard / "VR Dashboard on System Button".

If VCR records and correctly replays motion but not controller input, it is possible that the controller is not yet known to VCR.
VCR capture will notify about this in the CMD output and in the log files.
SteamVR does not allow enumerating controller inputs, so any controller currently needs to be hard coded. 

If you have a difference between the capture and replay (e.g. menu doesn't get hit correctly,
tracking seems to be off by some delta), it's possible that the app performs some calibration based
on the HMD position it sees first.
To fix this, please do not move the HMD while starting the application and the capture.
Put the HMD on a chair or a rack while starting both, then start interacting in VR normally.

If the replay driver does not seem to work (SteamVR starts and does not find an HMD after installing replay),
check SteamVR Options -> Startup/Shutdown -> Manage Add-Ons.
The `VCR_replay` driver may be disabled because of a previous crash, please re-enable it.
In this case, the vrserver.txt log visible in the SteamVR System Report tool will contain
a line like `Not loading driver null because it is disabled in settings` and/or
`Not loading driver replay because it was blocked by a previous safe mode event` (use the raw view and search with the text field in the bottom)

It is also possible that the driver does not seem to find the captured tape(s), or misbehaves in some other way.
If this is the case, please check that there is only one version of VCR installed. If VCR was installed from different directories, the installations will interfere - SteamVR only allows one instance of VCR to be active at a time. Use the helper tool to check for installed drivers.

The VCR replay driver needs to be removed from SteamVR when changing VCR versions or when other HMDs are used.
Either use the `4_uninstall_replay.cmd` script to remove the current VCR driver, or remove all 0.10+ VCR versions 
with a script in the helper directory - just open a CMD and run `drivers.cmd cleanup`.

If a replay log contains the message

`ERROR: Action Data index out of bounds. This can happen if incompatible tracking and hardware data files are mixed`

please make sure the `hardware.json` file and the `tracking_*.bin` files come from the same HMD and controllers.
Because different controllers have different inputs (buttons are different, etc.), VCR can not translate
between controllers. If these files need to be mixed, it would be possible to extend the filter sample to translate
between the controllers involved, and map the actions of the source controller to the actions of the target controller.

If a capture log contains the messages

`Warning! This device has an invalid role. Capturing only pose data, no action (e.g. button) inputs!`

or

`Warning! No left/right controller found. This might cause broken capture data.`

it is possible that the captured session is faulty. These situations can arise if SteamVR did not know how to
correctly handle a controller, or sometimes if different controllers were mixed when capturing.
Sometimes, restarting SteamVR can resolve an invalid controller role.

If you'd like to file an issue, please include the following information: 

- Error description (maybe a screenshot if it shows something useful)
- Which version you're using (VNC/version.txt, capture and replay should generate version output as well)
- The log text of any component involved (log of capture session can be useful if replay fails) - check the `logs` folder
- A SteamVR System Report (SteamVR menu -> Create System Report -> Save to file)


# NVIDIA VCR Filter Programming Guide

The VCR package contains the source and a binary of a filter that can load, modify and save the tracking files generated by `1_capture.cmd`.

## NVIDIA VCR Tracking Data Filter Sample


This sample shows how to implement a filter that allows to modify the tracking data between capture and replay.

The sample currently only touches the positional part of the tracking data, and smoothes this data. 
Smoothing rotations through simple averaging could be possible, but is not mathematically correct and is not scope of this example.

The sample can be built by first generating a solution via `CMake` and then compiling it with the compiler of your choice. Note that C++17 support is required.

Tracking data is read and written using `cereal`, and the data is structured as an instanciation of struct `TrackingData`, as can be found in `VCR/sample/shared/VRData.h`.

Reading and writing tracking data are shown in the functions `readTrackingData` and `writeTrackingData` in `sample.cpp`, respectively.

The filtering happens in the function `filterTrackingData`. 
For every tracking item, the filter first determines the neighborhood around which it can work - this neighborhood needs to be a symmetrical
amount of items before and after the item. E.g. for i=0 there's no items before the item, so the averaging only works on the item itself.
For i=2 the items 0,1,2,3,4 are used for averaging.

The filter then passes along the neighborhood, sums up the positional data and divides by its current domain size to generate an average.

The sample filter can be run by calling the command line `sample.exe <input.bin> <iterations> <output.bin>` where `<input.bin>` and `<output.bin>`
are valid paths for reading and writing tracking data. Note that the sample will not overwrite existing files.
`<iterations>` is the number of passes the sample should take over the tracking data.

# Resources

We've presented VCR at different conferences and events, here are some links to a few:

[VCR Webinar](https://info.nvidia.com/xr-vcr-reg-page.html)

[VCR presentation at AWE](https://www.youtube.com/watch?v=IOwR58p70z)

