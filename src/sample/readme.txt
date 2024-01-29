
NVIDIA VCR Tracking Data Filter Sample
--------------------------------------

NOTE: The sample uses the 'cereal' library for reading and writing the tracking_<i>.bin files. 
The library and its license are located in the VCR/sample/thirdparty folder.


This sample shows how to implement a filter that allows to modify the tracking data between capture and replay.

The sample currently only touches the positional part of the tracking data, and smoothes this data. 
Smoothing rotations through simple averaging could be possible, but is not mathematically correct and is not scope of this example.

The sample can be built by first generating a solution via CMake and then compiling it with the compiler of your choice. Note that C++17 support is required.

Tracking data is read and written using cereal, and the data is structured as an instanciation of struct TrackingData, as can be found in VCR/sample/shared/VRData.h.

Reading and writing tracking data are shown in the functions readTrackingData and writeTrackingData in sample.cpp, respectively.

The filtering happens in the function filterTrackingData. 
For every tracking item, the filter first determines the neighborhood around which it can work - this neighborhood needs to be a symmetrical
amount of items before and after the item. E.g. for i=0 there's no items before the item, so the averaging only works on the item itself.
For i=2 the items 0,1,2,3,4 are used for averaging.

The filter then passes along the neighborhood, sums up the positional data and divides by its current domain size to generate an average.

The sample filter can be run by calling the command line
	sample.exe <input.bin> <iterations> <output.bin> 
where <input.bin> and <output.bin> are valid paths for reading and writing tracking data. Note that the sample will not overwrite existing files.
<iterations> is the number of passes the sample should take over the tracking data.