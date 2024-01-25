
NVIDIA VCR Data Converter
-------------------------

Converts hardware and tracking data files to current VCR version.

The 'cereal' library supports unversioned and versioned data files. VCR v0.8 and before
used unversioned files, and they are incompatible to versioned data files.
With VCR v0.9, older (v0.7 and v0.8) data files can thus no longer be loaded.
Also, versions of VCR higher than v0.9 may have changes to the data structs.


Command Line Parameters

-m      --mode   (T)racking or (H)ardware:  Whether the file is a hardware or a tracking file
-i      --infile         input file:        Input file name
-o      --outfile        output file:       Output file name

Example: -m H -i hardware_old.json -o hardware_new.json

There are also two batch files (convert_hardware and convert_tracking) that take a single
file name as input and call convert with appropriate parameters, creating a _new file.