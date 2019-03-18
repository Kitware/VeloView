# Introduction

VeloView performs real-time visualization of live captured 3D LiDAR data
from Velodyne's HDL sensors (HDL-32E and HDL-64E).

VeloView can playback pre-recorded data stored in .pcap files. The HDL
sensor sweeps an array of lasers (32 or 64) 360&deg; and a vertical field of
view of 40&deg;/26&deg; with 5-20Hz and captures about a million points per
second (HDL-32E: ~700,000pt/sec; HDL-64E: ~1.3Million pt/sec).
VeloView displays the distance measurements from the HDL as point cloud
data and supports custom color maps of multiple variables such as
intensity-of-return, time, distance, azimuth, and laser id. The data can
be exported as XYZ data in CSV format or screenshots of the currently
displayed point cloud can be exported with the touch of a button.

# Features

-   Input from live sensor stream or recorded .pcap file
-   Visualization of LiDAR returns in 3D + time including 3d position
    and attribute data such as timestamp, azimuth, laser id, etc
-   Spreadsheet inspector for LiDAR attributes
-   Record to .pcap from sensor
-   Export to CSV or VTK formats
-   Record and export GPS and IMU data (*New in 2.0*)
-   Ruler tool (*New in 2.0*)
-   Visualize path of GPS data (*New in 2.0*)
-   Show multiple frames of data simultaneously (*New in 2.0*)
-   Show or hide a subset of lasers (*New in 2.0*)

# How to Obtain

Binary installers for VeloView are available as community contributed
applications:

* [Version 2.0 - Windows 64](http://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v4.1&type=app&os=win64&downloadFile=VeloView-2.0.0-31032014-Windows-64bit.exe)
* [Version 2.0 - Windows 32](http://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v4.1&type=app&os=win32&downloadFile=VeloView-2.0.0-31032014-Windows-32bit.exe)
* [Version 2.0 - Mac OSX 10.8](http://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v4.1&type=app&os=osx&downloadFile=VeloView-2.0.0-31032014-Darwin-64bit.dmg)

The source code for VeloView is made available under the Apache 2.0
license.

Sample data for VeloView can be obtained from
[Girder](https://girder.readthedocs.io) in the
[Velodyne LiDAR
collection](https://data.kitware.com/#collection/5b7f46f98d777f06857cb206).

# How to use

For "sensor streaming" (live display of sensor data) it
is important to change the network settings of the Ethernet adapter
connected to the sensor from automatic IP address to manual IP address
selection and choose:

* HDL-32E
  * IP address: 192.168.1.70 (70 as example, any number except 201 works)
  * Gateway: 255.255.255.0
* HDL-64E
  * IP address: 192.168.3.70 (70 as example, any number except 43 works)
  * Gateway: 192.168.3.255

In order for sensor streaming to work properly, it is important to
disable firewall restrictions for the Ethernet port. Disable the
firewall completely for the ethernet device connected to the sensor or
explicitly allow data from that Ethernet port of (including both public
and private networks).

When opening pre-recorded data or live sensor streaming data one is
prompted to choose a calibration file.

* For HDL-32E data no calibration
file is needed (the HDL-32E calibration values are already incorporated
in VeloView) therefore select "NONE".
* For HDL-64E data the correct
calibration file for that sensor needs to be chosen. The calibration
file can be found on the individual product CD that was send with the
HDL-64E sensor.

# How to build

Detailed instructions for building and packaging are available in the
[VeloView Developer Guide](Documentation/VeloView_Developer_Guide.md) .
