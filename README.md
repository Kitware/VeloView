# Introduction

LidarView performs real-time visualization of live captured 3D LiDAR data
from Velodyne's HDL sensors (HDL-32E and HDL-64E).

LidarView can playback pre-recorded data stored in .pcap files. The HDL
sensor sweeps an array of lasers (32 or 64) 360&deg; and a vertical field of
view of 40&deg;/26&deg; with 5-20Hz and captures about a million points per
second (HDL-32E: ~700,000pt/sec; HDL-64E: ~1.3Million pt/sec).
LidarView displays the distance measurements from the HDL as point cloud
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

Binary installers for LidarView are available as community contributed
applications:

* [Version 2.0 - Windows 64](http://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v4.1&type=app&os=win64&downloadFile=LidarView-2.0.0-31032014-Windows-64bit.exe)
* [Version 2.0 - Windows 32](http://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v4.1&type=app&os=win32&downloadFile=LidarView-2.0.0-31032014-Windows-32bit.exe)
* [Version 2.0 - Mac OSX 10.8](http://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v4.1&type=app&os=osx&downloadFile=LidarView-2.0.0-31032014-Darwin-64bit.dmg)

The source code for LidarView is made available under the Apache 2.0
license.

Sample data for LidarView can be obtained from
[MIDAS](http://www.midasplatform.org/) in the
[Velodyne LiDAR
collection](http://midas3.kitware.com/midas/community/29).

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
in LidarView) therefore select "NONE".
* For HDL-64E data the correct
calibration file for that sensor needs to be chosen. The calibration
file can be found on the individual product CD that was send with the
HDL-64E sensor.

# How to build

Detailed instructions for building and packaging are available in the
[LidarView Developer Guide (link non working in Gitlab UI)](
LVCore/Documentation/LidarView_Developer_Guide.md).
In Gitlab UI click `LVCore` above then `Documentation` then
`LidarView_Developer_Guide.md`.
<!--
This link indeed does not work in Gitlab web UI. We could link relatively
to the version blob/master of LVCore but this has the big risk of giving a
version ahead of what the user needs, thus desynced!
This link is however useful in IDEs that support following links.
-->

# For Github users
[Github](https://github.com/Kitware/VeloView) is a mirror of the
[official repository](https://gitlab.kitware.com/LidarView/VeloView-Velodyne).
We do not actively monitor issues or pull request on Github. Please use the
[official repository](https://gitlab.kitware.com/LidarView/VeloView-Velodyne) to report issues or contributes fixes.

