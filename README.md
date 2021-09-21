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

# How to Get

Binary installers for VeloView are available on this page: [https://gitlab.kitware.com/LidarView/VeloView-Velodyne/-/releases](https://gitlab.kitware.com/LidarView/VeloView-Velodyne/-/releases)

VeloView has the same runtime requirements as LidarView, see [INSTALLATION.md](https://gitlab.kitware.com/LidarView/lidarview-core/-/blob/master/Documentation/INSTALLATION.md)

# How to Build

VeloView compilation follows the same steps as LidarView, see [Developper Guide](https://gitlab.kitware.com/LidarView/lidarview-core/-/blob/master/Documentation/LidarView_Developer_Guide.md)

The source code for VeloView is made available under the Apache 2.0
license.

# How to Use

Take a look at: [VeloView User Guide](https://gitlab.kitware.com/LidarView/VeloView-Velodyne/-/blob/master/Documentation/VeloView_User_Guide.pdf)

Get started with SLAM using this Guide : [How to SLAM](https://gitlab.kitware.com/keu-computervision/slam/-/blob/master/paraview_wrapping/doc/How_to_SLAM_with_LidarView.md)

See LidarView & SLAM in action in the [LidarView 2021 Webinar Video](https://vimeo.com/524848891)

Sample data for VeloView can be obtained from
[MIDAS](http://www.midasplatform.org/) in the
[Velodyne LiDAR
collection](http://midas3.kitware.com/midas/community/29).

# Configuration Tips

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
file can be found on the individual product CD that was sent with the
HDL-64E sensor.

# For Github users

[Github](https://github.com/Kitware/VeloView) is a mirror of the
[official repository](https://gitlab.kitware.com/LidarView/VeloView-Velodyne).
We do not actively monitor issues or pull request on Github. Please use the
[official repository](https://gitlab.kitware.com/LidarView/VeloView-Velodyne) to report issues or contributes fixes.

