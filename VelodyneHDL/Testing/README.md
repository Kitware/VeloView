# HOW TO: Use VeloView testing


### Reader tests definition


The tests are based on the comparaison of the processing of a reference recording 
**pcap** file with the baseline **vtp** file, that have been generate on a **stable version** of the Reader.


### Enable VeloView testing


In VeloView CMAKE options, enable option `BUILD_TESTING`. Then rebuild Veloview.


### Get test data and baseline


The Test Data (**.pcap** file) and the baseline (**.vtp** file) are store on another 
gitlab repository. This repository is a submodule of Veloview, to get this data simply run:
```
git submodule update --init
```
This may took some minutes. Finally the test data and baseline could be found in `/TestData`


### Run the tests

To launch a test, use the program CTest. CTest needs to be run from the VeloView
build directory.

To use CTest from the command line on Linux or MacOS, do:
```
ctest -R <REGEX_TEST_NAME> [-VV]
```

* **-R** option enable to run run a specific test. This option is mandatory.
* **TEST_NAME** is the name of the test to run. You can use `tab` to autocomplete a test name.
* **-VV** option allows the test to be run in verbose mode, which displays more information. This option is optional.

**Using CTest on Windows:** On Windows, you need to add the option
`-C <debug/release>` according to your build type.


### Update test data

**Disclaimer:** In some rare cases, the functionality added to VeloView modifies
some properties of the 3D points tested above, and thus the tests will return a 
failure even if the values tested match the ones intended. Please run all the 
tests and ensure that just the one intended to fail fails for your modifications
before generating updated test data.

To generate updated test data automatically, go to your VeloView build directory
and launch VeloView with the option --script.
On Linux:
```
BUILD_DIR/veloview/src/veloview-build/bin/VeloView --script=BUILD_DIR/veloview/src/veloview-build/bin/generateTestData.py
```
On Windows:
```
INSTALL_DIR/bin/VeloView.exe --script=INSTALL_DIR/bin/generateTestData.py
```
On MacOS:
```
open PACKAGE_DIR/VeloView.app --args --script=BUILD_DIR/veloview/src/veloview-build/bin/generateTestData.py
```
Updated test data will be generated in `/TestData`. Commit your changes in the
submodule `TestData` fisrt and then commit them on this repository.


### Adding new test data

Adding new test data means adding a PCAP file and associated VTP baseline files to
VeloView-TestData. It has to be saved under `TestData`. If you need a custom
calibration file, it has to be in the `share` directory. Then, edit
`generateTestData.py.in` in order to add your PCAP and its associated calibration
file to the list of tests data to generate.

Finaly re generate Veloview with cmake, in order to create a new 'generateTestData.py' file
and add a new test in the `CMakeList.txt`.

Don't forget to commit your change!


**Requirement for HDL-64 live calibration**: Your PCAP file need to have at
least 12480 packets for it to works. The live calibration mode computes the
calibration from appended received data packet and it's the minimum required
to compute it correctly (the rolling calibration data span 4160 datapacket, but
VeloView requires some redondancy to be on the safe side).

