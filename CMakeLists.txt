# Copyright 2013 Velodyne Acoustics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20.3 FATAL_ERROR)
project(LidarView)
#Thanks to ExternalProject CMAKE_SOURCE_DIR will correctly be the same as this PROJECT_SOURCE_DIR

# add path to get all the needed modules used to config Lidarview
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/LVCore/CMake")

# we include the module that allow us to configure the Lidarview project
include(SetupLidarviewProject)

# custom VeloView here
# ...

# Must be invoked Last
include(FinalizeLidarViewProject)

