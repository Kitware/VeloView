// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VTK_TESTING_MACROS_H
#define VTK_TESTING_MACROS_H

#define ASSERT_EQUALS(X, Y)                                             \
  if((X) != (Y))                                                        \
    {                                                                   \
    std::cerr << "For " << #X << " == " << #Y << std::endl;             \
    std::cerr << "ERROR: " << (X) << " != " << (Y) << std::endl;        \
    std::cerr << __FILE__ << " at " << __LINE__ << std::endl;           \
    return 1;                                                           \
    }                                                                   \


#endif
