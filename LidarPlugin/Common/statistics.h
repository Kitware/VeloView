//=========================================================================
//
// Copyright 2019 Kitware, Inc.
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
//=========================================================================

#include <vector>

// source: https://stackoverflow.com/questions/1719070
template<typename T>
double ComputeMedian(std::vector<T> &v)
{
  size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  T vn = v[n];
  if(v.size() % 2 == 1)
  {
    return vn;
  }
  else
  {
    std::nth_element(v.begin(), v.begin() + n - 1, v.end());
    return 0.5 * (vn + v[n-1]);
  }
}

