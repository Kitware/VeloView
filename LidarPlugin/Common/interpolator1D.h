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
#include <iterator>
#include <algorithm>
#include <string>
#include <fstream>

template<typename T>
class Interpolator1D
{
public:
  Interpolator1D()
  {
  }

  Interpolator1D(const std::vector<T>& t, const std::vector<T>& x)
  {
    this->t = t; // makes a copy
    this->x = x; // makes a copy
    assert(this->t.size() == this->x.size());
  }

  T Get(T time)
  {
    // the signal is clamped to 0.0 outside its support
    if (time < this->t[0] || time > this->t[this->t.size() - 1])
    {
      return 0.0;
    }

    auto lb = std::lower_bound(this->t.begin(), this->t.end(), time);
    int sup = std::distance(this->t.begin(), lb);
    int inf = sup - 1;
    assert(inf >= 0);
    assert(sup <= this->t.size() - 1);
    T alpha = (this->t[sup] - time) / (this->t[sup] - this->t[inf]);
    return alpha * this->x[inf] + (1.0 - alpha) * this->x[sup];
  }

  void ApplyTimeShift(T shift)
  {
    for (unsigned int i = 0; i < this->t.size(); i++)
    {
      this->t[i] = this->t[i] + shift;
    }
  }

  void ApplyValueShift(T shift)
  {
    for (unsigned int i = 0; i < this->x.size(); i++)
    {
      this->x[i] = this->x[i] + shift;
    }
  }

  void ApplyValueScale(T scale)
  {
    for (unsigned int i = 0; i < this->x.size(); i++)
    {
      this->x[i] = scale * this->x[i];
    }
  }

  T GetMinimumT()
  {
    return this->t[0];
  }


  T GetMaximumT()
  {
    return this->t[this->t.size() - 1];
  }

  T GetAveragePeriod()
  {
    return (this->GetMaximumT() - this->GetMinimumT()) / (this->t.size() - 1);
  }

  T Mean()
  {
    if (this->x.size() == 0)
    {
      return 0.0;
    }
    T sum = 0.0;
    for (unsigned int i = 0; i < this->x.size(); i++)
    {
      sum += x[i];
    }
    return sum / static_cast<T>(this->x.size());
  }

  void WriteToFile(const std::string& path)
  {
    std::ofstream file;
    file.open(path);
    file << "t,f(t)\n";
    for (unsigned int i = 0; i < t.size(); i++)
    {
      file << t[i] << "," << x[i] << "\n";
    }
    file.close();
  }

private:
  std::vector<T> t;
  std::vector<T> x;
};
