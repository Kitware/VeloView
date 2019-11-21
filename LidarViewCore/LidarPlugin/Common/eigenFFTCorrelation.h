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

#include <unsupported/Eigen/FFT>
#include <cmath>

// This function was desgined to have the same output as
// scipy.signal.fftconvolve
// Uses Eigen to do the FFT transforms (2 forward, 1 backward).
template<typename T>
std::vector<T> fftconvolve(const std::vector<T>& a,
                           const std::vector<T>& b)
{
  assert(a.size() > 0 && b.size() > 0);
  int outSize = a.size() + b.size() - 1;
  int fshape = std::pow(2, std::ceil(std::log2(outSize)));

  std::vector<T> a_padded = std::vector<T>(fshape, 0.0);
  for (unsigned int i = 0; i < a.size(); i++)
  {
    a_padded[i] = a[i];
  }

  std::vector<T> b_padded = std::vector<T>(fshape, 0.0);
  for (unsigned int i = 0; i < b.size(); i++)
  {
    b_padded[i] = b[i];
  }

  Eigen::FFT<T> fft;
  std::vector<std::complex<T>> a_fwd;
  fft.fwd(a_fwd, a_padded);
  std::vector<std::complex<T>> b_fwd;
  fft.fwd(b_fwd, b_padded);

  std::vector<std::complex<T>> prod =
                  std::vector<std::complex<T>>(fshape);
  for (int i = 0; i < fshape; i++)
  {
    prod[i] = a_fwd[i] * b_fwd[i];
  }

  std::vector<std::complex<T>> inv_fft;
  fft.inv(inv_fft, prod);

  std::vector<T> out = std::vector<T>(outSize);
  for (int i = 0; i < outSize; i++)
  {
    out[i] = inv_fft[i].real();
  }

  return out;
}


// This function was designed to have the same output as
// scipy.signal.correlate when mode='full' and method='fft'
template<typename T>
std::vector<T> fftcorrelate(const std::vector<T>& a,
                            const std::vector<T>& b)
{
  // reverse and conjugate
  std::vector<T> b_reversed = std::vector<T>(b.size());
  for (unsigned int i = 0; i < b.size(); i++)
  {
    // real number so no need to conjugate
    b_reversed[i] = b[b.size() - 1 - i];
  }
  return fftconvolve(a, b_reversed);
}


// Compute the shift between a and b, in number of samples.
// The shift returned is in b.
// The signals must have the same sampling rate
// and should have the same mean.
template<typename T>
int max_fftcorrelation(const std::vector<T>& a,
                       const std::vector<T>& b)
{
  std::vector<T> corr = fftcorrelate(a, b);
  return std::distance(corr.begin(), std::max_element(corr.begin(), corr.end()))
      - b.size() + 1;
}

