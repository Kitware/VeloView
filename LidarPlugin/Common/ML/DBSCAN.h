//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Nick Laurenson
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
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
/**
 * @brief Implementation of the Density-Based Spacial Clustering of Applications with Noise
 * algorithm. Currently the
 */
template<class T>
class DBSCAN
{
public:
  DBSCAN(double epsilon, double minPts)
    : _epsilon(epsilon), _minPts(minPts) {}

  /**
   * @brief run the clustering and the return the label
   */
  std::vector<int> fit(std::vector<std::vector<T>> points);

  void setEpsilon(double value) { _epsilon = value; }
  void setMinPts(double value) { _minPts = value; }
  int getNbCluster() { return _nbCluster; }

private:
  void computeAdjacencyList();

  enum LABEL {
    UNDEFINED = -1,
    NOISE = 0
  };
  std::vector<std::vector<T>> _points;
  std::vector<std::vector<int>> _adjacencyList;
  int _nbCluster = 0;
  double _epsilon;
  int _minPts;
};


#include "DBSCAN.txx"
#endif // DBSCAN_H
