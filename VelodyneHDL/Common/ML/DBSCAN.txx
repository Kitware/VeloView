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
#include "DBSCAN.h"

#include <nanoflann.hpp>
#include <KDTreeVectorOfVectorsAdaptor.h>


//-----------------------------------------------------------------------------
template<class T>
std::vector<int> DBSCAN<T>::fit(std::vector<std::vector<T>> points)
{
  _points = points;
  computeAdjacencyList();
  std::vector<int> label(_points.size(), LABEL::UNDEFINED);
  int currentClusterID = 0;
  for (int i = 0; i < _points.size(); ++i)
  {
    if (label[i] != LABEL::UNDEFINED)
    {
      continue;
    }
    if (_adjacencyList[i].size() < _minPts)
    {
      label[i] = LABEL::NOISE;
      continue;
    }

    currentClusterID++;
    label[i] = currentClusterID;
    std::vector<int> clusterPointIdx = _adjacencyList[i];
    for (int j = 0; j < clusterPointIdx.size(); ++j )
    {
      int& currentPointLabel = label[clusterPointIdx[j]];
      if (currentPointLabel == LABEL::NOISE)
      {
        currentPointLabel = currentClusterID;
      }
      if (currentPointLabel != LABEL::NOISE && currentPointLabel != LABEL::UNDEFINED)
      {
        continue;
      }
      currentPointLabel = currentClusterID;
      std::vector<int> neighborsOfNeighborsIdx = _adjacencyList[clusterPointIdx[j]];
      if (neighborsOfNeighborsIdx.size() > _minPts)
      {
        // append new point to the end of the cluster Points
        for (int idx : neighborsOfNeighborsIdx)
        {
          if (label[idx] == LABEL::UNDEFINED)
          {
            clusterPointIdx.push_back(idx);
          }
        }
      }
    }
  }

  _nbCluster= currentClusterID;
  return label;
}

//-----------------------------------------------------------------------------
template<class T>
void DBSCAN<T>::computeAdjacencyList()
{
  _adjacencyList.clear();
  _adjacencyList.resize(_points.size());
  if (_points.empty())
  {
    return;
  }
  typedef std::vector<std::vector<double> > my_vector_of_vectors_t;
  typedef KDTreeVectorOfVectorsAdaptor< my_vector_of_vectors_t, double > my_kd_tree_t;

  my_kd_tree_t mat_index(-1 /*dim*/, _points, 10 /* max leaf */ );
  mat_index.index->buildIndex();

  std::vector<std::pair<size_t,double> > ret_matches;
  nanoflann::SearchParams params;
  params.sorted = false;

  for (int i = 0; i < _points.size(); ++i)
  {
    const size_t nMatches = mat_index.index->radiusSearch(_points[i].data(), _epsilon, ret_matches, params);
    for (int j = 0; j < nMatches; ++j)
    {
      _adjacencyList[i].push_back(ret_matches[j].first);
    }
  }
}
