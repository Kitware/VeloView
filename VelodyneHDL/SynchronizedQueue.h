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
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVelodyneHDLReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef SYNCHRONIZEDQUEUE_H
#define SYNCHRONIZEDQUEUE_H

#include <queue>
#include <boost/thread.hpp>

/**
 * @brief The SynchronizedQueue class is a FIFO structure whith some mutex to allow acces
 * from multiple thread.
 */
template<typename T>
class SynchronizedQueue
{
public:
  SynchronizedQueue()
    : queue_()
    , mutex_()
    , cond_()
    , request_to_end_(false)
    , enqueue_data_(true)
  {
  }

  void enqueue(const T &data)
  {
    boost::unique_lock<boost::mutex> lock(mutex_);

    if (enqueue_data_)
    {
      queue_.push(data);
      cond_.notify_one();
    }
  }

  bool dequeue(T &result)
  {
    boost::unique_lock<boost::mutex> lock(mutex_);

    while (queue_.empty() && (!request_to_end_))
    {
      cond_.wait(lock);
    }

    if (request_to_end_)
    {
      doEndActions();
      return false;
    }

    result = queue_.front();
    queue_.pop();

    return true;
  }

  void stopQueue()
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    request_to_end_ = true;
    cond_.notify_one();
  }

  unsigned int size()
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    return static_cast<unsigned int>(queue_.size());
  }

  bool isEmpty() const
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    return (queue_.empty());
  }

private:
  void doEndActions()
  {
    enqueue_data_ = false;

    while (!queue_.empty())
    {
      queue_.pop();
    }
  }

  std::queue<T> queue_;            // Use STL queue to store data
  mutable boost::mutex mutex_;     // The mutex to synchronise on
  boost::condition_variable cond_; // The condition to wait for

  bool request_to_end_;
  bool enqueue_data_;
};

#endif // SYNCHRONIZEDQUEUE_H
