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
class timer(object):

    def __init__(self):
        self.restart()

        self.Alpha = 0.9
        self.TimeWindow = 1.0
        self.AverageFPS = 1.0
        self.FramesThisWindow = 0
        self.LastUpdateTime = self.WindowStartTime = self.currentSeconds()

    def currentSeconds(self):
        return time.time()

    def elapsed(self):
        return self.currentSeconds() - self.startTime

    def restart(self):
        self.startTime = self.currentSeconds()

    def update(self):

        self.LastUpdateTime = self.currentSeconds()
        self.FramesThisWindow += 1

        elapsedTime = self.LastUpdateTime - self.WindowStartTime
        if elapsedTime > self.TimeWindow:

          # compute FPS for this time window
          averageFPSThisWindow = self.FramesThisWindow / elapsedTime

          # update moving average
          self.AverageFPS = self.Alpha * averageFPSThisWindow + (1.0 - self.Alpha) * self.AverageFPS

          # reset counters
          self.WindowStartTime = self.LastUpdateTime
          self.FramesThisWindow = 0
