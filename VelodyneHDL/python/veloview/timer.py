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
