folder = "D:/KITTI/2011_09_26_drive_0001_sync/velodyne_points/data"
kittiReader = LidarKITTIDataSetReader(guiName="KITTI", FileName = folder)

# update app status
vv.app.reader = kittiReader
vv.updateSliderTimeRange()
vv.enablePlaybackActions()
vv.updateUIwithNewFrame()

# enable GUI options:
vv.app.actions['actionLaunchSlam'].enabled = True
