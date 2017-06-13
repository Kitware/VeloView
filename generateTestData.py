pcapFolder="/home/louis/develop/VeloView/Testing/Data/"
calibFolder="/home/louis/develop/VeloView/share/"

tests = {
  'HDL-64_Dual_10to20.pcap' : { 'xml' : '' },
  'HDL-64_Single_10to70.pcap' : { 'xml' : '' },
  'VLP-16_Dual_10to20.pcap' : { 'xml' : 'VLP-16' },
  'VLP-16_Single_10to20.pcap' : { 'xml' : 'VLP-16' },
  'VLP-32c_Dual_10to20.pcap' : { 'xml' : 'VLP-32c' },
  'VLP-32c_Single_10to20.pcap' : { 'xml' :'VLP-32c' }
}

for pcap, conf in tests.iteritems():
  vtpPath = pcapFolder + "/" + pcap[:-5] + "/"
  calibPath = (calibFolder + "/" + conf['xml'] + ".xml" if conf['xml'] else "")

  print pcap, calibPath, vtpPath + pcap[:-5] + ".vtp"

  reader = vv.smp.VelodyneHDLReader(guiName = 'Data',
                                 FileName = pcapFolder + "/" + pcap,
                                 CalibrationFile = calibPath,
                                 ApplyTransform = False,
                                 NumberOfTrailingFrames = 0,
                                 PointsSkip = 000)
  reader.GetClientSideObject().ReadFrameInformation()
  w = vv.smp.XMLPPolyDataWriter(FileName = vtpPath + pcap[:-5] + ".vtp",
                             Input = reader,
                             Writealltimestepsasfileseries = 1)
  w.UpdatePipeline()

# TODO: rename the files