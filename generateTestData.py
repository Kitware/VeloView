import os, re

# edit this line with the path to VeloView sources directory
sourceDir = ""
pcapFolder= sourceDir + "/Testing/Data/"
calibFolder= sourceDir + "/share/"

# Dictionnary containing the different *.pcap to use for test data generation
tests = {
  'HDL-64_Dual_10to20.pcap' : { 'xml' : '' },
  'HDL-64_Single_10to70.pcap' : { 'xml' : '' },
  'VLP-16_Dual_10to20.pcap' : { 'xml' : 'VLP-16' },
  'VLP-16_Single_10to20.pcap' : { 'xml' : 'VLP-16' },
  'VLP-32c_Dual_10to20.pcap' : { 'xml' : 'VLP-32c' },
  'VLP-32c_Single_10to20.pcap' : { 'xml' :'VLP-32c' }
}

# Generates *.vtp files from *.pcap
for pcap, conf in tests.iteritems():
  print "Generates *.vtp for " + pcap[:-5]

  vtpPath = pcapFolder + pcap[:-5] + "/"
  calibPath = (calibFolder + "/" + conf['xml'] + ".xml" if conf['xml'] else "")

  reader = vv.smp.VelodyneHDLReader(guiName = 'Data',
                                 FileName = pcapFolder + "/" + pcap,
                                 CalibrationFile = calibPath,
                                 ApplyTransform = False,
                                 NumberOfTrailingFrames = 0,
                                 FiringsSkip = 000)
  reader.GetClientSideObject().ReadFrameInformation()
  w = vv.smp.XMLPPolyDataWriter(FileName = vtpPath + pcap[:-5] + ".vtp",
                             Input = reader,
                             Writealltimestepsasfileseries = 1)
  w.UpdatePipeline()

  # Renames file that need to be renamed
  files = os.listdir(vtpPath)
  files.sort()

  for filename in files:
    pattern = r"_([0-9])_0"
    repl = r'_0\1_0'
    newFilename = re.sub(pattern, repl, filename)
    if filename != newFilename:
      os.rename(vtpPath+filename, vtpPath+newFilename)

  files = os.listdir(vtpPath)
  files.sort()

  # Generates a list with the different *.vtp files in alphabetical order
  vtpListPath = vtpPath + pcap[:-5] + ".txt"
  vtpList = open(vtpListPath, "w")

  re.purge()

  for filename in files:
    pattern = r"_[0-9]{2}_0\.vtp"
    if re.search(pattern, filename) > 0:
      vtpList.write("%s\n" % filename)

  vtpList.close()