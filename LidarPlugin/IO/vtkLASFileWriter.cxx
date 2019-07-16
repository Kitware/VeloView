#include "vtkTemporalTransforms.h"
#include "vtkLASFileWriter.h"

#include "vtkTransform.h"
#include "vtkNew.h"
#include <vtkObjectFactory.h>
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include <iostream>

#include "vtkConversions.h"
#include "vtkStreamingDemandDrivenPipeline.h"

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkLASFileWriter)
vtkLASFileWriter::vtkLASFileWriter()
{
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::vtkLASFileWriter" << std::endl;
#endif
  // documentation of vtkDataObjectAlgorithm tells us that the number of output
  // ports must be changed from the default (1)
  this->SetNumberOfOutputPorts(0);
}

//-----------------------------------------------------------------------------
vtkLASFileWriter::~vtkLASFileWriter()
{
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::~vtkLASFileWriter" << std::endl;
#endif
  if (this->FileName)
  {
    delete[] this->FileName;
  }
}

//------------------------------------------------------------------------------
int vtkLASFileWriter::Write()
{
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::Write" << std::endl;
#endif
  if (this->GetNumberOfInputConnections(0) < 1)
  {
    vtkErrorMacro("No input provided!");
    return 0;
  }

  this->LASWriter.SetWriteSRS(this->WriteSRS);
  this->LASWriter.SetWriteColor(this->WriteColor);
  this->LASWriter.Open(this->FileName);

  bool useLatLonForOut = this->ExportType == EXPORT_LATLONG;
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "useLatLonForOut: " << useLatLonForOut << std::endl;
#endif
  if (useLatLonForOut)
  {
    this->LASWriter.SetPrecision(1e-8, 1e-3); // 1e-8 degrees give approx 1mm
  }
  else
  {
    this->LASWriter.SetPrecision(1e-3, 1e-3);
  }
  this->LASWriter.SetGeoConversionUTM(this->InOutSignedUTMZone, useLatLonForOut);
  // Mind the order for this the parameters for SetOrigin
  // This is a bit strange, but the exports seem to be correct.
  // Strange because if the input is ENU then this corresponds to switching to
  // a left handed referential (two axis are switched without flipping the
  // third).
  // Mind that SetOrigin will use this->LASWriter->OutGcsEPSG to write the
  // LAS header
  this->LASWriter.SetOrigin(Offset[1], Offset[0], Offset[2]);

  // the call to Modified() does not seems required, but it is done in
  // vtkFileSeriesWriter
  this->Modified();
  // the call to Update() is required, this is what triggers the writting
  // by way of running the pipeline
  this->Update();

  return 1;
}

//------------------------------------------------------------------------------
void vtkLASFileWriter::Modified()
{
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::Modified" << std::endl;
#endif
  this->Superclass::Modified();
}

void vtkLASFileWriter::Update()
{
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::Update" << std::endl;
#endif
  this->Superclass::Update();
}

int vtkLASFileWriter::ProcessRequest(
    vtkInformation* request,
    vtkInformationVector** inputVector,
    vtkInformationVector* outputVector)
{
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::ProcessRequest" << std::endl;
#endif
  return this->Superclass::ProcessRequest(request, inputVector, outputVector);
}

//------------------------------------------------------------------------------
int vtkLASFileWriter::RequestInformation(vtkInformation* vtkNotUsed(request),
                                         vtkInformationVector** inputVector,
                                         vtkInformationVector* vtkNotUsed(outputVector))
{
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::RequestInformation" << std::endl;
#endif
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  if (!inInfo->Has(vtkStreamingDemandDrivenPipeline::TIME_STEPS()))
  {
    vtkErrorMacro("There are no time steps, not writing anything")
    return 0;
  }

  this->NumberOfFrames = inInfo->Length(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::RequestInformation: Seen "
            << this->NumberOfFrames << " frames." << std::endl;
#endif
  if (this->LastFrame < 0)
  {
    this->LastFrame += this->NumberOfFrames;
  }

  if (this->FirstFrame < 0
      || this->FirstFrame > this->LastFrame
      || this->LastFrame > this->NumberOfFrames - 1)
  {
    vtkErrorMacro("Incorrect frame interval requested, not writing anything");
    return 0;
  }

  this->CurrentFrame = this->FirstFrame;

  if (this->SkipMetaDataPass)
  {
#ifdef DEBUG_VTKLASFILEWRITER
    std::cout << "vtkLASFileWriter::RequestInformation: skipping metadata pass"
              << std::endl;
#endif
    this->CurrentPass = 1;
    const double zero[3] = {0.0, 0.0, 0.0};
    this->LASWriter.SetMinPt(zero);
    this->LASWriter.SetMaxPt(zero);
  }

  return 1;
}

//------------------------------------------------------------------------------
int vtkLASFileWriter::RequestUpdateExtent(vtkInformation* vtkNotUsed(request),
                                          vtkInformationVector** inputVector,
                                          vtkInformationVector* vtkNotUsed(outputVector))
{
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::RequestUpdateExtent" << std::endl;
#endif

  double* inTimes = inputVector[0]
                  ->GetInformationObject(0)
                  ->Get(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
  if (!inTimes)
  {
    vtkErrorMacro("No time information, not writing anything");
    return 0;
  }

  const double currentTime = inTimes[this->CurrentFrame];
  inputVector[0]
      ->GetInformationObject(0)
      ->Set(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP(), currentTime);

  return 1;
}

//-----------------------------------------------------------------------------
int vtkLASFileWriter::RequestData(vtkInformation *request,
                                  vtkInformationVector **inputVector,
                                  vtkInformationVector *vtkNotUsed(outputVector))
{
  const int framesPerPass = (this->LastFrame - this->FirstFrame + 1);
  const int stepsToDo = (this->SkipMetaDataPass ? 1 : 2) * framesPerPass;
  const int actualPass = this->SkipMetaDataPass ? this->CurrentPass - 1 : this->CurrentPass;
  const int stepsDone = actualPass * framesPerPass + (this->CurrentFrame - this->FirstFrame);
  this->UpdateProgress(static_cast<double>(stepsDone) / static_cast<double>(stepsToDo));
#ifdef DEBUG_VTKLASFILEWRITER
  std::cout << "vtkLASFileWriter::RequestData"
            << " pass: " << this->CurrentPass
            << ", frame: " << this->CurrentFrame
            << ", last frame: " << this->LastFrame
            << std::endl;
#endif

  if (this->CurrentFrame == this->FirstFrame &&
      (this->CurrentPass == 0
       || (this->CurrentPass == 1 && this->SkipMetaDataPass)))
  {
    request->Set(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING(), 1);
#ifdef DEBUG_VTKLASFILEWRITER
    std::cout << "Setting CONTINUE_EXECUTING" << std::endl;
#endif
    this->Start = std::chrono::steady_clock::now();
  }

  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *polyData = vtkPolyData::SafeDownCast(
  inInfo->Get(vtkDataObject::DATA_OBJECT()));
  if (polyData == nullptr)
  {
    vtkErrorMacro("Input is not a vtkPolyData");
    return 0;
  }

  if (this->CurrentPass == 0)
  {
    this->LASWriter.UpdateMetaData(polyData);
  }
  else if (this->CurrentPass == 1)
  {
    this->LASWriter.WriteFrame(polyData);
  }
  else
  {
    vtkErrorMacro("Unknown pass");
    return 0;
  }

  // End of RequestData():
  this->CurrentFrame += this->FrameStride; // next "candidate" CurrentFrame
  if (this->CurrentFrame > this->LastFrame)
  {
    if (this->CurrentPass == this->PassCount - 1)
    {
      request->Remove(vtkStreamingDemandDrivenPipeline::CONTINUE_EXECUTING());
      this->End = std::chrono::steady_clock::now();
      double dt = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(this->End - this->Start).count();
      std::cout << "Exported LAS in " << dt << " seconds" << std::endl;
    }
    else
    {
      this->CurrentFrame = this->FirstFrame;
      this->CurrentPass++;
      this->LASWriter.FlushMetaData();
    }
  }
  return 1;
}
