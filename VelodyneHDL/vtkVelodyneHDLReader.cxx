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

#include "vtkVelodyneHDLReader.h"

#include "vtkNew.h"
#include "vtkSmartPointer.h"
#include "vtkCellData.h"
#include "vtkCellArray.h"
#include "vtkUnsignedCharArray.h"
#include "vtkPoints.h"
#include "vtkDoubleArray.h"
#include "vtkUnsignedShortArray.h"
#include "vtkDataArray.h"
#include "vtkFloatArray.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkMath.h"
#include "vtkStreamingDemandDrivenPipeline.h"

#include <sstream>
#include <algorithm>
#include <cmath>

#include <pcap.h>

//-----------------------------------------------------------------------------
class vtkVelodyneHDLReader::vtkInternal
{
public:

  vtkInternal()
  {
    this->Init();
  }

  std::vector<vtkSmartPointer<vtkPolyData> > Datasets;
  vtkSmartPointer<vtkPolyData> CurrentDataset;


  void SplitFrame();
  void LoadData(const std::string& filename);
  vtkSmartPointer<vtkPolyData> CreateData(vtkIdType numberOfPoints);
  vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);

  void LoadHDL32Corrections();
  void Init();
  void InitTables();
  void ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived);
};

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLReader);

//-----------------------------------------------------------------------------
vtkVelodyneHDLReader::vtkVelodyneHDLReader()
{
  this->Internal = new vtkInternal;
  this->FileName = 0;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
vtkVelodyneHDLReader::~vtkVelodyneHDLReader()
{
  this->SetFileName(0);
  delete this->Internal;
}


//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::RequestData(vtkInformation *request,
                              vtkInformationVector **inputVector,
                              vtkInformationVector *outputVector)
{
  this->UpdateProgress(0.0);
  vtkPolyData *output = vtkPolyData::GetData(outputVector);
  vtkInformation *info = outputVector->GetInformationObject(0);


  int timestep = 0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
    {
    double TimeStepsReq = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
    timestep = static_cast<int>(floor(TimeStepsReq));
    //printf("timestep request: %d\n", timestep);
    }

  if (timestep < 0 || timestep >= this->Internal->Datasets.size())
    {
    vtkErrorMacro("Cannot meet timestep request: " << timestep << ".  Have " << this->Internal->Datasets.size() << " datasets.");
    output->ShallowCopy(this->Internal->CreateData(0));
    return 0;
    }

  output->ShallowCopy(this->Internal->Datasets[timestep]);
  this->UpdateProgress(1.0);

  return 1;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::RequestInformation(vtkInformation *request,
                                     vtkInformationVector **inputVector,
                                     vtkInformationVector *outputVector)
{
  vtkInformation *outInfo = outputVector->GetInformationObject(0);

  if (this->FileName)
    {
    this->Internal->LoadData(this->FileName);
    }

  const int numberOfTimesteps = static_cast<int>(this->Internal->Datasets.size());

  printf("number of timesteps: %d\n", numberOfTimesteps);

  const int maxTimestep = (numberOfTimesteps > 0) ? numberOfTimesteps - 1 : 0;
  std::vector<double> timesteps;
  for (size_t i = 0; i < this->Internal->Datasets.size(); ++i)
    {
    timesteps.push_back(i);
    }

  double timeRange[2] = {0, maxTimestep};
  outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
  outInfo->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);

  return 1;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "FileName: "
     << (this->FileName ? this->FileName : "(NULL)") << endl;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::CanReadFile(const char *fname)
{
  return 1;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::ProcessHDLPacket(unsigned char *data, unsigned int bytesReceived)
{
  this->Internal->ProcessHDLPacket(data, bytesReceived);
}

//-----------------------------------------------------------------------------
std::vector<vtkSmartPointer<vtkPolyData> >& vtkVelodyneHDLReader::GetDatasets()
{
  return this->Internal->Datasets;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SplitFrame()
{
  this->Internal->SplitFrame();
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::vtkInternal::CreateData(vtkIdType numberOfPoints)
{
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // points
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(numberOfPoints);
  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(NewVertexCells(numberOfPoints));

  // intensity
  vtkNew<vtkUnsignedCharArray> intensity;
  intensity->SetName("intensity");
  intensity->SetNumberOfTuples(numberOfPoints);
  polyData->GetPointData()->AddArray(intensity.GetPointer());

  // laser number
  vtkNew<vtkUnsignedCharArray> laserId;
  laserId->SetName("laser_id");
  laserId->SetNumberOfTuples(numberOfPoints);
  polyData->GetPointData()->AddArray(laserId.GetPointer());

  // azimuth
  vtkNew<vtkUnsignedShortArray> azimuth;
  azimuth->SetName("azimuth");
  azimuth->SetNumberOfTuples(numberOfPoints);
  polyData->GetPointData()->AddArray(azimuth.GetPointer());

  // range
  vtkNew<vtkUnsignedShortArray> distance;
  distance->SetName("distance");
  distance->SetNumberOfTuples(numberOfPoints);
  polyData->GetPointData()->AddArray(distance.GetPointer());

  return polyData;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> vtkVelodyneHDLReader::vtkInternal::NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

namespace
{

#define HDL_Grabber_toRadians(x) ((x) * M_PI / 180.0)

const int HDL_NUM_ROT_ANGLES = 36000;
const int HDL_LASER_PER_FIRING = 32;
const int HDL_MAX_NUM_LASERS = 64;
const int HDL_FIRING_PER_PKT = 12;

enum HDLBlock
{
  BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
};

#pragma pack(push, 1)
typedef struct HDLLaserReturn
{
  unsigned short distance;
  unsigned char intensity;
} HDLLaserReturn;
#pragma pack(pop)

struct HDLFiringData
{
  unsigned short blockIdentifier;
  unsigned short rotationalPosition;
  HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
};

struct HDLDataPacket
{
  HDLFiringData firingData[HDL_FIRING_PER_PKT];
  unsigned int gpsTimestamp;
  unsigned char blank1;
  unsigned char blank2;
};

struct HDLLaserCorrection
{
  double azimuthCorrection;
  double verticalCorrection;
  double distanceCorrection;
  double verticalOffsetCorrection;
  double horizontalOffsetCorrection;
  double sinVertCorrection;
  double cosVertCorrection;
  double sinVertOffsetCorrection;
  double cosVertOffsetCorrection;
};

struct HDLRGB
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

double *cos_lookup_table_;
double *sin_lookup_table_;
HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
HDLRGB laser_rgb_mapping_[HDL_MAX_NUM_LASERS];


void PushFiringData(vtkPolyData* polyData, unsigned char laserId, unsigned short azimuth, HDLLaserReturn laserReturn, HDLLaserCorrection correction)
{
  double cosAzimuth, sinAzimuth;
  if (correction.azimuthCorrection == 0)
  {
    cosAzimuth = cos_lookup_table_[azimuth];
    sinAzimuth = sin_lookup_table_[azimuth];
  }
  else
  {
    double azimuthInRadians = HDL_Grabber_toRadians ((static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
    cosAzimuth = std::cos (azimuthInRadians);
    sinAzimuth = std::sin (azimuthInRadians);
  }
  double distanceM = laserReturn.distance * 0.002 + correction.distanceCorrection;

  double xyDistance = distanceM * correction.cosVertCorrection - correction.sinVertOffsetCorrection;

  double x = (xyDistance * sinAzimuth - correction.horizontalOffsetCorrection * cosAzimuth);
  double y = (xyDistance * cosAzimuth + correction.horizontalOffsetCorrection * sinAzimuth);
  double z = (distanceM * correction.sinVertCorrection + correction.cosVertOffsetCorrection);
  unsigned char intensity = laserReturn.intensity;

  if (vtkMath::IsNan(x)
      || vtkMath::IsNan(y)
      || vtkMath::IsNan(z))
    {
    return;
    }

  if (std::fabs(x) > 1000
      || std::fabs(y) > 1000
      || std::fabs(z) > 1000)
    {
    return;
    }



  vtkUnsignedCharArray::SafeDownCast(polyData->GetPointData()->GetArray("intensity"))->InsertNextValue(intensity);
  vtkUnsignedCharArray::SafeDownCast(polyData->GetPointData()->GetArray("laser_id"))->InsertNextValue(laserId);
  vtkUnsignedShortArray::SafeDownCast(polyData->GetPointData()->GetArray("azimuth"))->InsertNextValue(azimuth);
  vtkUnsignedShortArray::SafeDownCast(polyData->GetPointData()->GetArray("distance"))->InsertNextValue(laserReturn.distance);
  polyData->GetPoints()->InsertNextPoint(x,y,z);
}

}


//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::InitTables()
{
  if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL)
    {
    cos_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
    sin_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
      {
      double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
      cos_lookup_table_[i] = std::cos (rad);
      sin_lookup_table_[i] = std::sin (rad);
      }
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::LoadHDL32Corrections()
{
  double hdl32VerticalCorrections[] = {
    -30.67, -9.3299999, -29.33, -8, -28,
    -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67,
    -1.33, -21.33, 0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
    -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };

  for (int i = 0; i < HDL_LASER_PER_FIRING; i++)
    {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = hdl32VerticalCorrections[i];
    laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
    laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
    }

  for (int i = HDL_LASER_PER_FIRING; i < HDL_MAX_NUM_LASERS; i++)
    {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = 0.0;
    laser_corrections_[i].sinVertCorrection = 0.0;
    laser_corrections_[i].cosVertCorrection = 1.0;
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::Init()
{
  this->InitTables();
  this->LoadHDL32Corrections();

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
    {
    HDLLaserCorrection correction = laser_corrections_[i];
    laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.sinVertCorrection;
    laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.cosVertCorrection;
    }


  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
    laser_rgb_mapping_[i].r = laser_rgb_mapping_[i].g = laser_rgb_mapping_[i].b = 0;

  if (laser_corrections_[32].distanceCorrection == 0.0)
    {
    for (int i = 0; i < 16; i++)
      {
      laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 6 + 64);
      laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 6 + 64);
      }
    }
  else
    {
    for (int i = 0; i < 16; i++)
      {
      laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 3 + 64);
      laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 3 + 64);
      }
    for (int i = 0; i < 16; i++)
      {
      laser_rgb_mapping_[i * 2 + 32].b = static_cast<uint8_t> (i * 3 + 160);
      laser_rgb_mapping_[i * 2 + 33].b = static_cast<uint8_t> ( (i + 16) * 3 + 160);
      }
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::SplitFrame()
{
  if (!this->CurrentDataset || !this->CurrentDataset->GetNumberOfPoints())
    {
    return;
    }

  this->CurrentDataset->SetVerts(this->NewVertexCells(this->CurrentDataset->GetNumberOfPoints()));
  this->Datasets.push_back(this->CurrentDataset);
  this->CurrentDataset = 0;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived)
{
  if (bytesReceived != 1206)
    {
    return;
    }

  if (sizeof (HDLLaserReturn) != 3)
    {
    return;
    }

  HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket *>(data);

  static unsigned int last_azimuth_ = 0;
  static unsigned int packetCounter = 0;

  for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
    {
    HDLFiringData firingData = dataPacket->firingData[i];
    int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

    if (firingData.rotationalPosition > 18000 && last_azimuth_ < 18000
        && ((last_azimuth_ - firingData.rotationalPosition) > 18000)
        && this->CurrentDataset && this->CurrentDataset->GetNumberOfPoints())
      {
      //printf("azimuths: %d %d\n", firingData.rotationalPosition, last_azimuth_);
      this->SplitFrame();
      }
    last_azimuth_ = firingData.rotationalPosition;

    if (!this->CurrentDataset)
      {
      this->CurrentDataset = this->CreateData(0);
      }

    for (int j = 0; j < HDL_LASER_PER_FIRING; j++)
      {
      unsigned char laserId = static_cast<unsigned char>(j + offset);
      PushFiringData(this->CurrentDataset, laserId, firingData.rotationalPosition, firingData.laserReturns[j], laser_corrections_[j + offset]);
      }
    }

  //if ((++packetCounter % 170) == 0)
  //  {
  //  this->SplitFrame();
  //  }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::LoadData(const std::string& filename)
{

  printf("reading file: %s\n", filename.c_str());

  struct pcap_pkthdr *header;
  const unsigned char *data;
  char errbuff[PCAP_ERRBUF_SIZE];

  pcap_t *pcap = pcap_open_offline (filename.c_str (), errbuff);

  struct bpf_program filter;
  std::ostringstream stringStream;

  stringStream << "udp ";


  // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
  if (pcap_compile (pcap, &filter, stringStream.str ().c_str(), 0, 0xffffffff) == -1)
    {
    vtkGenericWarningMacro("pcap_compile message: " << pcap_geterr(pcap));
    }
  else if (pcap_setfilter(pcap, &filter) == -1)
    {
    vtkGenericWarningMacro("pcap_setfilter message: " << pcap_geterr(pcap));
    }

  struct timeval lasttime;
  unsigned long long uSecDelay;

  lasttime.tv_sec = 0;

  int returnValue = pcap_next_ex(pcap, &header, &data);

  while (returnValue >= 0)
    {

    if (lasttime.tv_sec == 0)
      {
      lasttime.tv_sec = header->ts.tv_sec;
      lasttime.tv_usec = header->ts.tv_usec;
      }

    if (lasttime.tv_usec > header->ts.tv_usec)
      {
      lasttime.tv_usec -= 1000000;
      lasttime.tv_sec++;
      }

    uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
                (header->ts.tv_usec - lasttime.tv_usec);

    //boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay));

    lasttime.tv_sec = header->ts.tv_sec;
    lasttime.tv_usec = header->ts.tv_usec;

    // The ETHERNET header is 42 bytes long; unnecessary
    this->ProcessHDLPacket(const_cast<unsigned char*>(data) + 42, header->len - 42);

    returnValue = pcap_next_ex(pcap, &header, &data);
    }


}
