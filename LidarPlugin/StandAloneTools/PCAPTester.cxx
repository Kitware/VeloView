#include "vtkLidarReader.h"
#include "vtkVelodynePacketInterpreter.h"
#include "vtkVelodyneHDLPositionReader.h"
#include "statistics.h"

#include "vtkFieldData.h"
#include "vtkPointData.h"

#include <boost/algorithm/string.hpp>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <chrono>
#include <regex>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <sstream>

// source: https://stackoverflow.com/a/16606128
template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

// source: https://stackoverflow.com/a/5100745
template< typename T >
std::string int_to_hex(T i, bool pad = false)
{
  std::stringstream stream;
  stream << "0x";
  if (pad)
  {
    stream << std::setfill ('0') << std::setw(sizeof(T)*2);
  }
  stream << std::hex << i;
  return stream.str();
}

// source: https://stackoverflow.com/a/6039648
long GetFileSize(std::string filename)
{
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

// source: https://stackoverflow.com/a/23625419
std::string bytes_to_human_readable(long bytes) {
    if (bytes < 1024)
    {
      return std::to_string(bytes) + " Bytes";
    }
    else if (bytes < 1048576)
    {
      return to_string_with_precision((bytes / 1024), 3) + " KB";
    }
    else if (bytes < 1073741824)
    {
      return to_string_with_precision((bytes / 1048576), 3) + " MB";
    }
    else
    {
      return to_string_with_precision((bytes / 1073741824), 3) + " GB";
    }
};



int main(int argc, char* argv[])
{
  const std::string sep = ",";
  bool pretty = false;
  bool showCSVHeader = false;
  bool showPCAPPath = false;
  po::options_description visible("Allowed options");
  visible.add_options()
      ("help", "produce help message")
      ("pretty", po::bool_switch(&pretty), "Pretty-print columns")
      ("showCSVHeader", po::bool_switch(&showCSVHeader), "Print CSV header line then exit")
      ("pcapFile", po::value<std::string>()->default_value(""), "PCAP File")
      ("calibrationFile", po::value<std::string>()->default_value(""), "Calibration File")
      ("showPCAPPath", po::bool_switch(&showPCAPPath), "First column is PCAP Path")
      ;

  po::options_description cmdline_options;
  cmdline_options.add(visible);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
            options(cmdline_options).run(), vm);
  po::notify(vm);

  if (vm.count("help") || argc < 2) {
      cout << "Usage: " << argv[0] << " [options]" << std::endl;
      cout << visible << "\n"; // shows usage
      std::cout << "Tip: it can be interesting to use a calibration even if it"
                   " is not appropriate to the sensor which produced the PCAP,"
                   " so that we can parse a few frames (this tool does not care"
                   " about the positions of the 3D points)."
                << std::endl;
      return 1;
  }

  if (showCSVHeader)
  {
    if (showPCAPPath)
    {
      std::cout << "file" << sep;
    }
    // must be kept in sync with printing order the values below
    std::cout << "modelString" << sep;
    std::cout << "FF2" << sep;
    std::cout << "dualReturnString," << sep;
    std::cout << "FF1" << sep;
    std::cout << "DualR" << sep;
    std::cout << "RPMs" << sep;
    std::cout << "PPSSynced" << sep;
    std::cout << "TSEstim" << sep;
    std::cout << "Lat:" << sep;
    std::cout << "Lon:" << sep;
    std::cout << "Frames " << sep;
    std::cout << "Size " << sep;
    std::cout << "HRSize " << sep;
    std::cout << "Pts/frame";
    std::cout << std::endl;
    return 0;
  }

  // TODO: detect if velodyne packet
  std::string pcapPath = vm["pcapFile"].as<std::string>();
  std::string calibrationPath = vm["calibrationFile"].as<std::string>();
  bool haveCalibration = calibrationPath != "";

  auto reader = vtkSmartPointer<vtkLidarReader>::New();
  auto interp = vtkSmartPointer<vtkVelodynePacketInterpreter>::New();
  reader->SetInterpreter(interp);
  reader->SetFileName(pcapPath);
  if (haveCalibration)
  {
    reader->SetCalibrationFileName(calibrationPath);
  }
  reader->Modified();
  reader->Update();

  // variables we would like to fill:
  int frameCount = 0;
  int factoryField1 = 0;
  std::string dualReturnString = "?";
  int factoryField2 = 0;
  std::string modelString = "?";
  double RPM = 0.0;
  double pointsPerFrame = 0.0;

  bool isPPSSynced = false;
  bool hasGPSTimeShiftEstimation = false;

  // Read information provided by the Lidar Packets presents inside the PCAP:
  frameCount = reader->GetNumberOfFrames();
  if (frameCount != 0 && haveCalibration)
  {
    std::vector<double> RPMs;
    std::vector<double> pointsPerFrames;
    vtkPolyData* currentFrame = nullptr;
    for (int i = 0; i < std::min(frameCount, 10); i++)
    {
      reader->Open();
      currentFrame = reader->GetFrame(i);
      RPMs.push_back(currentFrame->GetFieldData()->GetArray("RotationPerMinute")->GetTuple1(0));
      pointsPerFrames.push_back(currentFrame->GetNumberOfPoints());
      reader->Close();
    }
    RPM = ComputeMedian(RPMs);
    pointsPerFrame = ComputeMedian(pointsPerFrames);

  }

  std::string sensorInfo = reader->GetSensorInformation();
  static const std::regex parseSensorInfo = std::regex(".*:(.*)\\(.*\\)(.*)\\|.*:(.*)\\(.*\\)(.*)");
  std::smatch matches;

  if(std::regex_search(sensorInfo, matches, parseSensorInfo)
                  && matches.size() == 5)
  {
    factoryField1 = std::stoi(matches[1]);
    dualReturnString = matches[2];
    boost::trim(dualReturnString);
    factoryField2 = std::stoi(matches[3]);
    modelString = matches[4];
    boost::trim(modelString);
  }

  // Read information provided by the GPS/IMU Packets presents inside the PCAP:
  auto position_reader = vtkSmartPointer<vtkVelodyneHDLPositionReader>::New();
  position_reader->SetFileName(pcapPath);
  position_reader->Modified();
  position_reader->Update();
  isPPSSynced = position_reader->GetPPSSynced();
  hasGPSTimeShiftEstimation = position_reader->GetHasTimeshiftEstimation();

  bool haveLatLon = false;
  double lat = 0.0;
  double lon = 0.0;
  if (position_reader->GetOutput()->GetPointData()->GetArray("lat") != nullptr
    && position_reader->GetOutput()->GetPointData()->GetArray("lat")->GetNumberOfTuples() > 0
    && position_reader->GetOutput()->GetPointData()->GetArray("lon") != nullptr
    && position_reader->GetOutput()->GetPointData()->GetArray("lon")->GetNumberOfTuples() > 0)
  {
    haveLatLon = true;
    lat = position_reader->GetOutput()->GetPointData()->GetArray("lat")->GetTuple1(0);
    lon = position_reader->GetOutput()->GetPointData()->GetArray("lon")->GetTuple1(0);
  }

  // GetHasDualReturn() returns a correct value only after some frames have
  // been read.
  if (showPCAPPath)
  {
    std::cout << (pretty ? "Path:" : "") << "\"" << pcapPath << "\"" << sep;
  }

  const int latLongPrecision = 4; // 4 digits give 11m of precision
  // must be kept in sync with printing of CSV header line above
  std::cout << (pretty ? "Model:" : "") << modelString << sep;
  std::cout << (pretty ? "FF2:" : "") << int_to_hex(factoryField2) << sep;
  std::cout << (pretty ? "Returns:" : "") << dualReturnString << sep;
  std::cout << (pretty ? "FF1:" : "") << int_to_hex(factoryField1) << sep;
  std::cout << (pretty ? "DualR:" : "") << (haveCalibration ? ((interp->GetHasDualReturn() ? "True" : "False")) : "?") << sep;
  std::cout << (pretty ? "RPMs:" : "") << (haveCalibration ? to_string_with_precision(RPM, 1) : "?") << sep;
  std::cout << (pretty ? "PPSSynced:" : "") << (isPPSSynced ? "True" : "False") << sep;
  std::cout << (pretty ? "TSEstim:" : "") << (hasGPSTimeShiftEstimation ? "True" : "False") << sep;
  std::cout << (pretty ? "Lat:" : "") << (haveLatLon ? to_string_with_precision(lat, latLongPrecision) : "None") << sep;
  std::cout << (pretty ? "Lon:" : "") << (haveLatLon ? to_string_with_precision(lon, latLongPrecision) : "None") << sep;
  std::cout << (pretty ? "Frames:" : "") << std::to_string(frameCount) << sep;
  std::cout << (pretty ? "Size:" : "") << GetFileSize(pcapPath) << sep;
  std::cout << (pretty ? "HRSize:" : "") << bytes_to_human_readable(GetFileSize(pcapPath)) << sep;
  std::cout << (pretty ? "Pts/frame:" : "") << (haveCalibration ? to_string_with_precision(pointsPerFrame, 1) : "?");
  std::cout << std::endl;
  return 1;
}
