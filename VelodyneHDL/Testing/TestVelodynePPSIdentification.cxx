#include "vtkVelodyneHDLPositionReader.h"

#include <iostream>
#include <iomanip>

#include "TestHelpers.h"

const double epsilon = 1e-3;
bool check(vtkSmartPointer<vtkVelodyneHDLPositionReader> reader,
           std::string& PCAP,
           bool PPSSynced,
           bool HasTimeshiftEstimation,
           double BaseTimeshift)
{
  reader->SetFileName(PCAP);
  reader->Modified();
  reader->Update();

  bool success = true;
  success &= (reader->GetPPSSynced() == PPSSynced);
  success &= (reader->GetHasTimeshiftEstimation() == HasTimeshiftEstimation);

  if (HasTimeshiftEstimation)
  {
    std::cout << std::fixed << std::setprecision(9)
              << "expected: " << BaseTimeshift
              << " got: " << reader->GetTimeshiftEstimation()
                 - reader->GetAssumedHardwareLag() << std::endl;
  }

  success &= !HasTimeshiftEstimation
                  || epsilon > std::abs((reader->GetTimeshiftEstimation()
                                         - reader->GetAssumedHardwareLag())
                                        - BaseTimeshift);
  return success;
}

int main(int argc, char* argv[])
{
  if (argc != 4)
  {
    std::cout << "Wrong number of arguments" << std::endl;
    return 1;
  }

  std::string PCAP_HDL32_no_PPS_sync = argv[1];
  std::string PCAP_VLP16_no_GPS_data = argv[2]; // (no GPS data implies no sync)
  std::string PCAP_VLP16_with_PPS_sync = argv[3];

  auto reader = vtkSmartPointer<vtkVelodyneHDLPositionReader>::New();

  bool success = true;

  // First check in some order:
  success &= check(reader, PCAP_HDL32_no_PPS_sync, false, true, 75599.1423);
  success &= check(reader, PCAP_VLP16_no_GPS_data, false, false, 0.0);
  success &= check(reader, PCAP_VLP16_with_PPS_sync, true, true, 68399.9062);

  // Then check that the reader is resetted correctly when file is changed:
  success &= check(reader, PCAP_VLP16_no_GPS_data, false, false, 0.0);

  return success ? 0 : 1;
}
