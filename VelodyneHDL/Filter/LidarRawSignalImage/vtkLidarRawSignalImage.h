/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkLidarRawSignalImage.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef VTK_LIDAR_RAW_SIGNAL_IMAGE_H
#define VTK_LIDAR_RAW_SIGNAL_IMAGE_H

#include <vector>

#include <vtkImageAlgorithm.h>

class vtkTable;

/**
 * @brief The vtkLidarRawSignalImage class makes a cylindrical projection
 * of a point cloud to create a panorama image.
 *
 * @warning one image column corresponds to one laser.
 */
class VTK_EXPORT vtkLidarRawSignalImage : public vtkImageAlgorithm
{
public:
  static vtkLidarRawSignalImage *New();
  vtkTypeMacro(vtkLidarRawSignalImage, vtkImageAlgorithm)
  void PrintSelf(ostream& os, vtkIndent indent);

  //! @{
  //! @copydoc Width
  vtkGetMacro(Width, int)
  vtkSetMacro(Width, int)
  //! @}

  //! @{
  //! @copydoc Spacing
  vtkGetVector3Macro(Spacing, double)
  vtkSetVector3Macro(Spacing, double)
  //! @}

  //! @{
  //! @copydoc Origin
  vtkGetVector3Macro(Origin, double)
  vtkSetVector3Macro(Origin, double)
  //! @}

  //! @{
  //! @copydoc Origin
  vtkGetMacro(Scale, double)
  vtkSetMacro(Scale, double)
  //! @}

protected:
  vtkLidarRawSignalImage();
  int FillInputPortInformation(int port, vtkInformation *info) override;
  int RequestInformation(vtkInformation *request, vtkInformationVector **inputVector, vtkInformationVector *outputVector) override;
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) override;

  // Initiate filter parameters
  // using the input sensor calibration
  bool InitializationFromCalibration(vtkTable* calibration);

  // permutation to map laser index
  // from firing index to vertical
  // ordered index
  std::vector<int> VerticallySortedIndex;

  double VerticalFOV = 30.0;
  double HorizontalFOV = 360.0;

  //! Height of the output image
  //! automatically computed depending
  //! on the calibration table provided
  int Height = 64;
  //! Width of the output image
  int Width = 1080;
  //! Spacing of the output image
  double Spacing[3] = {1,1,1};
  //! Origin of the output image
  double Origin[3] = {0,0,0};
  ///! Scale of the image
  double Scale = 1.0;

private:
  vtkLidarRawSignalImage(const vtkLidarRawSignalImage&) = delete;
  void operator=(const vtkLidarRawSignalImage&) = delete;
};

#endif // VTK_LIDAR_RAW_SIGNAL_IMAGE_H
