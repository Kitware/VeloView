#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkMath.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkRansacPlaneModel.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedIntArray.h>

#include <iostream>
#include <iomanip>

#include <Eigen/Dense>


int main(int argc, char* argv[])
{
    srand((unsigned int) time(0));

    int N = 1000;
    int N_OUTLIERS = 10;
    float noise_sigma = 0.1;
    Eigen::MatrixXf angles = Eigen::MatrixXf::Random(1, N);
    angles.array() += 1;
    angles.array() *= 3.1415;
    Eigen::MatrixXf radius = Eigen::MatrixXf::Random(1, N);
    radius.array() *= 10;
    Eigen::Matrix3Xf pts(3, N);

    // add noise on z
    Eigen::MatrixXf noisy_z = Eigen::MatrixXf::Random(1, N);
    noisy_z.array() *= noise_sigma;

    // add outliers
    for (int i = N - N_OUTLIERS; i < N; ++i)
    {
        auto v = noisy_z(i);
        if (v < 0) v = 10 * v - 5;
        else v = 10 * v + 5;
        noisy_z(i) = v;
    }
    pts << radius.array().cwiseProduct(angles.array().cos()), radius.array().cwiseProduct(angles.array().sin()), noisy_z;

    // transform the initial plane
    float initial_rot_X = static_cast<float>(std::rand() % 100) / 10.0;
    float initial_rot_Y = static_cast<float>(std::rand() % 100) / 10.0;
    float initial_z_translation = -static_cast<float>(std::rand() % 100) / 10.0;
    Eigen::AngleAxisf rotX(vtkMath::RadiansFromDegrees(initial_rot_X), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotY(vtkMath::RadiansFromDegrees(initial_rot_Y), Eigen::Vector3f::UnitY());
    Eigen::Translation3f trans(0, 0, initial_z_translation);
    Eigen::Affine3f T = rotX * rotY * trans;
    pts = T * pts;

    // build polydata
    auto polydata = vtkSmartPointer<vtkPolyData>::New();
    auto array = vtkSmartPointer<vtkFloatArray>::New();
    array->SetNumberOfComponents(3);
    array->SetNumberOfTuples(N);
    float temp_f[3];
    for (int i = 0; i < N; ++i)
    {
        temp_f[0] = pts.col(i).x();
        temp_f[1] = pts.col(i).y();
        temp_f[2] = pts.col(i).z();
        array->SetTuple(i, temp_f);
    }
    auto new_points = vtkSmartPointer<vtkPoints>::New();
    new_points->SetData(array);
    polydata->SetPoints(new_points);

    // apply filter ransac plane model
    auto filter = vtkSmartPointer<vtkRansacPlaneModel>::New();
    filter->SetAlignOutput(1);
    filter->SetInputData(polydata);
    filter->Update();
    auto output = filter->GetOutput();

    // check that the points are correctly aligned to XY
    double temp_d[3];
    double max_z_threshold = 1.0;       // threshold on z for points with small noise
    for (int i = 0; i < N - N_OUTLIERS; ++i)
    {
        output->GetPoint(i, temp_d);
        if (std::abs(temp_d[2]) > max_z_threshold)
        {
            std::cout << "Error: point " << i << " has z = " << temp_d[2] << std::endl;
            return -1;
        }

    }

    // check that outliers are detected as outliers
    vtkSmartPointer<vtkUnsignedIntArray> inliers_array = 
    vtkUnsignedIntArray::SafeDownCast(output->GetPointData()->GetArray("ransac_plane_inliers"));
    for (int i =  N - N_OUTLIERS; i < N; ++i)
    {
        if (inliers_array->GetValue(i) == 1)
        {
            std::cout << "Error: Point " << i << " should be classified as outliers" << std::endl;
            return -1;
        }
    }

    return 0;
}
