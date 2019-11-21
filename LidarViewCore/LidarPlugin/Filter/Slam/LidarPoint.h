#ifndef LIDARPOINT_H
#define LIDARPOINT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


struct EIGEN_ALIGN16 _PointXYZTIId
{
  PCL_ADD_POINT4D // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  double time;
  uint8_t intensity;
  uint8_t laserId;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const _PointXYZTIId& p);

/** \brief A point structure representing Euclidean xyz coordinates, time,  intensity, and laserId.
  * \ingroup common
  */
struct PointXYZTIId : public _PointXYZTIId
{
  inline PointXYZTIId (const _PointXYZTIId &p)
  {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = p.time;
    intensity = p.intensity;
    laserId = p.laserId;
  }

  inline PointXYZTIId ()
  {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    data[3] = 1.0f;
    time = 0.0;
    intensity = 0;
    laserId = 0;
  }

  friend std::ostream& operator << (std::ostream& os, const PointXYZTIId& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZTIId,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (double, time, time)
                                   (uint8_t, intensity, intensity)
                                   (uint8_t, laserId, laserId)
)
#endif // LIDARPOINT_H
