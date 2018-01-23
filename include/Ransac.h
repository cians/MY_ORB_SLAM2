#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "MapPoint.h"
namespace ORB_SLAM2
{
    using namespace std;
    struct Point3d
    {
        double x, y, z;
    };

    struct Point2d
    {
        double x, y;

    };
class MapPoint;

class Ransac
{
    public:
    Ransac(){};
    void RansacFitPlane(const vector< Point3d > &Data, const size_t num,const int minNum4fit,
                         const double distanceThreshold, double *outParams);
    void RansacFitLine(const vector<Point2d> &Data, const size_t num,const int minNum4fit,
                         const double distanceThreshold, double *outParams);
    void RansacFitPoint(const vector<double> &Data, const size_t num,const int minNum4fit,
                         const double distanceThreshold, double *outParams);
    //template <typename DataType,typename PlaneParamT>
    void RansacFitPlaneD(const vector<MapPoint*>& Data, const size_t num,
                         const Eigen::Vector3d PlaneABC,
                         const double distanceThreshold, double &outParams);
    void Point_fit(double* sample, double *result);
    void Plane_fit(MapPoint* sampleMap, Eigen::Vector3d PlaneABC, double &result);

    bool sample_degenerate(MapPoint* sample);

    private:

    int max_ransac_times_;
};

}//namespace myRansac end