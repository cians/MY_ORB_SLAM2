/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Eigen>
#include <vector>
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "Ransac.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"

namespace ORB_SLAM2
{
// !! 0.88 is set to const and used in 2 place : one is used in optimizing pose in optimizer.h,
// another is used in Tracking.cc to initialize Frame's mPlaneParams in StereoInitialization()
const double kGounrd2Camera = 0.88;
class Ransac;
class LoopClosing;
//拟合平面需要的顶点和边
class VertexParam4Plane: public g2o::BaseVertex<4, Eigen::Vector4d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexParam4Plane(){};
    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual void setToOriginImpl()
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }
    //增量函数，增量为传进的参数updat
    virtual void oplusImpl(const double *update)
    {
        //将C++数组转化为eigen的一个类。
        Eigen::Vector4d::ConstMapType v(update);
        _estimate += v;
    }

};
class UnaryEdgeMapPoint2Plane0 : public g2o::BaseUnaryEdge<1, Eigen::Vector3d, VertexParam4Plane>
{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      UnaryEdgeMapPoint2Plane0(){};
      virtual bool read(std::istream& /*is*/)
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }

      virtual bool write(std::ostream& /*os*/) const
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }

      void computeError()
      {
          const VertexParam4Plane *params = static_cast<const VertexParam4Plane*>(vertex(0));
          const double& a = params->estimate()(0);
          const double& b = params->estimate()(1);
          const double& c = params->estimate()(2);
          const double& d = params->estimate()(3);
          double res = a*measurement()(0) + b*measurement()(1) + c*measurement()(2) + d;
          _error(0) = abs(res)/sqrt(a*a +b*b +c*c);
      }

}; 
class BinaryEdgeMapPoint2Plane0 : public g2o::BaseBinaryEdge<1, double, VertexParam4Plane,  g2o::VertexSBAPointXYZ>
{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      BinaryEdgeMapPoint2Plane0(){};
      virtual bool read(std::istream& /*is*/)
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }

      virtual bool write(std::ostream& /*os*/) const
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }

      void computeError()
      {
          const VertexParam4Plane *params = dynamic_cast<const VertexParam4Plane*>(vertex(0));
          const g2o::VertexSBAPointXYZ *CaPt = dynamic_cast<const  g2o::VertexSBAPointXYZ*>(vertex(1)); 
          const double& a = params->estimate()(0);
          const double& b = params->estimate()(1);
          const double& c = params->estimate()(2);
          const double& d = params->estimate()(3);
          double res = a*CaPt->estimate()(0) + b*CaPt->estimate()(1) + c*CaPt->estimate()(2) + d;
          _error(0) = measurement()*abs(res)/sqrt(a*a +b*b +c*c);
      }

};  

class BinaryEdgePose2MapPoint : public g2o::BaseBinaryEdge<1, Eigen::Vector3d,  g2o::VertexSE3Expmap, g2o::VertexSBAPointXYZ>
{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      BinaryEdgePose2MapPoint(){};
      virtual bool read(std::istream& /*is*/)
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }

      virtual bool write(std::ostream& /*os*/) const
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }
      void computeError()
      {
          const g2o::VertexSE3Expmap *pPose = dynamic_cast<const  g2o::VertexSE3Expmap*>(vertex(0)); 
          g2o::SE3Quat CamPose = pPose->estimate();
          Eigen::Matrix<double,4,4> eigMat = CamPose.to_homogeneous_matrix(); 
          Eigen::Vector3d center = -eigMat.block<3,3>(0,0).inverse()*eigMat.block<3,1>(0,3);
          const g2o::VertexSBAPointXYZ *pMapPoint = dynamic_cast<const  g2o::VertexSBAPointXYZ*>(vertex(1)); 
          Eigen::Vector3d center2MapP = center - pMapPoint->estimate();
          double cast_distance = center2MapP.dot(measurement().normalized());
          _error(0) = abs(cast_distance) - kGounrd2Camera;
      }

};
//use UnaryEdge because poseOptimization doesn’t optimizate the map points.  use mapPoint+vector as measuerment.
class UnaryEdgePose2MapPoint : public g2o::BaseUnaryEdge<1, Eigen::Matrix<double, 6, 1>,  g2o::VertexSE3Expmap>
{
      public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      UnaryEdgePose2MapPoint(){};
      virtual bool read(std::istream& /*is*/)
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }

      virtual bool write(std::ostream& /*os*/) const
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }
      void computeError()
      {
          const g2o::VertexSE3Expmap *pPose = dynamic_cast<const  g2o::VertexSE3Expmap*>(vertex(0)); 
          g2o::SE3Quat CamPose = pPose->estimate();
          Eigen::Matrix<double,4,4> eigMat = CamPose.to_homogeneous_matrix(); 
          Eigen::Vector3d center = -eigMat.block<3,3>(0,0).inverse()*eigMat.block<3,1>(0,3); 
          Eigen::Vector3d center2MapP = center - measurement().segment(0,3);
          Eigen::Vector3d vect = measurement().segment(3,3);
          double cast_distance = center2MapP.dot(vect.normalized());
          _error(0) = abs(cast_distance) - kGounrd2Camera;
      }
};
//measurement ke当做权重
class EdgePose2Plane : public g2o::BaseBinaryEdge<1, double, VertexParam4Plane,  g2o::VertexSE3Expmap>
{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgePose2Plane(){};
      virtual bool read(std::istream& /*is*/)
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }

      virtual bool write(std::ostream& /*os*/) const
      {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
      }

      void computeError()
      {
          const VertexParam4Plane *params = dynamic_cast<const VertexParam4Plane*>(vertex(0));
          const g2o::VertexSE3Expmap *CaPt = dynamic_cast<const  g2o::VertexSE3Expmap*>(vertex(1)); 
          g2o::SE3Quat CamPose = CaPt->estimate();
          Eigen::Matrix<double,4,4> eigMat = CamPose.to_homogeneous_matrix(); 
          Eigen::Vector3d center = -eigMat.block<3,3>(0,0).inverse()*eigMat.block<3,1>(0,3);
          const double& a = params->estimate()(0);
          const double& b = params->estimate()(1);
          const double& c = params->estimate()(2);
          const double& d = params->estimate()(3);
          double res = a*center(0) + b*center(1) + c*center(2) + d;
          res = abs(res)/sqrt(a*a +b*b +c*c);
          _error(0) = res - kGounrd2Camera;
      }

};  
class VertexParam1Plane: public g2o::BaseVertex<1, double>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexParam1Plane(){};
    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual void setToOriginImpl()
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }
    //增量函数，增量为传进的参数updat
    virtual void oplusImpl(const double *update)
    {
        //将C++数组转化为eigen的一个类。
        //Eigen::Vector4d::ConstMapType v(update);
        _estimate += *update;
    }

};
class EdgeMapPoint1Plane : public g2o::BaseUnaryEdge<1, Eigen::Matrix<double,6,1>, VertexParam1Plane>
{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeMapPoint1Plane(){};
    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

        void computeError()
        {
            const VertexParam1Plane *params = static_cast<const VertexParam1Plane*>(vertex(0));
            const double& d = params->estimate();
            const double& a = measurement()(3);
            const double& b = measurement()(4);
            const double& c = measurement()(5);
            _error(0) = a*measurement()(0) + b*measurement()(1) + c*measurement()(2) + d;
            //_error(0) = b*measurement()(1) + d;
        }

}; 

class EdgeLinellPlane : public g2o::BaseUnaryEdge<1, Eigen::Vector3d, VertexParam4Plane>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeLinellPlane(){};
    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

        void computeError()
        {
            const VertexParam4Plane *params = static_cast<const VertexParam4Plane*>(vertex(0));
            const double& a = params->estimate()(0);
            const double& b = params->estimate()(1);
            const double& c = params->estimate()(2);
            double res = a*measurement()(0) + b*measurement()(1) + c*measurement()(2);
            _error(0) = res;
        }
};
//D 为error information 的维度。E为measurement的类型。
class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
