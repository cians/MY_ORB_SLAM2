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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    //显示所有的地图点（不包括局部地图点），大小为2个像素，黑色
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    //显示局部地图点，大小为2个像素，红色
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        if((*sit)->isGround)
        {
            glPointSize(mPointSize+1);
            glColor3f(1.0,0.0,0.0);
        }
        else
        {
            glColor3f(0.0,1.0,0.0);
            glPointSize(mPointSize);
        }
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::DrawGroundPlane(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float c = w*3;//横向
    const float k = w*15;//前进向
  //  const float y = 1;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif
    glLineWidth(mCameraLineWidth);
    
    // draw yellow as pseudo groundtruth
    // glColor4ub(255,255,0,105);
    // glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    // glBegin(GL_TRIANGLES);
    // glVertex3f(-c,y,0);
    // glVertex3f(-c,y,k);
    // glVertex3f(c,y,0);
    
    // glVertex3f(-c,y,k);
    // glVertex3f(c,y,k);
    // glVertex3f(c,y,0);
    // glEnd();
   // glPopMatrix();
    
    //1. 以Z = 0 起始刀，Z = k 终止刀
    //2. 以x = c 为右刀，x = -c 为左刀
    //去切AX+By+Cz +D =0
    // xz 平面俯视图：
    //  pc0,pk1    0     pc1,pk1
    //
    //
    //
    //  pc0,pk0    0     pc1,pk0

    if (mPlaneParams.dims ==2 && mPlaneParams.cols == 1 && mPlaneParams.rows == 4 && mPlaneParams.type() == CV_32F) //check if it has been initialized
    {
        //mPlaneParams 转化到第一帧的坐标系。
        //使用两个点来跟踪平面方程参数M，N
        cv::Mat fMat = mPlaneParams.clone();
        float planeN[3] = {0, -fMat.at<float>(3)/fMat.at<float>(1), 0};
        float planeM[3] = {fMat.at<float>(0), fMat.at<float>(1) - fMat.at<float>(3)/fMat.at<float>(1), fMat.at<float>(2)};
        cv::Mat N = cv::Mat(3, 1, CV_32F, planeN);
        cv::Mat M = cv::Mat(3, 1, CV_32F, planeM);
        cv::Mat fRcw = mCameraPose.rowRange(0,3).colRange(0,3).clone();
        cv::Mat ftcw = mCameraPose.rowRange(0,3).col(3).clone();
        cv::Mat M0 = fRcw* (M + ftcw);
        cv::Mat N0 = fRcw* (N + ftcw); 
        cv::Mat NM = M0 - N0;
        float D0 =0 -( NM.at<float>(0)*N0.at<float>(0) + NM.at<float>(1)*N0.at<float>(1) + NM.at<float>(2)*N0.at<float>(2) );
        float planeParams0[4] = {NM.at<float>(0), NM.at<float>(1), NM.at<float>(2), D0};
        cv::Mat fMat0 = cv::Mat(4, 1, CV_32F, planeParams0);

        float pk0 = 0;
        float pk1 = k;
        float pc0 = -c;
        float pc1 = c;

        float paraY0l = (-fMat0.at<float>(3) - fMat0.at<float>(0)*(pc0) - fMat0.at<float>(2)*pk0) / fMat0.at<float>(1);
        float paraY0r = (-fMat0.at<float>(3) - fMat0.at<float>(0)*(pc1) - fMat0.at<float>(2)*pk0) / fMat0.at<float>(1);
        float paraY1l = (-fMat0.at<float>(3) - fMat0.at<float>(0)*(pc0) - fMat0.at<float>(2)*pk1) / fMat0.at<float>(1);
        float paraY1r = (-fMat0.at<float>(3) - fMat0.at<float>(0)*(pc1) - fMat0.at<float>(2)*pk1) / fMat0.at<float>(1);
        // //printf("the y :%f %f \n",paraY0l,paraY1r);
        glColor4ub(0,255,255,105);
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glBegin(GL_TRIANGLES);
        glVertex3f(pc0,paraY0l,pk0);
        glVertex3f(pc1,paraY0r,pk0);
        glVertex3f(pc0,paraY1l,pk1);

        glVertex3f(pc1,paraY0r,pk0);
        glVertex3f(pc0,paraY1l,pk1);
        glVertex3f(pc1,paraY1r,pk1); 
        glEnd();
    }

    glPopMatrix();

}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::SetCurrentPlaneParams(const cv::Mat &m)
{
    unique_lock<mutex> lock(mMutexCamera);
    mPlaneParams = m.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
