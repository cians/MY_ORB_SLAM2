#include "Ransac.h"
#include "Converter.h"
namespace ORB_SLAM2
{
//printf color control
    #define NONE                 "\e[0m"
    #define BLACK                "\e[0;30m"
    #define L_BLACK              "\e[1;30m"
    #define RED                  "\e[0;31m"
    #define L_RED                "\e[1;31m"
    #define GREEN                "\e[0;32m"
    #define L_GREEN              "\e[1;32m"
    #define BROWN                "\e[0;33m"
    #define YELLOW               "\e[1;33m"
    #define BLUE                 "\e[0;34m"
    #define L_BLUE               "\e[1;34m"
    #define PURPLE               "\e[0;35m"
    #define L_PURPLE             "\e[1;35m"
    #define CYAN                 "\e[0;36m"
    #define L_CYAN               "\e[1;36m"
    #define GRAY                 "\e[0;37m"
    #define WHITE                "\e[1;37m"

    #define BOLD                 "\e[1m"
    #define UNDERLINE            "\e[4m"
    #define BLINK                "\e[5m"
    #define REVERSE              "\e[7m"
    #define HIDE                 "\e[8m"
    #define CLEAR                "\e[2J"
void Ransac::RansacFitPoint(const vector<double>& Data, const size_t num, const int minNum4fit,
                         const double distanceThreshold, double *outParams)
{
    // assert(Data.size() > minNum4fit);
    // assert(Data.size() < 10000); //浪费空间了
    // //printf

    // //shared_ptr<int> pSelect_block = new int [num];
    // //改用vector吧
    // shared_ptr<double> sample = new double[minNum4fit];//double may be point3d /2d
    // //double outParams;
    // max_ransac_times_ = 100;
    // int best_model_score = 0;
    // for(int i = 0; i < max_ransac_times_; ++i )
    // {
    //     //memset (pSelect_block, 0, num*sizeof(double));
    //     //一遍拟合 
    //     shared_ptr<double> testResult;
    //     bool degenerate = true;
    //     while (degenerate)
    //     {
    //         //随机抽取
    //         int indexList[minNum4fit];
    //         memset(indexList, -1, sizeof(int)*minNum4fit);
    //         for(int j = 0; j < minNum4fit; ++j)
    //         {
    //             int select_index = rand() % num;
    //             for(int t = 0; t < j; ++t)   // 不取重复的
    //             {
    //                 if (indexList[t] == select_index)
    //                 {
    //                     --j;
    //                     continue;
    //                 }
    //             }
    //             sample[j] = Data[select_index];// respect it is filled with select sample
    //             indexList[j] = select_index;
    //         }
    //         degenerate = sample_degenerate(sample);
    //         if (!degenerate)
    //         {
    //             Point_fit(sample,testResult);
    //             //需要存储每个sample的分数到循环外

    //             double test_model = testResult[0]; // 本是循环
    //             int model_score = 0;
    //             for (size_t i = 0; i < Data.size(); ++i)
    //             {
    //                 double dist = Data[i] - test_model; // fit 函数
    //                 if (dist < distanceThreshold)
    //                      ++model_score;
    //             }

    //             if (model_score > best_model_score)
    //             {
    //                 best_model_score = model_score;
    //                 outParams[0] = testResult[0]; //本是循环
    //             }
    //             degenerate = true;
    //         }
    //     }

    // }
}
void Ransac::Point_fit(double* sample, double *result)
{
	//can do more ...
		result[0] = sample[0];
}
void Ransac::RansacFitPlaneD(const vector<MapPoint*>& Data, const size_t num, Eigen::Vector3d PlaneABC,
                         const double distanceThreshold, double &outParams)
{
    //assert(Data.size() > 1);
    if (Data.size() < 4)
    {
        cout<<"no enough points to fit a plane\n";
        return;
    }
    assert(Data.size() < 1000); //浪费空间了
    assert(Data.size() == num);
    //printf

    double resultParamsD;
    max_ransac_times_ = 200;
    int best_model_score = 0;
    //memset (pSelect_block, 0, num*sizeof(double));
    //一遍拟合 
    double testResult;
    bool degenerate = true;
    int run_times = 0;
    //printf("test Data valid if :%lf  ", Data[0]->GetWorldPos().at<float>(0));
    while (degenerate && run_times < max_ransac_times_)
    {
        //随机抽取
        int select_index = rand() % num;
       // MapPoint* sample = Data[select_index];// respect it is filled with select sample
        degenerate = sample_degenerate(Data[select_index]);
        if (!degenerate)
        {
            Plane_fit(Data[select_index], PlaneABC, testResult);
            //需要存储每个sample的分数到循环外
            double test_model = testResult; // multi则是循环
            int model_score = 0;
            for (size_t i = 0; i < Data.size(); ++i)
            {
                Eigen::Vector3d Data_ie = Converter::toVector3d(Data[i]->GetWorldPos());
                double dist = Data_ie(0) * PlaneABC(0) + Data_ie(1) * PlaneABC(1) + Data_ie(2) * PlaneABC(2) + test_model; // fit 函数
                if (abs(dist) < distanceThreshold)
                        ++model_score;
            }
            //printf("=%d= ", model_score);
            if (model_score > best_model_score)
            {
                best_model_score = model_score;
                resultParamsD = testResult; //multi则是循环
            }
            degenerate = true;
            ++run_times;
        }
    }
    outParams = resultParamsD;
    //check threshold is very good
    //printf("--%lf-- ",PlaneABC(1));
    // for (size_t i = 0; i < Data.size(); ++i)
    // {
    //     Eigen::Vector3d Data_ie = Converter::toVector3d(Data[i]->GetWorldPos());
    //     double dist = Data_ie(0) * PlaneABC(0) +Data_ie(1) * PlaneABC(1) + Data_ie(2) * PlaneABC(2) + outParams; // fit 函数
    //     if (abs(dist) < distanceThreshold)
    //     {
    //         // printf( GREEN " ==%7lf==" NONE,dist);
    //         Data[i]->isGround = true;
    //     }
    //     else
    //         Data[i]->isGround = false;
    //     // else if (abs(dist) > 2*distanceThreshold)
    //     //     printf ( RED " ==%7lf==" NONE,dist);
    //     // else
    //     //     printf (NONE " ==%7lf==" NONE,dist);
    // }
}

void Ransac::Plane_fit(MapPoint* sampleMap, Eigen::Vector3d PlaneABC, double &result)
{
	//Ax +By +Cz +d  = 0
    Eigen::Vector3d sample = Converter::toVector3d(sampleMap->GetWorldPos());
	result = 0 - (sample(0)*PlaneABC(0) + sample(1)*PlaneABC(1) + sample(2)*PlaneABC(2));
}
bool Ransac::sample_degenerate(MapPoint* sample)
{
    
    return false;
}
} // namespace myRansac