#pragma once

#include<iostream>
#include<vector>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/opencv.hpp"

#include "DataProcess.hpp"

typedef std::vector<std::vector<cv::Point>> Contours;

struct GrabResult {
    cv::Point2f center;
    float x;
    float y;
    float z;
    float rx;
    float ry;
    float rz;
};

// 抓取点图像处理及数据
class GrabData {
public:
    // 2D
    static void Pretreatment(cv::Mat &srcImage);            //图像预处理

    cv::Mat getCP(cv::Mat &srcImage, cv::Mat &ImageDealt);    //获取抓取点center和phi

    static cv::Mat findLargesrArea(cv::Mat srcImage);    //获取最大连通域

    static float get_phi(cv::Point2f p1, cv::Point2f p2);    // 计算抓取方向

    // 将图像坐标转换为世界坐标
    static cv::Point2f ImgtRob(float k1, float k2, float k3, float k4, float b1, float b2, cv::Point ImCenter);  //二维
    cv::Point3f convertItoW(cv::Point3f ImagePoint);  //三维

    static float getDepth(const cv::Mat& depthImage, cv::Rect depthRoi);    // 获取深度

    static void PreAllmasks(std::vector<cv::Mat> &masksVec); // 预处理所有掩膜

    static std::vector<int> getArea(std::vector<cv::Mat> &masksVec);  //存放所有掩膜像素面积

    static void ShrinkRotatedRect(cv::RotatedRect &rect, double alpha);

    static void updateDepth(cv::Mat &depth);         // 更新深度图（按划定区域）,为了点云

    static Contours getContours(cv::Mat mask);           //获得掩膜轮廓

    static std::vector<Contours> saveAllContours(std::vector<cv::Mat> &masksVec, int maskNum);   //定义一个存放所有掩膜轮廓的容器

    cv::Rect ExtractTargetArea(Contours contours, const cv::Mat& outColor);     //提取目标区域

    static cv::Rect rectCenterScale(cv::Rect rect, cv::Size size);     //围绕矩形中心缩放

    static cv::Mat ColorFilter(cv::Mat outDepth, cv::Mat outColor);    // 采用深度方法过滤原彩图

    static double perpendicularDistance(const cv::Point2f &A, const cv::Point2f &B, const cv::Point2f &C);  //点c到向量ab的距离

    // 6DoF
    std::vector<Eigen::Matrix4d> computeBHos(const std::vector<Eigen::Matrix4d> &cHos);

    Eigen::Matrix4d computeBHo(const Eigen::Matrix4d &cHo);

    Eigen::Matrix4d computeBHg(const Eigen::Matrix4d &bHo);

    static Eigen::Matrix4d computeBHg2(const Eigen::Matrix4d &bHo);

    static std::vector<double> matrixToPose(const Eigen::Matrix4d &T);

    static void drawSegmentsOnImage(const cv::Mat &image,
                                    const std::vector<std::vector<cv::Point2f>> &segments,
                                    std::vector<cv::Point2f> &centers,
                                    const cv::Scalar &outlineColor = cv::Scalar(204, 255, 196),
                                    const cv::Scalar &fillColor = cv::Scalar(204, 255, 196),
                                    int outlineThickness = 2,
                                    double alpha = 0.3);

    cv::Mat imagePointToCamera(const cv::Mat &uvd);

    cv::Mat cameraPointToImage(float x, float y, float z);

    cv::Mat cameraPointToImage(const cv::Mat &xyz);

    cv::Mat cameraPointToBase(const cv::Mat &xyz);

    static void setTranslation(Eigen::Matrix4d &cHo, const cv::Mat &xyz);

    void drawObjectPose(cv::Mat &image, const Eigen::Matrix4d &cHo, double axisLength = 20);

    static Eigen::Matrix4d computeNewcHo(const Eigen::Matrix4d &cHo);

    static int chooseEffectorAmp(std::string &className);

    static int chooseEffectorZOffset(std::string &className);

    cv::Point2f getHandleCenter(std::vector<cv::Point2f> contour, const cv::Mat &color);

    static Eigen::Matrix4d adjustObjectPose(
            const cv::Mat &image,
            const std::vector<cv::Point2f> &contourPoints,
            const Eigen::Matrix4d &T_obj_to_cam);

    static Eigen::Matrix4d adjustObjectPoseWithRect(
            const cv::Mat &image,
            const std::vector<cv::Point2f> &contourPoints,
            const Eigen::Matrix4d &T_obj_to_cam);


public:
    GrabResult m_result;


private:

    //变换矩阵
    cv::Mat m_Rotation = (cv::Mat_<float>(3, 3) <<
                                                0.0152687771719423, 1.01193821800274, -0.0670457472434118,
            1.00488918620605, -0.0139786167061463, -0.0106444365114153,
            -0.0289868230220798, -0.00667871932235755, -1.06236019286833);
    cv::Mat m_Translation = (cv::Mat_<float>(3, 1) <<
                                                   505.365565524215, 39.2582276409764, 1388.82825076893);

    std::vector<float> m_R0 = {
            0.0f, 0.0f, 1.0f,
            -1.0f, 0.0f, 0.0f,
            0.0f, -1.0f, 0.0f
    };

    cv::Mat m_cameraIntrinsic = (cv::Mat_<float>(3, 3) <<
                                                       1090.08, 0, 661.26,
            0, 1089.79, 469.49,
            0, 0, 1.00);


    float m_depth_intri[3][3] = {1062.67, 0, 646.17,
                                 0, 1062.67, 474.24,
                                 0, 0, 1.00};
    float m_color_instri[3][3] = {1090.08, 0, 661.26,
                                  0, 1089.79, 469.49,
                                  0, 0, 1.00};

    Eigen::Matrix4d m_bHc =
            (Eigen::Matrix4d() <<
                               -0.000738244, -0.998949, 0.0458212, 469.9,
                    -0.99989, 0.00141737, 0.0147905, 52.2929,
                    -0.01484, -0.0458052, -0.99884, 1107.01,
                    0, 0, 0, 1
            ).finished();  // ca1

    Eigen::Matrix4d m_oHg =
            (Eigen::Matrix4d() <<
                               -0.0223043, -0.9994506, 0.0245150, 50.2075796,
                    0.9981294, -0.0208653, 0.0574653, 16.6334746,
                    -0.0569222, 0.0257509, 0.9980465, -128.751472,
                    0, 0, 0, 1.00).finished();
};
