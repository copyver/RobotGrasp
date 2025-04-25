//
// Created by yhlever on 24-12-26.
//

#ifndef ROBOTGRASP_POSEMODEL_HPP
#define ROBOTGRASP_POSEMODEL_HPP

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Python.h>
#include <unistd.h>
#include <climits>
#include <iomanip>
#include <cstring>
#include <string>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <numpy/arrayobject.h>
#include <Eigen/Dense>


struct ModelPoseResult {
    std::vector<std::vector<float>> predRs;  // 用于存储 pred_Rs 的二维数组
    std::vector<std::vector<float>> predTs;  // 用于存储 pred_Ts 的二维数组
    std::vector<Eigen::Matrix4d> cHos;
    std::vector<float> poseScores;           // 用于存储 pred_scores 的一维数组
    std::vector<float> segScores;
    std::vector<float> addMetrics;
    std::vector<std::string> classNames;
    std::vector<int> classIds;
    std::vector<std::vector<cv::Point2f>> segments;
    float segTime;
    float poseTime;
};

class ModelPose {
public:
    ModelPose();

    ~ModelPose();

    PyObject *loadModel();

    bool runPoseEstimation(PyObject *pFunc, cv::Mat colorImage, cv::Mat depthImage);

    ModelPoseResult result;

    void printPoseResult();


private:
    PyObject *matToNumpy(const cv::Mat &mat);

    bool parseModelPoseResults(PyObject *pResult);
};

#endif
