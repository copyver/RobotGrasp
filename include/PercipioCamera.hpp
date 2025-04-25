//
// Created by yhlever on 24-7-18.
//

#ifndef ROBOTGRASP_PERCIPIOCAMERA_HPP
#define ROBOTGRASP_PERCIPIOCAMERA_HPP

#include "common.hpp"
#include "TYImageProc.h"

#define MAP_DEPTH_TO_COLOR  1

class PercipioCamera{
public:
    PercipioCamera(): hIface(nullptr), hDevice(nullptr), frameBuffer{nullptr, nullptr}, depthViewer("Depth") {
        init();
    }
    ~PercipioCamera() {
        cleanup();
    }
    std::pair<cv::Mat, cv::Mat> SimpleFetchColor_Depth();

private:
    const std::string ID = "207000119140";
    const std::string IP = "192.168.1.121";
    TY_INTERFACE_HANDLE hIface;
    TY_DEV_HANDLE hDevice;
    TY_ISP_HANDLE ispHandle{};
    char* frameBuffer[2];
    struct CallbackData {
        int             index;
        TY_ISP_HANDLE   IspHandle;
        TY_DEV_HANDLE   hDevice;
        DepthRender*    render;
        DepthViewer*    depthViewer;
        bool            needUndistort;
        float           scale_unit;
        bool            isTof;
        TY_CAMERA_CALIB_INFO depth_calib;
        TY_CAMERA_CALIB_INFO color_calib;
    };

    CallbackData cb_data{};
    DepthViewer depthViewer;
    DepthRender render;

    // 事件回调
    static void eventCallback(TY_EVENT_INFO *event_info, void *userdata);

    // 初始化
    void init();

    // 对齐
    static void
    doRegister(const TY_CAMERA_CALIB_INFO &depth_calib, const TY_CAMERA_CALIB_INFO &color_calib, const cv::Mat &depth,
               float f_scale_unit, const cv::Mat &color, bool needUndistort, cv::Mat &undistort_color,
               cv::Mat &out, bool map_depth_to_color);

    // 帧处理
    std::pair<cv::Mat, cv::Mat> handleFrame(TY_FRAME_DATA* frame, void* userdata);

    // 释放
    void cleanup();

    //数据格式转换 mat to TY_IMAGE_DATA
    static void mat2TY_IMAGE_DATA(int comp, const cv::Mat &mat, TY_IMAGE_DATA &data);

    //数据格式转换 cv pixel format to TY_PIXEL_FORMAT
    static int cvpf2typf(int cvpf);

    // info
    void printIntrinsic(const TY_CAMERA_INTRINSIC& intrinsic);
    void showCameraIntrinsics(const CallbackData& cb_data);

};

#endif //ROBOTGRASP_PERCIPIOCAMERA_HPP
