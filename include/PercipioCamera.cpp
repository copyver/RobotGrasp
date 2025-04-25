
#include "PercipioCamera.hpp"

//
// Created by yhlever on 24-7-18.
//

void PercipioCamera::init(){
    LOGD("=== Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO ver;
    ASSERT_OK( TYLibVersion(&ver) );
    LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);

    std::vector<TY_DEVICE_BASE_INFO> selected;
    ASSERT_OK( selectDevice(TY_INTERFACE_ALL, ID, IP, 1, selected) );
    ASSERT(!selected.empty());
    TY_DEVICE_BASE_INFO& selectedDev = selected[0];

    ASSERT_OK( TYOpenInterface(selectedDev.iface.id, &hIface) );
    ASSERT_OK( TYOpenDevice(hIface, selectedDev.id, &hDevice) );

    TY_COMPONENT_ID allComps;
    ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );
    if(!(allComps & TY_COMPONENT_RGB_CAM)){
        LOGE("=== Has no RGB m_camera, cant do registration");
    }
    TY_ISP_HANDLE isp_handle;
    ASSERT_OK(TYISPCreate(&isp_handle));
    ASSERT_OK(ColorIspInitSetting(isp_handle, hDevice));
    //You can turn on auto exposure function as follows ,but frame rate may reduce .
    //Device also may be casually stucked  1~2 seconds when software trying to adjust device exposure time value
#if 0
    ASSERT_OK(ColorIspInitAutoExposure(isp_handle, hDevice));
#endif

    LOGD("=== Configure components");
    TY_COMPONENT_ID componentIDs = TY_COMPONENT_DEPTH_CAM | TY_COMPONENT_RGB_CAM;
    ASSERT_OK( TYEnableComponents(hDevice, componentIDs) );

    // ASSERT_OK( TYSetEnum(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_YUYV_640x480) );
    bool hasUndistortSwitch, hasDistortionCoef;
    ASSERT_OK( TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_BOOL_UNDISTORTION, &hasUndistortSwitch) );
    ASSERT_OK( TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_DISTORTION, &hasDistortionCoef) );
    if (hasUndistortSwitch) {
        ASSERT_OK( TYSetBool(hDevice, TY_COMPONENT_RGB_CAM, TY_BOOL_UNDISTORTION, true) );
    }

    //获取RGB支持的属性
    bool hasRGB_ANALOG_GAIN, hasRGB_R_GAIN, hasRGB_G_GAIN, hasRGB_B_GAIN, hasRGB_EXPOSURE_TIME;
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_ANALOG_GAIN, &hasRGB_ANALOG_GAIN));
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_R_GAIN, &hasRGB_R_GAIN));
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_G_GAIN, &hasRGB_G_GAIN));
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_B_GAIN, &hasRGB_B_GAIN));
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME, &hasRGB_EXPOSURE_TIME));
    //修改RGB支持的属性
    if (hasRGB_ANALOG_GAIN) {
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_ANALOG_GAIN, 1));//设置RGB模拟增益
    }
    if (hasRGB_R_GAIN) {
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_R_GAIN, 1900));//设置RGB数字增益R通道
    }
    if (hasRGB_G_GAIN) {
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_G_GAIN, 1400));//设置RGB数字增益G通道
    }
    if (hasRGB_B_GAIN) {
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_B_GAIN, 2200));//设置RGB数字增益B通道
    }
    if (hasRGB_EXPOSURE_TIME) {
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME, 850));//设置RGB曝光时间
    }

    //设置左右IR的模拟增益，数字增益和曝光
    //获取左右IR支持的属性
    bool hasIR_ANALOG_GAIN, hasIR_GAIN, hasIR_EXPOSURE_TIME, hasIR_HDR;
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_ANALOG_GAIN, &hasIR_ANALOG_GAIN));
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_GAIN, &hasIR_GAIN));
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, &hasIR_EXPOSURE_TIME));
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_BOOL_HDR, &hasIR_HDR));
    if (hasIR_ANALOG_GAIN) {
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_ANALOG_GAIN, 1));
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_ANALOG_GAIN, 1));//设置左右IR模拟增益
    }
    if (hasIR_GAIN) {
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_GAIN, 32));
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_GAIN, 32));//设置左右IR数字增益
    }
    if (hasIR_EXPOSURE_TIME) {
        ASSERT_OK(TYSetInt(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, 500)); //设置IR曝光时间
    }
    if (hasIR_HDR) {
        ASSERT_OK(TYSetBool(hDevice, TY_COMPONENT_IR_CAM_LEFT, TY_BOOL_HDR, true));
        ASSERT_OK(TYSetBool(hDevice, TY_COMPONENT_IR_CAM_RIGHT, TY_BOOL_HDR, true));//设置开启HDR功能
    }

//    LOGD("=== Prepare image buffer");
//    uint32_t frameSize;
//    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
//    LOGD("     - Get size of framebuffer, %d", frameSize);
//    LOGD("     - Allocate & enqueue buffers");
//
//    frameBuffer[0] = new char[frameSize];
//    frameBuffer[1] = new char[frameSize];
//    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
//    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
//    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
//    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );
//
//    LOGD("=== Register event callback");
//    ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, nullptr));

    bool hasTriggerParam = false;
    ASSERT_OK( TYHasFeature(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &hasTriggerParam) );
    if (hasTriggerParam) {
        LOGD("=== Disable trigger mode");
        TY_TRIGGER_PARAM_EX trigger;
        trigger.mode = TY_TRIGGER_MODE_OFF;
        ASSERT_OK(TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger)));
    }

    // 回调函数初始化
    cb_data.index = 0;
    cb_data.hDevice = hDevice;
    cb_data.depthViewer = &depthViewer;
    cb_data.render = &render;
    cb_data.needUndistort = !hasUndistortSwitch && hasDistortionCoef;
    cb_data.IspHandle = isp_handle;

    float scale_unit = 1.;
    TYGetFloat(hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit);
    cb_data.scale_unit = scale_unit;
    depthViewer.depth_scale_unit = scale_unit;


    LOGD("=== Read depth calib info");
    ASSERT_OK( TYGetStruct(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA
            , &cb_data.depth_calib, sizeof(cb_data.depth_calib)) );

    LOGD("=== Read color calib info");
    ASSERT_OK( TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA
            , &cb_data.color_calib, sizeof(cb_data.color_calib)) );

    showCameraIntrinsics(cb_data);
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_DISTORTION, &cb_data.isTof));
}

void PercipioCamera::doRegister(const TY_CAMERA_CALIB_INFO& depth_calib
        , const TY_CAMERA_CALIB_INFO& color_calib
        , const cv::Mat& depth
        , const float f_scale_unit
        , const cv::Mat& color
        , bool needUndistort
        , cv::Mat& undistort_color
        , cv::Mat& out
        , bool map_depth_to_color
)
{
    int32_t         image_size;
    TY_PIXEL_FORMAT color_fmt;
    if(color.type() == CV_16U) {
        image_size = color.size().area() * 2;
        color_fmt = TY_PIXEL_FORMAT_MONO16;
    }
    else if(color.type() == CV_16UC3)
    {
        image_size = color.size().area() * 6;
        color_fmt = TY_PIXEL_FORMAT_RGB48;
    }
    else {
        image_size = color.size().area() * 3;
        color_fmt = TY_PIXEL_FORMAT_RGB;
    }
    // do undistortion
    if (needUndistort) {
        if(color_fmt == TY_PIXEL_FORMAT_MONO16)
            undistort_color = cv::Mat(color.size(), CV_16U);
        else if(color_fmt == TY_PIXEL_FORMAT_RGB48)
            undistort_color = cv::Mat(color.size(), CV_16UC3);
        else
            undistort_color = cv::Mat(color.size(), CV_8UC3);

        TY_IMAGE_DATA src;
        src.width = color.cols;
        src.height = color.rows;
        src.size = image_size;
        src.pixelFormat = color_fmt;
        src.buffer = color.data;

        TY_IMAGE_DATA dst;
        dst.width = color.cols;
        dst.height = color.rows;
        dst.size = image_size;
        dst.pixelFormat = color_fmt;
        dst.buffer = undistort_color.data;
        ASSERT_OK(TYUndistortImage(&color_calib, &src, nullptr, &dst));
    }
    else {
        undistort_color = color;
    }

    // do register
    if (map_depth_to_color) {
        out = cv::Mat::zeros(undistort_color.size(), CV_16U);
        ASSERT_OK(
                TYMapDepthImageToColorCoordinate(
                        &depth_calib,
                        depth.cols, depth.rows, depth.ptr<uint16_t>(),
                        &color_calib,
                        out.cols, out.rows, out.ptr<uint16_t>(), f_scale_unit
                )
        );
        cv::Mat temp;
        //you may want to use median filter to fill holes in projected depth image
        //or do something else here
        cv::medianBlur(out, temp, 5);
        out = temp;
    }
    else {
        if(color_fmt == TY_PIXEL_FORMAT_MONO16)
        {
            out = cv::Mat::zeros(depth.size(), CV_16U);
            ASSERT_OK(
                    TYMapMono16ImageToDepthCoordinate(
                            &depth_calib,
                            depth.cols, depth.rows, depth.ptr<uint16_t>(),
                            &color_calib,
                            undistort_color.cols, undistort_color.rows, undistort_color.ptr<uint16_t>(),
                            out.ptr<uint16_t>(), f_scale_unit
                    )
            );
        }
        else if(color_fmt == TY_PIXEL_FORMAT_RGB48)
        {
            out = cv::Mat::zeros(depth.size(), CV_16UC3);
            ASSERT_OK(
                    TYMapRGB48ImageToDepthCoordinate(
                            &depth_calib,
                            depth.cols, depth.rows, depth.ptr<uint16_t>(),
                            &color_calib,
                            undistort_color.cols, undistort_color.rows, undistort_color.ptr<uint16_t>(),
                            out.ptr<uint16_t>(), f_scale_unit
                    )
            );
        }
        else{
            out = cv::Mat::zeros(depth.size(), CV_8UC3);
            ASSERT_OK(
                    TYMapRGBImageToDepthCoordinate(
                            &depth_calib,
                            depth.cols, depth.rows, depth.ptr<uint16_t>(),
                            &color_calib,
                            undistort_color.cols, undistort_color.rows, undistort_color.ptr<uint8_t>(),
                            out.ptr<uint8_t>(), f_scale_unit
                    )
            );
        }
    }
}

void PercipioCamera::cleanup() {
    ASSERT_OK(TYCloseDevice(hDevice));
    ASSERT_OK(TYCloseInterface(hIface));
    ASSERT_OK(TYDeinitLib());
    delete[] frameBuffer[0];
    delete[] frameBuffer[1];
}

std::pair<cv::Mat, cv::Mat> PercipioCamera::handleFrame(TY_FRAME_DATA *frame, void *userdata) {
    auto* pData = (CallbackData*)userdata;
    LOGD("=== Get frame %d", ++pData->index);

    cv::Mat depth, color;
    parseFrame(*frame, &depth, nullptr, nullptr, &color, pData->IspHandle);
    bool FillHole = true;  //填洞开关
    bool SpeckleFilter = true;  //星噪滤波开关
    if (!depth.empty()) {
        if (pData->isTof)
        {
            TY_IMAGE_DATA src;
            src.width = depth.cols;
            src.height = depth.rows;
            src.size = depth.size().area() * 2;
            src.pixelFormat = TY_PIXEL_FORMAT_DEPTH16;
            src.buffer = depth.data;

            cv::Mat undistort_depth = cv::Mat(depth.size(), CV_16U);
            TY_IMAGE_DATA dst;
            dst.width = depth.cols;
            dst.height = depth.rows;
            dst.size = undistort_depth.size().area() * 2;
            dst.buffer = undistort_depth.data;
            dst.pixelFormat = TY_PIXEL_FORMAT_DEPTH16;
            ASSERT_OK(TYUndistortImage(&pData->depth_calib, &src, NULL, &dst));
            depth = undistort_depth.clone();
        }
        if (FillHole) {
            //深度图填洞处理
            DepthInpainter inpainter;
            inpainter._kernelSize = 10;
            inpainter._maxInternalHoleToBeFilled = 1800;
            inpainter._fillAll = false;
            inpainter.inpaint(depth, depth, cv::Mat());
        }
        if (SpeckleFilter) {
            //使用星噪滤波
            TY_IMAGE_DATA tyFilteredDepth;
            cv::Mat filteredDepth(depth.size(), depth.type());
            filteredDepth = depth.clone();
            mat2TY_IMAGE_DATA(TY_COMPONENT_DEPTH_CAM, filteredDepth, tyFilteredDepth);
            struct DepthSpeckleFilterParameters sfparam = DepthSpeckleFilterParameters_Initializer;
            sfparam.max_speckle_size = 300;//噪点面积小于该值将被过滤
            sfparam.max_speckle_diff = 12;//相邻像素视差大于该值将被视为噪点
            TYDepthSpeckleFilter(&tyFilteredDepth, &sfparam);
        }
    }

    cv::Mat undistort_color, depth_mapped;
    if (!depth.empty() && !color.empty()) {
        cv::Mat out;
        if (pData->needUndistort || MAP_DEPTH_TO_COLOR) {
            doRegister(pData->depth_calib, pData->color_calib, depth, pData->scale_unit, color, pData->needUndistort, undistort_color, out, MAP_DEPTH_TO_COLOR);
        }
        if (MAP_DEPTH_TO_COLOR){
            depth_mapped = out;
            undistort_color = color;
        }else{
            undistort_color = out;
            depth_mapped = depth;
        }
    }
    LOGD("=== Re-enqueue buffer(%p, %d)", frame->userBuffer, frame->bufferSize);
    ASSERT_OK(TYEnqueueBuffer(pData->hDevice, frame->userBuffer, frame->bufferSize));

    // 后处理
    depth_mapped.convertTo(depth_mapped, CV_16UC1);
    cv::Rect rect(280, 100, 720, 540);
    for (int i = 0; i < 960; i++) {
        for (int j = 0; j < 1280; j++) {
            if (!rect.contains(cv::Point(j, i))) {
                undistort_color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
                depth_mapped.at<ushort>(i, j) = 0;
            }
            if (depth_mapped.at<ushort>(i, j) < 500){
                depth_mapped.at<ushort>(i, j) = 0;
            }
        }
    }

    return {undistort_color, depth_mapped};
}

void PercipioCamera::eventCallback(TY_EVENT_INFO *event_info, void *userdata) {
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
        LOGD("=== Event Callback: Device Offline!");
        // Note:
        //     Please set TY_BOOL_KEEP_ALIVE_ONOFF feature to false if you need to debug with breakpoint!
    }
    else if (event_info->eventId == TY_EVENT_LICENSE_ERROR) {
        LOGD("=== Event Callback: License Error!");
    }

}

std::pair<cv::Mat, cv::Mat> PercipioCamera::SimpleFetchColor_Depth() {
    LOGD("=== Prepare image buffer");
    uint32_t frameSize;
    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
    LOGD("     - Get size of framebuffer, %d", frameSize);
    LOGD("     - Allocate & enqueue buffers");

    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );

    LOGD("=== Register event callback");
    ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, nullptr));


    LOGD("=== Start capture");
    ASSERT_OK( TYStartCapture(hDevice) );

    LOGD("=== Wait for callback");

    TY_FRAME_DATA frame;
    int error = TYFetchFrame(hDevice, &frame, -1);
    if (TYFetchFrame(hDevice, &frame, -1) != TY_STATUS_OK) {
        LOGE("Fetch frame error %d: %s", error, TYErrorString(error));
    }
    auto [color, depth] = handleFrame(&frame, &cb_data);
    TYISPUpdateDevice(cb_data.IspHandle);

    ASSERT_OK( TYStopCapture(hDevice) );
    LOGD("=== Main done!");

    return {color, depth};
}

void PercipioCamera::printIntrinsic(const TY_CAMERA_INTRINSIC &intrinsic) {
    LOGD("=== Intrinsic Matrix: ");
    for (int i = 0; i < 3; ++i) {
        LOGD("=== %.2f %.2f %.2f",intrinsic.data[i * 3 ], intrinsic.data[i * 3 + 1], intrinsic.data[i * 3 + 2]);
    }
}

void PercipioCamera::showCameraIntrinsics(const PercipioCamera::CallbackData &data) {
    LOGD("=== Depth Camera Calibration Info :");
    printIntrinsic(data.depth_calib.intrinsic);

    LOGD("=== Color Camera Calibration Info :");
    printIntrinsic(data.color_calib.intrinsic);
}

void PercipioCamera::mat2TY_IMAGE_DATA(int comp, const cv::Mat &mat, TY_IMAGE_DATA &data) {
    data.status = 0;
    data.componentID = comp;
    data.size = mat.total() * mat.elemSize();
    data.buffer = mat.data;
    data.width = mat.cols;
    data.height = mat.rows;
    data.pixelFormat = cvpf2typf(mat.type());
}

int PercipioCamera::cvpf2typf(int cvpf) {
    switch (cvpf) {
        case CV_8U:
            return TY_PIXEL_FORMAT_MONO;
        case CV_8UC3:
            return TY_PIXEL_FORMAT_RGB;
        case CV_16UC1:
            return TY_PIXEL_FORMAT_DEPTH16;
        default:
            return TY_PIXEL_FORMAT_UNDEFINED;
    }
}
