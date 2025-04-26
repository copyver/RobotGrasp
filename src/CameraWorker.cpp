//
// Created by yhlever on 25-4-3.
//
#include <QThread>
#include <QDebug>

#include "CameraWorker.hpp"

QImage CameraWorker::cvMatToQImageInternal(const cv::Mat &mat) {
    if (mat.empty()) return {};
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    if (mat.type() == CV_8UC3)
        return QImage(mat.data, mat.cols, mat.rows,
                      static_cast<int>(mat.step),
                      QImage::Format_BGR888).copy();
#else
    if(mat.type()==CV_8UC3){
        cv::Mat rgb; cv::cvtColor(mat,rgb,cv::COLOR_BGR2RGB);
        return QImage(rgb.data,rgb.cols,rgb.rows,static_cast<int>(rgb.step),QImage::Format_RGB888).copy();
    }
#endif
    if (mat.type() == CV_8UC1)
        return QImage(mat.data, mat.cols, mat.rows,
                      static_cast<int>(mat.step),
                      QImage::Format_Grayscale8).copy();
    qWarning() << "[CameraWorker] cvMatToQImageInternal: Unsupported cv::Mat type:" << mat.type();
    return {};
}

CameraWorker::CameraWorker(QObject *parent) : QObject(parent), m_camera(nullptr) {}

CameraWorker::~CameraWorker() {
    disconnectCamera();
}

std::shared_ptr<PercipioCamera> CameraWorker::getCamera() const {
    return m_camera;
}

void CameraWorker::connectCamera() {

    if (m_camera) {
        Q_EMIT logMessage("[CameraWorker] 相机已连接 (无需重复连接)。");
        return;
    }

    Q_EMIT logMessage("[CameraWorker] 正在尝试连接相机...");
    try {
        m_camera = std::make_shared<PercipioCamera>();
        if (m_camera) {
            Q_EMIT logMessage("[CameraWorker] 相机连接成功。");
            Q_EMIT cameraConnected(m_camera);
        } else {
            throw std::runtime_error("创建相机对象失败");
        }
    } catch (const std::exception &e) {
        m_camera.reset();
        Q_EMIT errorMessage(QString("[CameraWorker] 连接相机失败: %1").arg(e.what()));
        Q_EMIT cameraDisconnected();
    } catch (...) {
        m_camera.reset();
        Q_EMIT errorMessage("[CameraWorker] 连接相机时发生未知错误。");
        Q_EMIT cameraDisconnected();
    }
}

void CameraWorker::disconnectCamera() {
    if (m_camera) {
        Q_EMIT logMessage("[CameraWorker] 正在断开相机连接...");
        try {
            Q_EMIT logMessage("[CameraWorker] 调用相机硬件关闭函数..."); // Placeholder log

        } catch (const std::exception &e) {
            Q_EMIT errorMessage(QString("[CameraWorker] 断开相机硬件时出错: %1").arg(e.what()));
        } catch (...) {
            Q_EMIT errorMessage("[CameraWorker] 断开相机硬件时发生未知错误。");
        }

        m_camera.reset();
        Q_EMIT logMessage("[CameraWorker] 相机已断开。");
        Q_EMIT cameraDisconnected();
    } else {
        Q_EMIT logMessage("[CameraWorker] 相机本就未连接。");
    }
}

void CameraWorker::captureSingleFrameSlot() {
    if (!m_camera) {
        Q_EMIT errorMessage("[CameraWorker] 请求单次采集失败：相机未连接。");
        Q_EMIT singleFrameCaptured(QImage());
        return;
    }

    Q_EMIT logMessage("[CameraWorker] 正在执行单次图像采集...");
    try {
        auto [color_mat, depth_mat] = m_camera->SimpleFetchColor_Depth();

        if (color_mat.empty()) {
            Q_EMIT errorMessage("[CameraWorker] 单次采集失败：获取到的彩色图像为空。");
            Q_EMIT singleFrameCaptured(QImage());
        } else {
            cv::Mat croppedColorMat = color_mat(
                    cv::Rect(280, 100, 720, 540) & cv::Rect(0, 0, color_mat.cols, color_mat.rows));
            QImage qImageResult = cvMatToQImageInternal(croppedColorMat);

            if (qImageResult.isNull()) {
                Q_EMIT errorMessage("[CameraWorker] 单次采集失败：无法将cv::Mat转换为QImage。");
                Q_EMIT singleFrameCaptured(QImage());
            } else {
                Q_EMIT logMessage("[CameraWorker] 单次图像采集成功。");
                Q_EMIT singleFrameCaptured(qImageResult); // Emit the captured image
            }
        }
    } catch (const std::exception &e) {
        Q_EMIT errorMessage(QString("[CameraWorker] 单次采集时发生异常: %1").arg(e.what()));
        Q_EMIT singleFrameCaptured(QImage());
    } catch (...) {
        Q_EMIT errorMessage("[CameraWorker] 单次采集时发生未知错误。");
        Q_EMIT singleFrameCaptured(QImage());
    }
}