//
// Created by yhlever on 25-4-3.
//

#ifndef ROBOT_GRASP_CAMERAWORKER_HPP
#define ROBOT_GRASP_CAMERAWORKER_HPP

#include <QObject>
#include <memory>
#include <QImage>
#include <opencv2/opencv.hpp>

#include "PercipioCamera.hpp"

class CameraWorker : public QObject {
Q_OBJECT
public:
    explicit CameraWorker(QObject *parent = nullptr);

    ~CameraWorker() override;

    std::shared_ptr<PercipioCamera> getCamera() const;

public Q_SLOTS:

    void connectCamera();

    void disconnectCamera();

    void captureSingleFrameSlot();

Q_SIGNALS:

    void cameraConnected(std::shared_ptr<PercipioCamera> cameraPtr);

    void cameraDisconnected();

    void singleFrameCaptured(const QImage &image);

    void logMessage(const QString &msg);

    void errorMessage(const QString &msg);

private:
    std::shared_ptr<PercipioCamera> m_camera;

    static QImage cvMatToQImageInternal(const cv::Mat &mat);
};

#endif //ROBOT_GRASP_CAMERAWORKER_HPP
