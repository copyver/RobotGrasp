//
// Created by yhlever on 25-4-1.
//

#ifndef ROBOT_GRASP_ROBOTWORKER_HPP
#define ROBOT_GRASP_ROBOTWORKER_HPP

#include <QObject>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "DobotTcp.hpp"
#include "ImageProcess.hpp"
#include "PoseModel.hpp"
#include "DobotTcp.hpp"
#include "PercipioCamera.hpp"

struct MotionSharedData {
    std::vector<double> PoseBHo; // xyzrpy
    std::string className;       // 物体类别
};

namespace RobotConstants {
    // Timing
    constexpr std::chrono::milliseconds GRASP_EFFECTOR_DELAY{500};
    constexpr std::chrono::milliseconds NON_RECOGNITION_SLEEP{2000};
    constexpr std::chrono::milliseconds PAUSE_CHECK_INTERVAL{1000};

    // Robot Parameters
    constexpr int EFFECTOR_REGISTER_INDEX = 40000;
    constexpr int EFFECTOR_ON_DURATION_MS = 500;
    constexpr int EFFECTOR_OFF_DURATION_MS = 1000;

    // Grasping Parameters
    constexpr double APPROACH_Z_OFFSET = 20.0;
    constexpr double LIFT_Z_OFFSET = 20.0;
    constexpr int GRASP_SPEED = 20;
    constexpr double PRE_GRASP_Y_OFFSET = -28.0;
    constexpr double RZ_FLIP_THRESHOLD = -120.0;
    constexpr double RZ_FLIP_ADJUSTMENT = 180.0;

    // Image Processing
    constexpr int DEPTH_ROI_HALF_SIZE = 3; // ROI is (2*size+1)x(2*size+1) = 7x7
    constexpr int POSE_DRAW_AXIS_LENGTH = 60;
    const cv::Rect IMAGE_CROP_ROI(280, 100, 720, 540); // ROI for UI display

    // Control Flow
    constexpr int MAX_RECOGNITION_FAILURES = 3;

    // Mode 11 parameters
    constexpr double MODE11_TEST_OFFSET_Y = 80.0;
    constexpr double MODE11_TEST_MIN_Y = -160;
    constexpr double MODE11_TEST_MAX_Y = 340;
    constexpr double MODE11_TEST_Z = 150.0;
    constexpr double MODE11_INTERMEDIATE_OFFSET_Z = 5.0;
}


class RobotWorker : public QObject {
Q_OBJECT

public:
    RobotWorker(std::shared_ptr<PercipioCamera> cam,
                std::shared_ptr<ModelPose> poseModel,
                PyObject *pFunc,
                std::shared_ptr<GrabData> grabData,
                QObject *parent = nullptr);

    ~RobotWorker() override;

    DobotTcp *getRobot();

public Q_SLOTS:

    void connectRobot();             // 连接
    void disconnectRobot();          // 断开 (stops thread and cleans up)
    void startGrasp();               // 抓取执行 (requires camera and robot connected)
    void requestPause(bool pause);   // 请求暂停/继续
    void requestEmergencyStop();     // 请求紧急停止
    void requestReset();             // 请求复位

    void setCamera(std::shared_ptr<PercipioCamera> cam); // Slot to receive camera object
    void handleCameraDisconnected();

    void stopAndJoinThread();

Q_SIGNALS:

    void signalLog(const QString &text);

    void signalImage(const QImage &img);

    void signalRobotPose(double x, double y, double z, double rx, double ry, double rz);

    void signalFinished();

    void signalRobotConnected();

    void signalRobotDisconnected();

    void signalError(const QString &errorMessage);

private:
    std::shared_ptr<PercipioCamera> m_camera;
    std::shared_ptr<ModelPose> m_poseModel;
    std::shared_ptr<GrabData> m_grabData;
    PyObject *m_pFunc{};
    std::shared_ptr<DobotTcp> m_dobot;


    // --- Threading and State ---
    std::thread m_robotThread;
    std::atomic<bool> m_isRunning;
    std::atomic<bool> m_isPaused;
    std::atomic<bool> m_stopRequested{};
    bool m_needMoveRobot;
    bool m_robotMotionDone;
    std::mutex m_mtx;
    std::condition_variable m_cv;
    MotionSharedData m_motionData;

    // --- Private Methods ---
    void robotMotionLoop();

    bool acquireImages(cv::Mat &colorMat, cv::Mat &depthMat);

    bool performPoseEstimation(const cv::Mat &colorMat, const cv::Mat &depthMat, ModelPoseResult &poseResult);

    bool calculateTargetPose(const cv::Mat &colorMat, const cv::Mat &depthMat, const ModelPoseResult &poseResult,
                             std::vector<double> &targetPoseBHo, std::string &className, cv::Mat &vizImage);

    void prepareAndEmitImage(const cv::Mat &vizImage);

    void triggerRobotMovement(const std::vector<double> &targetPoseBHo, const std::string &className);

    void waitForRobotMovement();

    bool checkGraspPrerequisites();

    void signalLoopsToStop();
};


#endif //ROBOT_GRASP_ROBOTWORKER_HPP
