//
// Created by yhlever on 25-4-1.
//
#include <thread>
#include <sstream>
#include <QImage>
#include <QDebug>
#include <opencv2/imgcodecs.hpp>
#include <utility>

#include "RobotWorker.hpp"


RobotWorker::RobotWorker(std::shared_ptr<PercipioCamera> cam,
                         std::shared_ptr<ModelPose> poseModel,
                         PyObject *pFunc,
                         std::shared_ptr<GrabData> grabData,
                         QObject *parent)
        : QObject(parent),
          m_camera(std::move(cam)),
          m_poseModel(std::move(poseModel)),
          m_pFunc(pFunc),
          m_grabData(std::move(grabData)),
          m_dobot(nullptr), // Initialize shared_ptr to null
          m_isRunning(false),
          m_isPaused(false),
          m_needMoveRobot(false),
          m_robotMotionDone(false) {}

RobotWorker::~RobotWorker() {
    stopAndJoinThread();
}

void RobotWorker::connectRobot() {
    if (m_dobot) { // 检查是否已连接
        Q_EMIT signalLog("机器人已经连接");
        return;
    }
    if (m_isRunning.load()) {
        Q_EMIT signalError("请先停止当前任务再连接机器人。");
        return;
    }
    try {
        m_dobot = std::make_shared<DobotTcp>();
        if (m_dobot) {
            m_dobot->setInitState();
            m_dobot->setModbusStation();
            m_dobot->moveRobotOrigin(); // Move to origin on connect
            Q_EMIT signalLog("机器人连接成功");
            Q_EMIT signalRobotConnected();
        } else {
            Q_EMIT signalError("创建 DobotTcp 对象失败");
        }
    } catch (const std::exception &e) {
        Q_EMIT signalError(QString("机器人连接异常: %1").arg(e.what()));
        m_dobot.reset();
        Q_EMIT signalRobotDisconnected();
    }
}

void RobotWorker::disconnectRobot() {
    Q_EMIT signalLog("收到断开机器人请求...");
    stopAndJoinThread();
    if (m_dobot) {
        m_dobot.reset();
        Q_EMIT signalLog("机器人已断开");
        Q_EMIT signalRobotDisconnected();
    } else {
        Q_EMIT signalLog("机器人本就未连接。");
    }
}

void RobotWorker::setCamera(std::shared_ptr<PercipioCamera> cam) {
    Q_EMIT signalLog("接收到相机对象...");
    m_camera = std::move(cam); // Update the internal shared_ptr
    if (!m_camera) {
        Q_EMIT signalError("接收到的相机对象为空");
    } else {
        Q_EMIT signalLog("相机对象设置成功。");
    }
}

void RobotWorker::handleCameraDisconnected() {
    Q_EMIT signalError("处理相机断开事件...");
    m_camera.reset();
    if (m_isRunning.load()) {
        Q_EMIT signalError("相机丢失，正在停止当前抓取任务...");
        requestEmergencyStop();
        stopAndJoinThread();
    }
}

DobotTcp *RobotWorker::getRobot() {
    return m_dobot ? m_dobot.get() : nullptr;
}

void RobotWorker::requestPause(bool pause) {
    if (!m_dobot) {
        Q_EMIT signalError("机器人未连接，无法暂停/继续。");
        return;
    }
    m_isPaused.store(pause);

    try {
        if (pause) {
            m_dobot->pauseRobot();
            Q_EMIT signalLog("机器人硬件暂停指令已发送。");
        } else {
            m_dobot->continueRobot();
            Q_EMIT signalLog("机器人硬件继续指令已发送。");
            m_cv.notify_all();
        }
    } catch (const std::exception &e) {
        Q_EMIT signalError(QString("发送暂停/继续指令失败: %1").arg(e.what()));
    }
}

void RobotWorker::requestEmergencyStop() {
    Q_EMIT signalLog("收到紧急停止请求！");
    m_stopRequested.store(true);
    signalLoopsToStop();

    if (m_dobot) {
        try {
            Q_EMIT signalLog("正在发送硬件停止指令...");
            m_dobot->stopRobot();
            Q_EMIT signalLog("硬件停止指令已发送。");
        }
        catch (const std::exception &e) {
            Q_EMIT signalError(QString("发送硬件停止指令失败: %1").arg(e.what()));
        }
    } else {
        Q_EMIT signalLog("机器人未连接，仅停止软件循环。");
    }
}

void RobotWorker::requestReset() {
    Q_EMIT signalLog("收到复位请求...");
    if (m_isRunning.load()) {
        Q_EMIT signalLog("正在停止当前任务以进行复位...");
        stopAndJoinThread();
    }

    if (m_dobot) {
        try {
            Q_EMIT signalLog("正在发送硬件复位指令...");
            m_dobot->resetRobot();
            Q_EMIT signalLog("硬件复位指令已发送。机器人可能需要重新连接或初始化。");
            m_dobot.reset();
            Q_EMIT signalRobotDisconnected();
        }
        catch (const std::exception &e) {
            Q_EMIT signalError(QString("发送硬件复位指令失败: %1").arg(e.what()));
            m_dobot.reset();
            Q_EMIT signalRobotDisconnected();
        }
    } else {
        Q_EMIT signalError("机器人未连接，无法复位。");
    }

    m_isRunning.store(false);
    m_isPaused.store(false);
    m_stopRequested.store(false);
}

void RobotWorker::signalLoopsToStop() {
    m_isRunning.store(false);
    m_isPaused.store(false);
    m_cv.notify_all();
}

void RobotWorker::stopAndJoinThread() {
    if (!m_isRunning.exchange(false) && !m_robotThread.joinable()) {
        return;
    }
    Q_EMIT signalLog("正在停止内部线程和循环...");
    m_stopRequested.store(true);
    signalLoopsToStop();

    if (m_robotThread.joinable()) {
        Q_EMIT signalLog("等待运动线程(robotMotionLoop)结束...");
        try {
            m_robotThread.join();
            Q_EMIT signalLog("运动线程已停止。");
        } catch (const std::system_error &e) {
            Q_EMIT signalError(QString("运动线程 join 异常: %1").arg(e.what()));
        }
    } else {
        Q_EMIT signalLog("运动线程未运行或已结束。");
    }

    m_needMoveRobot = false;
    m_robotMotionDone = false;
    m_stopRequested.store(false);
}

void RobotWorker::robotMotionLoop() {
    std::cout << "[RobotMotionLoop] Thread started.\n";
    DobotTcp *dobot = nullptr;

    while (m_isRunning.load()) {
        MotionSharedData localData;

        // 等待“主线程让机器人运动”的信号
        {
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait(lk, [this] {
                return !m_isRunning.load() || m_stopRequested.load() || m_needMoveRobot;
            });

            if (!m_isRunning.load() || m_stopRequested.load()) {
                break;
            }

            if (m_needMoveRobot) {
                localData = m_motionData;
                m_needMoveRobot = false;
                dobot = m_dobot.get();
                if (!dobot) {
                    Q_EMIT signalError("运动线程错误：机器人对象在需要移动时为空！");
                    signalLoopsToStop();
                    break;
                }
            } else {
                continue;
            }
        }

        // 执行机器人运动
        Q_EMIT signalLog(QString("开始执行 '%1' 的抓取动作...").arg(QString::fromStdString(localData.className)));
        bool motion_success = true;

        try {
            int effector_amp = GrabData::chooseEffectorAmp(localData.className);
            int effector_z = GrabData::chooseEffectorZOffset(localData.className);
#define CHECK_STOP_BREAK if(!m_isRunning.load() || m_stopRequested.load()){ motion_success = false; break; }

            dobot->moveRobotSecond();
            CHECK_STOP_BREAK

            dobot->moveRobotCustomOffsetZ(localData.PoseBHo, RobotConstants::APPROACH_Z_OFFSET);
            CHECK_STOP_BREAK

            dobot->setEffectorRegs(dobot->index, RobotConstants::EFFECTOR_REGISTER_INDEX, effector_amp,
                                   RobotConstants::EFFECTOR_ON_DURATION_MS);

            dobot->moveRobotCustomOffsetZ(localData.PoseBHo, -effector_z, RobotConstants::GRASP_SPEED);
            CHECK_STOP_BREAK

            std::this_thread::sleep_for(RobotConstants::GRASP_EFFECTOR_DELAY);
            dobot->setEffectorRegs(dobot->index, RobotConstants::EFFECTOR_REGISTER_INDEX, 0,
                                   RobotConstants::EFFECTOR_OFF_DURATION_MS);

            // 碰撞处理机制
            if (dobot->isRobotMode11) {
                Q_EMIT signalLog("执行 Mode 11 特殊动作...");
                double x = localData.PoseBHo[0];
                double y = localData.PoseBHo[1];
                double z = localData.PoseBHo[2];
                double rx = localData.PoseBHo[3];
                double ry = localData.PoseBHo[4];
                double rz = localData.PoseBHo[5];

                dobot->moveRobotCustom(
                        std::vector<double>{x, y, z + RobotConstants::MODE11_INTERMEDIATE_OFFSET_Z, rx, ry, rz});
                CHECK_STOP_BREAK

                dobot->moveRobotCustom(std::vector<double>{x, y - RobotConstants::MODE11_TEST_OFFSET_Y,
                                                           z + RobotConstants::MODE11_INTERMEDIATE_OFFSET_Z, rx, ry,
                                                           rz});

                dobot->moveRobotCustom(std::vector<double>{x, y - RobotConstants::MODE11_TEST_OFFSET_Y,
                                                           z - RobotConstants::MODE11_INTERMEDIATE_OFFSET_Z, rx, ry,
                                                           rz});
                CHECK_STOP_BREAK
                dobot->moveRobotCustom(std::vector<double>{x, y + RobotConstants::MODE11_TEST_OFFSET_Y,
                                                           z - RobotConstants::MODE11_INTERMEDIATE_OFFSET_Z, rx, ry,
                                                           rz});
                CHECK_STOP_BREAK
                dobot->moveRobotCustom(std::vector<double>{x, y + RobotConstants::MODE11_TEST_OFFSET_Y, RobotConstants::MODE11_TEST_Z, rx, ry, rz});
                CHECK_STOP_BREAK

                dobot->moveRobotOrigin();
                CHECK_STOP_BREAK

                dobot->setEffectorRegs(dobot->index, RobotConstants::EFFECTOR_REGISTER_INDEX, effector_amp);

                dobot->isRobotMode11 = false;

                if (motion_success) {
                    Q_EMIT signalLog(
                            QString("Mode 11 动作完成 for '%1'.").arg(QString::fromStdString(localData.className)));
                    {
                        std::lock_guard<std::mutex> lk_done(m_mtx);
                        m_robotMotionDone = true;
                    }
                    m_cv.notify_one(); // Notify the waiting startGrasp loop
                }
                m_cv.notify_one();
                continue;
            }

            dobot->moveRobotCustomOffsetZ(localData.PoseBHo, effector_z + RobotConstants::LIFT_Z_OFFSET,
                                          RobotConstants::GRASP_SPEED);
            CHECK_STOP_BREAK

            dobot->moveRobotSecond();
            CHECK_STOP_BREAK

            dobot->moveRobotOrigin();
            CHECK_STOP_BREAK

            // 回到原点，通知主线程
            if (motion_success) { // Only notify if not stopped
                {
                    std::lock_guard<std::mutex> lk_done(m_mtx);
                    m_robotMotionDone = true; // Signal completion of main part
                }
                m_cv.notify_one();
            } else {
                // If stopped during main motion, ensure main thread (if waiting) is woken up
                {
                    std::lock_guard<std::mutex> lk_done(m_mtx);
                    m_robotMotionDone = true;
                }
                m_cv.notify_one();
                break; // Exit loop as stop was requested
            }

            dobot->moveRobotEnd();
            CHECK_STOP_BREAK
            dobot->setEffectorRegs(dobot->index, RobotConstants::EFFECTOR_REGISTER_INDEX, effector_amp);
            dobot->moveRobotOrigin();
            CHECK_STOP_BREAK

            if (motion_success) {
                Q_EMIT signalLog(QString("抓取动作完成 for '%1'.").arg(QString::fromStdString(localData.className)));
            }


        } catch (const std::exception &e) {
            Q_EMIT signalError(QString("机器人运动时发生错误: %1").arg(e.what()));
            motion_success = false;
            {
                std::lock_guard<std::mutex> lk_done(m_mtx);
                m_robotMotionDone = true;
            }
            m_cv.notify_one();
        }

        if (!motion_success) {
            break;
        }

    }

    std::cout << "[RobotMotionLoop] Exiting...\n";
    {
        std::lock_guard<std::mutex> lk_done(m_mtx);
        m_robotMotionDone = true;
    }
    m_cv.notify_one();
}


void RobotWorker::startGrasp() {
    if (m_isRunning.load()) {
        Q_EMIT signalLog("抓取流程已在运行中。");
        return;
    }

    if (!checkGraspPrerequisites()) {
        Q_EMIT signalFinished();
        return;
    }

    Q_EMIT signalLog("开始抓取流程...");
    m_isRunning.store(true);
    m_isPaused.store(false);
    m_stopRequested.store(false);

    // 先回到原点
    try {
        m_dobot->moveRobotOrigin();
    } catch (const std::exception &e) {
        Q_EMIT signalError(QString("移动机器人到原点失败: %1").arg(e.what()));
        m_isRunning.store(false);
        Q_EMIT signalFinished();
        return;
    }

    if (m_robotThread.joinable()) {
        try { m_robotThread.join(); } catch (...) { /* ignore */ }
    }
    m_robotThread = std::thread(&RobotWorker::robotMotionLoop, this);

    int consecutiveFailures = 0;
    while (m_isRunning.load() && !m_stopRequested.load() &&
           consecutiveFailures < RobotConstants::MAX_RECOGNITION_FAILURES) {

        // 暂停处理机制
        while (m_isPaused.load() && m_isRunning.load() && !m_stopRequested.load()) {
            std::unique_lock<std::mutex> lk(m_mtx);
            m_cv.wait_for(lk, RobotConstants::PAUSE_CHECK_INTERVAL,
                          [this] { return !m_isPaused.load() || !m_isRunning.load() || m_stopRequested.load(); });
        }
        if (!m_isRunning.load() || m_stopRequested.load()) break;

        if (!m_camera) {
            Q_EMIT signalError("相机丢失，无法继续抓取。");
            signalLoopsToStop();
            break;
        }

        // 获取RGB-D
        cv::Mat colorMat, depthMat;
        if (!acquireImages(colorMat, depthMat)) {
            std::this_thread::sleep_for(RobotConstants::NON_RECOGNITION_SLEEP); // Wait before retrying
            continue;
        }
//        cv::imwrite("/home/yhlever/CLionProjects/ROBOT-GRASP/results/color.png", colorMat);

        // 推理位姿
        ModelPoseResult poseResult;
        if (!performPoseEstimation(colorMat, depthMat, poseResult)) {
            consecutiveFailures++;
            Q_EMIT signalLog(QString("未识别到对象 %1/%2 次").arg(consecutiveFailures).arg(
                    RobotConstants::MAX_RECOGNITION_FAILURES));
            if (consecutiveFailures >= RobotConstants::MAX_RECOGNITION_FAILURES) {
                Q_EMIT signalLog("连续识别失败次数达到上限，停止抓取。");
                signalLoopsToStop();
                break;
            }
            std::this_thread::sleep_for(RobotConstants::NON_RECOGNITION_SLEEP);
            continue;
        }
        // 重置失败次数
        consecutiveFailures = 0;

        // ============ 计算新的相机->物体姿态，得到机器人坐标系下的目标位置/姿态 ============
        std::vector<double> targetPoseBHo;
        std::string className;
        cv::Mat vizImage;
        if (!calculateTargetPose(colorMat, depthMat, poseResult, targetPoseBHo, className, vizImage)) {
            std::this_thread::sleep_for(RobotConstants::NON_RECOGNITION_SLEEP);
            continue;
        }

        // 更新UI界面
        prepareAndEmitImage(vizImage);
        Q_EMIT signalRobotPose(targetPoseBHo[0], targetPoseBHo[1], targetPoseBHo[2],
                               targetPoseBHo[3], targetPoseBHo[4], targetPoseBHo[5]);

        // 通知子线程(机器人线程)执行机器人运动
        triggerRobotMovement(targetPoseBHo, className);
        waitForRobotMovement();

        if (!m_isRunning.load() || m_stopRequested.load()) break;
        Q_EMIT signalLog("该轮抓取与运动完成，继续下一轮识别...");
    }

    if (!m_stopRequested.load() && consecutiveFailures >= RobotConstants::MAX_RECOGNITION_FAILURES) {
        Q_EMIT signalLog(QString("因连续 %1 次未识别到对象而停止。").arg(RobotConstants::MAX_RECOGNITION_FAILURES));
    } else if (m_stopRequested.load()) {
        Q_EMIT signalLog("抓取流程被手动停止。");
    } else {
        Q_EMIT signalLog("抓取流程因外部原因停止。");
    }

    if (m_dobot) {
        try {
            m_dobot->setEffectorRegs(m_dobot->index, RobotConstants::EFFECTOR_REGISTER_INDEX, 0);
            // m_dobot->moveRobotOrigin();
        } catch (const std::exception &e) {
            Q_EMIT signalError(QString("结束时关闭执行器失败: %1").arg(e.what()));
        }
    }

    stopAndJoinThread();
    Q_EMIT signalLog("抓取流程结束！");
    m_isRunning.store(false);
    Q_EMIT signalFinished();
}

bool RobotWorker::acquireImages(cv::Mat &colorMat, cv::Mat &depthMat) {
    if (!m_camera) {
        Q_EMIT signalError("尝试采集图像但相机对象为空。");
        return false;
    }
    try {
        auto [color_mat_local, depth_mat_local] = m_camera->SimpleFetchColor_Depth();
        if (color_mat_local.empty() || depth_mat_local.empty()) {
            Q_EMIT signalLog("图像获取失败 (空图像)。");
            return false;
        }
        colorMat = color_mat_local;
        depthMat = depth_mat_local;
        return true;
    } catch (const std::exception &e) {
        Q_EMIT signalError(QString("图像获取异常: %1").arg(e.what()));
        return false;
    }
}


bool RobotWorker::performPoseEstimation(const cv::Mat &colorMat, const cv::Mat &depthMat, ModelPoseResult &poseResult) {
    if (!m_poseModel || !m_pFunc) {
        Q_EMIT signalError("位姿模型或 Python 函数未初始化。");
        return false;
    }
    try {
        bool is_recognized = m_poseModel->runPoseEstimation(m_pFunc, colorMat, depthMat);
        if (is_recognized) {
            std::ostringstream strCout;
            std::streambuf *oldCoutStreamBuf = std::cout.rdbuf();
            std::cout.rdbuf(strCout.rdbuf());
            m_poseModel->printPoseResult();
            std::cout.rdbuf(oldCoutStreamBuf);
            std::string poseResultStr = strCout.str();
            Q_EMIT signalLog(QString("位姿识别结果:\n%1").arg(QString::fromStdString(poseResultStr)));
            poseResult = m_poseModel->result; // Copy result
            return true;
        } else {
            return false;
        }
    } catch (const std::exception &e) {
        Q_EMIT signalError(QString("位姿识别异常: %1").arg(e.what()));
        return false;
    }
}

bool RobotWorker::calculateTargetPose(const cv::Mat &colorMat, const cv::Mat &depthMat,
                                      const ModelPoseResult &poseResult, std::vector<double> &targetPoseBHo,
                                      std::string &className, cv::Mat &vizImage) {
    if (!m_grabData) {
        Q_EMIT signalError("抓取数据处理模块未初始化。");
        return false;
    }
    try {
        if (poseResult.segments.empty() || poseResult.cHos.empty() || poseResult.classNames.empty()) {
            Q_EMIT signalLog("位姿结果数据不完整，无法计算目标。");
            return false;
        }
        vizImage = colorMat.clone();

        // 平移安全校正策略
        std::vector<cv::Point2f> centers;
        GrabData::drawSegmentsOnImage(vizImage, poseResult.segments, centers); // Modifies vizImage

        if (centers.empty()) {
            Q_EMIT signalLog("分割结果中没有中心点，跳过此轮。");
            return false;
        }

        cv::Point2f center = centers[0];
        int roi_x = static_cast<int>(center.x) - RobotConstants::DEPTH_ROI_HALF_SIZE;
        int roi_y = static_cast<int>(center.y) - RobotConstants::DEPTH_ROI_HALF_SIZE;
        int roi_size = 2 * RobotConstants::DEPTH_ROI_HALF_SIZE + 1;
        cv::Rect roi(roi_x, roi_y, roi_size, roi_size);

        roi &= cv::Rect(0, 0, depthMat.cols, depthMat.rows);
        if (roi.area() <= 0) {
            Q_EMIT signalLog("计算的深度 ROI 无效或超出边界。");
            return false;
        }

        float depth = m_grabData->getDepth(depthMat, roi);
        if (depth <= 0) { // Check for invalid depth
            Q_EMIT signalLog(QString("获取深度失败 (无效深度: %1)。").arg(depth));
            return false;
        }

        cv::Mat uvd = (cv::Mat_<float>(3, 1) << center.x, center.y, depth);
        cv::Mat xyz_cam = m_grabData->imagePointToCamera(uvd);

        Eigen::Matrix4d cHo = poseResult.cHos[0];
        GrabData::setTranslation(cHo, xyz_cam);

        // 旋转安全校正策略
        Eigen::Matrix4d new_cHo = m_grabData->computeNewcHo(cHo);
        new_cHo = GrabData::adjustObjectPoseWithRect(colorMat, poseResult.segments[0], new_cHo);
        Eigen::Matrix4d new_bHo = m_grabData->computeBHo(new_cHo);

        targetPoseBHo = m_grabData->matrixToPose(new_bHo);
        className = poseResult.classNames[0];

        // 标定补偿
        targetPoseBHo[1] += RobotConstants::PRE_GRASP_Y_OFFSET;
        if (targetPoseBHo[5] < RobotConstants::RZ_FLIP_THRESHOLD) {
            targetPoseBHo[5] += RobotConstants::RZ_FLIP_ADJUSTMENT;
        }

        // 可视化
        m_grabData->drawObjectPose(vizImage, new_cHo, RobotConstants::POSE_DRAW_AXIS_LENGTH);
        // cv::imwrite("../results/debug_final_pose.png", vizImage);
        return true;
    } catch (const std::exception &e) {
        Q_EMIT signalError(QString("计算目标位姿时出错: %1").arg(e.what()));
        return false;
    }
}

void RobotWorker::prepareAndEmitImage(const cv::Mat &vizImage) {
    if (vizImage.empty()) {
        Q_EMIT signalError("准备发送图像时发现图像为空。");
        return;
    }
    try {
        cv::Mat croppedPoseMap = vizImage(
                RobotConstants::IMAGE_CROP_ROI & cv::Rect(0, 0, vizImage.cols, vizImage.rows));

        if (croppedPoseMap.empty()) {
            Q_EMIT signalError("图像裁剪失败，无法发送图像信号。");
            return;
        }

        QImage q_poseimg(
                croppedPoseMap.data,
                croppedPoseMap.cols,
                croppedPoseMap.rows,
                static_cast<int>(croppedPoseMap.step),
                QImage::Format_BGR888
        );

        if (!q_poseimg.isNull()) {
            Q_EMIT signalImage(q_poseimg.copy());
        } else {
            Q_EMIT signalLog("OpenCV Mat 转换为 QImage 失败。");
        }
    } catch (const cv::Exception &e) {
        Q_EMIT signalError(QString("处理或发送图像时 OpenCV 异常: %1").arg(e.what()));
    } catch (const std::exception &e) {
        Q_EMIT signalError(QString("处理或发送图像时异常: %1").arg(e.what()));
    }
}

void RobotWorker::triggerRobotMovement(const std::vector<double> &targetPoseBHo, const std::string &className) {
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_motionData.PoseBHo = targetPoseBHo;
        m_motionData.className = className;
        m_needMoveRobot = true;
        m_robotMotionDone = false;
    }
    m_cv.notify_one();
    Q_EMIT signalLog("已发送运动指令到机器人线程。");
}

void RobotWorker::waitForRobotMovement() {
    Q_EMIT signalLog("等待机器人运动完成...");
    {
        std::unique_lock<std::mutex> lk(m_mtx);
        m_cv.wait(lk, [this] {
            return m_robotMotionDone || !m_isRunning.load();
        });
    }
    if (m_robotMotionDone) {
        Q_EMIT signalLog("机器人运动完成信号收到。");
    } else {
        Q_EMIT signalLog("等待运动时收到停止信号。");
    }
}

bool RobotWorker::checkGraspPrerequisites() {
    if (!m_dobot) {
        Q_EMIT signalError("机器人未连接，无法开始抓取。");
        return false;
    }
    if (!m_camera) {
        Q_EMIT signalError("相机未连接或未设置，无法开始抓取。");
        return false;
    }
    if (!m_poseModel || !m_pFunc || !m_grabData) {
        Q_EMIT signalError("内部模块（模型、数据处理）未初始化，无法开始抓取。");
        return false;
    }
    return true;
}
