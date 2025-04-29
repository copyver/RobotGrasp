#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QThread>
#include <memory>

#include "PoseModel.hpp"
#include "ImageProcess.hpp"
#include "DobotTcp.hpp"
#include "RobotWorker.hpp"
#include "CustomButton.hpp"
#include "CameraWorker.hpp"


class MainWindow : public QMainWindow {
Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow() override;

private:
    QWidget *createButtonPanel();      // 左上：按钮区域
    QWidget *createControlDataPanel(); // 左下：机器人坐标显示
    QWidget *createImagePanel();       // 右上：实时图像显示
    QWidget *createLogPanel();         // 右下：日志显示

    // 辅助
    void appendLog(const QString &msg, bool isError = false);

    static QImage cvMatToQImage(const cv::Mat &mat);

    void updateButtonStates();

    // 抓取流程状态
    enum class GraspState {
        Idle, Running, Paused
    };
    GraspState graspState_ = GraspState::Idle;

    // --- UI 控件 ---
    SwitchButton *btnConnectCamera{};
    SwitchButton *btnConnectRobot{};
    QPushButton *btnCaptureImage{};
    QPushButton *btnStartGrasp{};
    QPushButton *btnPause{};
    QPushButton *btnReset{};
    QPushButton *btnEmergencyStop{};
    QLabel *cameraStatusLabel{};
    QLabel *robotStatusLabel{};
    QLabel *labelSpeed{};
    QLabel *labelX{};
    QLabel *labelY{};
    QLabel *labelZ{};
    QLabel *labelRx{};
    QLabel *labelRy{};
    QLabel *labelRz{};
    QLabel *labelImage{};
    QTextEdit *textLog{};

    // 对象
    std::shared_ptr<ModelPose> poseModel;
    std::shared_ptr<GrabData> grabData;
    PyObject *pFunc{};

    // 相机
    CameraWorker *cameraWorker{};
    QThread *cameraThread{};
    std::shared_ptr<PercipioCamera> camera;

    // 机器人
    RobotWorker *robotWorker{};
    QThread *robotThread{};


private Q_SLOTS:

    // UI 按钮槽
    void onConnectCameraToggled(bool checked); // 相机按钮
    void onConnectRobotToggled(bool checked);  // 机器人按钮
    void onCaptureImageClicked();   // 图像采集按钮
    void onStartGraspClicked();     // 开始按钮
    void onPauseClicked();   // 暂停按钮
    void onEmergencyStopClicked();  // 急停按钮
    void onResetClicked();  // 复位按钮

    void slotAppendLog(const QString &msg); // 显示日志
    void slotAppendError(const QString &msg);   // 显示错误日志

    // camera
    void slotCameraConnected(const std::shared_ptr<PercipioCamera> &cam);

    void slotCameraDisconnected();

    void slotDisplaySingleCapture(const QImage &img);


    // robotWorker
    void slotRobotConnected();

    void slotRobotDisconnected();

    void slotUpdatePose(double x, double y, double z, double rx, double ry, double rz);

    void slotUpdateImage(const QImage &img);

    void slotGraspFinished();

Q_SIGNALS:

    void requestSingleCapture();
};

#endif // MAINWINDOW_H
