#include <QGroupBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QDateTime>
#include <QPixmap>
#include <QScrollBar>
#include <QMessageBox>
#include <opencv2/imgproc.hpp>

#include "mainwindow.hpp"


QImage MainWindow::cvMatToQImage(const cv::Mat &mat) {
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
    return {};
}

void MainWindow::appendLog(const QString &msg, bool isError) {
    if (!textLog) return;
    QMetaObject::invokeMethod(textLog, [=]() {
        QTextCharFormat fmt;
        if (isError) {
            fmt.setForeground(Qt::red);
        }
        else fmt.setForeground(Qt::black);
        textLog->setCurrentCharFormat(fmt);
        textLog->append(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") + msg);
        textLog->setCurrentCharFormat(QTextCharFormat{}); // Reset format
        textLog->verticalScrollBar()->setValue(textLog->verticalScrollBar()->maximum());
    }, Qt::QueuedConnection);
}

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        graspState_(GraspState::Idle) {
    // ---------- UI 布局 ----------
    auto *central = new QWidget(this);
    auto *grid = new QGridLayout(central);
    grid->setSpacing(15);
    grid->setContentsMargins(15, 15, 15, 15);

    grid->addWidget(createButtonPanel(), 0, 0);
    grid->addWidget(createControlDataPanel(), 1, 0);
    grid->addWidget(createImagePanel(), 0, 1);
    grid->addWidget(createLogPanel(), 1, 1);
    grid->setRowStretch(0, 3);
    grid->setRowStretch(1, 2);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 2);
    setCentralWidget(central);
    this->setWindowTitle("机器人抓取控制台"); // Set a window title
    this->setMinimumSize(800, 600); // Set a minimum size

    // ---------- 业务初始化 ----------
    try {
        poseModel = std::make_shared<ModelPose>();
        pFunc = poseModel->loadModel();
        if (!pFunc) {
            throw std::runtime_error("加载位姿模型或 Python 函数失败!");
        }
        grabData = std::make_shared<GrabData>();
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "初始化错误", QString("业务对象初始化失败: %1").arg(e.what()));
        appendLog(QString("业务对象初始化失败: %1").arg(e.what()), true);
    }

    // 相机
    cameraWorker = new CameraWorker();
    cameraThread = new QThread(this);
    cameraWorker->moveToThread(cameraThread);
    connect(cameraThread, &QThread::finished, cameraWorker, &QObject::deleteLater);
    connect(cameraWorker, &CameraWorker::cameraConnected, this, &MainWindow::slotCameraConnected);
    connect(cameraWorker, &CameraWorker::cameraDisconnected, this, &MainWindow::slotCameraDisconnected);
    connect(this, &MainWindow::requestSingleCapture, cameraWorker, &CameraWorker::captureSingleFrameSlot,
            Qt::QueuedConnection);
    connect(cameraWorker, &CameraWorker::singleFrameCaptured, this, &MainWindow::slotDisplaySingleCapture,
            Qt::QueuedConnection);
    cameraThread->start();


    // 机器人
    robotWorker = new RobotWorker(nullptr, poseModel, pFunc, grabData);
    robotThread = new QThread(this);
    robotWorker->moveToThread(robotThread);
    connect(robotThread, &QThread::finished, robotWorker, &QObject::deleteLater);
    connect(robotWorker, &RobotWorker::signalRobotConnected, this, &MainWindow::slotRobotConnected);
    connect(robotWorker, &RobotWorker::signalRobotDisconnected, this, &MainWindow::slotRobotDisconnected);
    connect(robotWorker, &RobotWorker::signalLog, this, &MainWindow::slotAppendLog);
    connect(robotWorker, &RobotWorker::signalError, this, &MainWindow::slotAppendError);
    connect(robotWorker, &RobotWorker::signalImage, this, &MainWindow::slotUpdateImage);
    connect(robotWorker, &RobotWorker::signalRobotPose, this, &MainWindow::slotUpdatePose);
    connect(robotWorker, &RobotWorker::signalFinished, this, &MainWindow::slotGraspFinished);
    robotThread->start();

    // 初始化按钮
    updateButtonStates();
}

MainWindow::~MainWindow() {
    appendLog("正在关闭应用程序...");
    if (robotThread) {
        if (robotWorker) {
            QMetaObject::invokeMethod(robotWorker, "stopAndJoinThread", Qt::QueuedConnection);
        }
        robotThread->quit();
        if (!robotThread->wait(5000)) {
            appendLog("Robot thread did not finish gracefully, terminating...", true);
            robotThread->terminate();
            robotThread->wait();
        } else {
            appendLog("Robot thread finished.");
        }
    }
    if (cameraThread) {
        if (cameraWorker) {
            QMetaObject::invokeMethod(cameraWorker, "disconnectCamera", Qt::QueuedConnection);
        }
        cameraThread->quit();
        if (!cameraThread->wait(5000)) {
            appendLog("Camera thread did not finish gracefully, terminating...", true);
            cameraThread->terminate();
            cameraThread->wait();
        } else {
            appendLog("Camera thread finished.");
        }
    }
    appendLog("清理完成。");
}

QWidget *MainWindow::createButtonPanel() {
    auto *box = new QGroupBox("操作窗口", this);
    auto *main = new QVBoxLayout(box);
    main->setSpacing(12);
    main->setContentsMargins(10, 20, 10, 10);

    // 相机行
    btnConnectCamera = new SwitchButton;
    cameraStatusLabel = new QLabel("相机状态：");
    cameraStatusLabel->setFont(QFont("Arial", 12));
    auto camRow = new QHBoxLayout;
    camRow->addWidget(cameraStatusLabel);
    camRow->addStretch();
    camRow->addWidget(btnConnectCamera);
    connect(btnConnectCamera, &SwitchButton::toggled, this, &MainWindow::onConnectCameraToggled);


    // 机器人行
    btnConnectRobot = new SwitchButton;
    robotStatusLabel = new QLabel("机器人状态：");
    robotStatusLabel->setFont(QFont("Arial", 12));
    auto robRow = new QHBoxLayout;
    robRow->addWidget(robotStatusLabel);
    robRow->addStretch();
    robRow->addWidget(btnConnectRobot);
    connect(btnConnectRobot, &SwitchButton::toggled, this, &MainWindow::onConnectRobotToggled);

    // 顶部按钮
    btnCaptureImage = new QPushButton("采集图像");
    btnCaptureImage->setObjectName("btnCaptureImage");
    connect(btnCaptureImage, &QPushButton::clicked, this, &MainWindow::onCaptureImageClicked);
    btnCaptureImage->setEnabled(false);

    auto *top = new QVBoxLayout;
    top->addLayout(camRow);
    top->addLayout(robRow);
    top->addWidget(btnCaptureImage);

    // 网格按钮
    btnStartGrasp = new QPushButton("开始抓取");
    btnStartGrasp->setObjectName("btnStartGrasp");
    connect(btnStartGrasp, &QPushButton::clicked, this, &MainWindow::onStartGraspClicked);

    btnPause = new QPushButton("暂停");
    btnPause->setObjectName("btnPause");
    connect(btnPause, &QPushButton::clicked, this, &MainWindow::onPauseClicked);

    btnEmergencyStop = new QPushButton("急停");
    btnEmergencyStop->setObjectName("btnEmergencyStop");
    connect(btnEmergencyStop, &QPushButton::clicked, this, &MainWindow::onEmergencyStopClicked);

    btnReset = new QPushButton("复位");
    btnReset->setObjectName("btnReset");
    connect(btnReset, &QPushButton::clicked, this, &MainWindow::onResetClicked);

    auto grid = new QGridLayout;
    grid->setSpacing(8);
    grid->addWidget(btnStartGrasp, 0, 0);
    grid->addWidget(btnPause, 0, 1);
    grid->addWidget(btnEmergencyStop, 1, 0);
    grid->addWidget(btnReset, 1, 1);

    main->addLayout(top);
    main->addLayout(grid);
    main->addStretch();

    btnConnectRobot->setEnabled(false);
    btnStartGrasp->setEnabled(false);
    btnPause->setEnabled(false);
    btnEmergencyStop->setEnabled(false);
    btnReset->setEnabled(false);

    return box;
}

QWidget *MainWindow::createControlDataPanel() {
    auto *box = new QGroupBox("机器人窗口", this);
    auto *form = new QFormLayout(box);

    labelX = new QLabel("0");
    labelY = new QLabel("0");
    labelZ = new QLabel("0");
    labelRx = new QLabel("0");
    labelRy = new QLabel("0");
    labelRz = new QLabel("0");
    labelSpeed = new QLabel("1.0");

    form->addRow("X坐标:", labelX);
    form->addRow("Y坐标:", labelY);
    form->addRow("Z坐标:", labelZ);
    form->addRow("Rx转角:", labelRx);
    form->addRow("Ry转角:", labelRy);
    form->addRow("Rz转角:", labelRz);
    form->addRow("机器人速率:", labelSpeed);
    return box;
}

QWidget *MainWindow::createImagePanel() {
    auto *box = new QGroupBox("显示窗口", this);
    auto *lay = new QVBoxLayout(box);
    lay->setSpacing(12);
    lay->setContentsMargins(10, 20, 10, 10);
    labelImage = new QLabel("相机预览");
    labelImage->setAlignment(Qt::AlignCenter);
    labelImage->setMinimumSize(320, 240);
    lay->addWidget(labelImage);
    return box;
}

QWidget *MainWindow::createLogPanel() {
    auto *box = new QGroupBox("日志窗口", this);
    auto *lay = new QVBoxLayout(box);
    lay->setSpacing(12);
    lay->setContentsMargins(10, 20, 10, 10);
    textLog = new QTextEdit;
    textLog->setReadOnly(true);
    textLog->setFont(QFont("Monospace", 9));
    lay->addWidget(textLog);
    appendLog("系统日志初始化...");
    return box;
}

void MainWindow::updateButtonStates() {
    switch (graspState_) {
        case GraspState::Idle:
            btnStartGrasp->setEnabled(btnConnectRobot->isChecked()); // Enable only if robot is connected
            btnReset->setEnabled(btnConnectRobot->isChecked());      // Enable only if robot is connected
            btnPause->setEnabled(false);
            btnPause->setText("暂停");
            btnEmergencyStop->setEnabled(false);
            break;
        case GraspState::Running:
            btnStartGrasp->setEnabled(false);
            btnReset->setEnabled(false);
            btnPause->setEnabled(true);
            btnPause->setText("暂停");
            btnEmergencyStop->setEnabled(true);
            break;
        case GraspState::Paused:
            btnStartGrasp->setEnabled(false);
            btnReset->setEnabled(false);
            btnPause->setEnabled(true);
            btnPause->setText("继续");
            btnEmergencyStop->setEnabled(true); // Allow stop from paused state
            break;
    }
    // Capture button depends only on camera connection
    btnCaptureImage->setEnabled(btnConnectCamera->isChecked());
    // Robot connect button depends on camera connection
    btnConnectRobot->setEnabled(btnConnectCamera->isChecked());
}

void MainWindow::onConnectCameraToggled(bool checked) {
    const char *slot = checked ? "connectCamera" : "disconnectCamera";
    // Invoke method on CameraWorker living in cameraThread
    QMetaObject::invokeMethod(cameraWorker, slot, Qt::QueuedConnection);
}

void MainWindow::onConnectRobotToggled(bool checked) {
    if (!robotWorker) {
        appendLog("RobotWorker 未初始化", true);
        btnConnectRobot->setChecked(false); // Revert toggle state
        return;
    }
    if (!btnConnectCamera->isChecked() && checked) {
        appendLog("请先连接相机才能连接机器人", true);
        btnConnectRobot->setChecked(false); // Revert toggle state
        return;
    }

    const char *slot = checked ? "connectRobot" : "disconnectRobot";
    // Invoke method on RobotWorker living in robotThread
    QMetaObject::invokeMethod(robotWorker, slot, Qt::QueuedConnection);
}

void MainWindow::onCaptureImageClicked() {
    if (!btnConnectCamera->isChecked()) {
        appendLog("请先连接相机", true);
        return;
    }
    appendLog("请求单次图像采集...");
    Q_EMIT requestSingleCapture();
}

void MainWindow::onStartGraspClicked() {
    if (graspState_ != GraspState::Idle) {
        appendLog(graspState_ == GraspState::Paused ?
                  "机器人暂停中，请先继续或停止。" : "机器人正在运行中，请先停止。", true);
        return;
    }
    if (!robotWorker || !btnConnectRobot->isChecked()) {
        appendLog("请先连接机器人", true);
        return;
    }

    graspState_ = GraspState::Running;
    updateButtonStates();
    appendLog("发送开始抓取指令...");
    QMetaObject::invokeMethod(robotWorker, "startGrasp", Qt::QueuedConnection);
}

void MainWindow::onPauseClicked() {
    if (graspState_ == GraspState::Idle) {
        appendLog("机器人未在运行", true);
        return;
    }
    if (!robotWorker) return;

    bool pauseRequest = (graspState_ == GraspState::Running);
    graspState_ = pauseRequest ? GraspState::Paused : GraspState::Running;
    QMetaObject::invokeMethod(robotWorker, "requestPause", Qt::DirectConnection, Q_ARG(bool, pauseRequest));
    appendLog(pauseRequest ? "已发送暂停指令" : "已发送继续指令");
    updateButtonStates();
}


void MainWindow::onEmergencyStopClicked() {
    if (graspState_ == GraspState::Idle) {
        appendLog("机器人未在运行", true);
        return;
    }
    if (!robotWorker) return;

    QMetaObject::invokeMethod(robotWorker, "requestEmergencyStop",
                              Qt::DirectConnection);
    QMetaObject::invokeMethod(robotWorker, "stopAndJoinThread", Qt::QueuedConnection);
    graspState_ = GraspState::Idle;
    appendLog("已发送停止指令");
    updateButtonStates();
}

void MainWindow::onResetClicked() {
    if (graspState_ != GraspState::Idle) {
        appendLog(
                graspState_ == GraspState::Paused ? "机器人暂停中，请先停止再复位。" : "机器人正在运行中，请先停止再复位。",
                true);
        return;
    }
    if (!robotWorker || !btnConnectRobot->isChecked()) {
        appendLog("请先连接机器人", true);
        return;
    }

    QMetaObject::invokeMethod(robotWorker, "requestReset", Qt::QueuedConnection);
    graspState_ = GraspState::Running;
    updateButtonStates();
    appendLog("已发送复位指令");
}


void MainWindow::slotAppendLog(const QString &msg) {
    appendLog(msg, false);
}

void MainWindow::slotAppendError(const QString &msg) {
    appendLog(msg, true);
}

void MainWindow::slotDisplaySingleCapture(const QImage &img) {
    if (img.isNull()) {
        appendLog("单次采集图像处理失败或无图像", true);
        return;
    }
    labelImage->setPixmap(
            QPixmap::fromImage(img).scaled(labelImage->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    appendLog("单次图像采集并显示成功");
}

void MainWindow::slotUpdateImage(const QImage &img) {
    if (!img.isNull())
        labelImage->setPixmap(QPixmap::fromImage(img).scaled(labelImage->size(), Qt::KeepAspectRatio));
}

void MainWindow::slotUpdatePose(double x, double y, double z, double rx, double ry, double rz) {
    QMetaObject::invokeMethod(this, [=]() {
        if (labelX) labelX->setText(QString::number(x, 'f', 1));
        if (labelY) labelY->setText(QString::number(y, 'f', 1));
        if (labelZ) labelZ->setText(QString::number(z, 'f', 1));
        if (labelRx) labelRx->setText(QString::number(rx, 'f', 1));
        if (labelRy) labelRy->setText(QString::number(ry, 'f', 1));
        if (labelRz) labelRz->setText(QString::number(rz, 'f', 1));
    }, Qt::QueuedConnection);
}


void MainWindow::slotGraspFinished() {
    QMetaObject::invokeMethod(this, [=]() {
        graspState_ = GraspState::Idle;
        updateButtonStates();
        appendLog("抓取流程已结束");
    }, Qt::QueuedConnection);
}

void MainWindow::slotCameraConnected(const std::shared_ptr<PercipioCamera>& cam) {
    camera = cameraWorker->getCamera();
    appendLog("相机已连接");
    btnConnectCamera->setChecked(true);

    if (robotWorker) {
        QMetaObject::invokeMethod(robotWorker, "setCamera", Qt::QueuedConnection,
                                  Q_ARG(std::shared_ptr<PercipioCamera>, cam));
    }

    btnConnectRobot->setEnabled(true);
    btnCaptureImage->setEnabled(true);
    updateButtonStates();
}

void MainWindow::slotCameraDisconnected() {
    appendLog("相机已断开", true);
    btnConnectCamera->setChecked(false);

    if (robotWorker) {
        QMetaObject::invokeMethod(robotWorker, "handleCameraDisconnected", Qt::QueuedConnection);

        if (btnConnectRobot->isChecked()) {
            appendLog("相机断开，正在断开机器人连接...", true);
            QMetaObject::invokeMethod(robotWorker, "disconnectRobot", Qt::QueuedConnection);
            btnConnectRobot->setChecked(false);
            if (graspState_ != GraspState::Idle) {
                onEmergencyStopClicked();
            }
        }
    }

    btnConnectRobot->setEnabled(false);
    btnCaptureImage->setEnabled(false);
    updateButtonStates();
}

void MainWindow::slotRobotConnected() {
    appendLog("机器人已连接");
    btnConnectRobot->setChecked(true);
    updateButtonStates();
}

void MainWindow::slotRobotDisconnected() {
    appendLog("机器人已断开");
    btnConnectRobot->setChecked(false);
    if (graspState_ != GraspState::Idle) {
        appendLog("机器人意外断开，停止当前任务。", true);
        graspState_ = GraspState::Idle;
    }
    updateButtonStates();
}
