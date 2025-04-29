#include "logindialog.hpp"

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout> // Include QHBoxLayout
#include <QMessageBox>
#include <QSpacerItem>
#include <QPixmap>
#include <QWidget> // Include QWidget

LoginDialog::LoginDialog(QWidget *parent)
        : QDialog(parent)
{
    setupUi();
    setWindowTitle("机器人登录界面");
    setFixedSize(700, 400);
}

void LoginDialog::setupUi() {
    // 左侧界面
    leftImageLabel = new QLabel(this);
    const int imageWidth = 240;
    const int imageHeight = 427;
    QSize imageSize(imageWidth, imageHeight);
    leftImageLabel->setFixedSize(imageSize);
    leftImageLabel->setStyleSheet("background-color: #f8f9fa; border-radius: 5px;");

    QPixmap originalPixmap(":/app_logo.png"); // Make sure this path is correct in your .qrc

    if (originalPixmap.isNull()) {
        qWarning("Failed to load image from resource: :/app_logo.png");
        leftImageLabel->setText("Image\nNot Found");
        leftImageLabel->setAlignment(Qt::AlignCenter);
        leftImageLabel->setStyleSheet("background-color: #ffe0e0; border: 1px solid red; border-radius: 5px;");
    } else {
        QPixmap scaledPixmap = originalPixmap.scaled(imageSize,
                                                     Qt::KeepAspectRatio,
                                                     Qt::SmoothTransformation);
        leftImageLabel->setPixmap(scaledPixmap);
        leftImageLabel->setAlignment(Qt::AlignCenter);
    }

    //  右侧界面
    welcomeLabel = new QLabel("<b>欢迎回来!</b>", this);
    promptLabel = new QLabel("请输入你的账号和密码以登录。", this);

    usernameLineEdit = new QLineEdit(this);
    usernameLineEdit->setPlaceholderText("用户名");
    usernameLineEdit->setText(correctUsername);

    passwordLineEdit = new QLineEdit(this);
    passwordLineEdit->setPlaceholderText("密码");
    passwordLineEdit->setEchoMode(QLineEdit::Password);
    passwordLineEdit->setText(correctPassword);

    rememberCheckBox = new QCheckBox("记住我", this);
    rememberCheckBox->setChecked(true);

    loginButton = new QPushButton("登录", this);
    loginButton->setCursor(Qt::PointingHandCursor);
    loginButton->setDefault(true);
    loginButton->setObjectName("loginButton");


    // 右侧布局
    rightLayout = new QVBoxLayout();
    rightLayout->addSpacerItem(new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding));
    rightLayout->addWidget(welcomeLabel);
    rightLayout->addSpacing(5);
    rightLayout->addWidget(promptLabel);
    rightLayout->addSpacing(30);
    rightLayout->addWidget(usernameLineEdit);
    rightLayout->addSpacing(15);
    rightLayout->addWidget(passwordLineEdit);
    rightLayout->addSpacing(20);
    rightLayout->addWidget(rememberCheckBox);
    rightLayout->addSpacing(30);
    rightLayout->addWidget(loginButton);
    rightLayout->addSpacerItem(new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding));

    auto *rightWidget = new QWidget(this);
    rightWidget->setObjectName("rightWidget"); // Name for styling
    rightWidget->setLayout(rightLayout);
    rightLayout->setContentsMargins(40, 30, 40, 30);


    // 主布局
    mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(leftImageLabel);
    mainLayout->addWidget(rightWidget, 1);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);


    // 样式表
    QString styleSheet = R"(
        QWidget#rightWidget {
            background-color: #ffffff; /* White background for the form area */
        }

        QLabel {
            color: #333333; /* Dark grey text */
        }

        QLabel#welcomeLabel {
            font-size: 22pt;
            font-weight: bold;
            margin-bottom: 2px;
        }

        /* Prompt Label */
        QLabel#promptLabel {
            font-size: 11pt;
            color: #777777;
            margin-bottom: 10px;
        }

        /* Line Edit Styles */
        QLineEdit {
            border: 1px solid #cccccc;
            border-radius: 4px;
            padding: 10px 8px;
            font-size: 11pt;
            background-color: #ffffff;
        }
        QLineEdit:focus {
            border: 1px solid #007bff;
        }
        QLineEdit::up-button, QLineEdit::down-button {
             subcontrol-origin: border;
             subcontrol-position: top right;
             width: 0px;
             border-left-width: 0px;
             border-left-color: darkgray;
             border-left-style: solid;
             border-top-right-radius: 3px;
             border-bottom-right-radius: 3px;
        }

        /* CheckBox Styles */
        QCheckBox {
            font-size: 10pt;
            color: #555555;
            spacing: 8px;
        }
        QCheckBox::indicator {
            width: 16px;
            height: 16px;
            border: 1px solid #cccccc;
            border-radius: 3px;
        }
        QCheckBox::indicator:unchecked {
            background-color: #ffffff;
        }
        QCheckBox::indicator:unchecked:hover {
            border: 1px solid #bbbbbb;
        }
        QCheckBox::indicator:checked {
            background-color: #007bff;
            border: 1px solid #007bff;
        }
        QCheckBox::indicator:checked:hover {
            background-color: #0056b3;
            border: 1px solid #0056b3;
        }

        /* Push Button Styles */
        QPushButton#loginButton {
            background-color: #007bff;
            color: white;
            font-size: 11pt;
            font-weight: bold;
            padding: 12px 20px;
            border: none;
            border-radius: 5px;
            min-height: 25px;
        }
        QPushButton#loginButton:hover {
            background-color: #0056b3;
        }
        QPushButton#loginButton:pressed {
            background-color: #004085;
        }
        QPushButton#loginButton:disabled {
            background-color: #d3d3d3;
            color: #aaaaaa;
        }
    )";

    welcomeLabel->setObjectName("welcomeLabel");
    promptLabel->setObjectName("promptLabel");

    this->setStyleSheet(styleSheet);

    connect(loginButton, &QPushButton::clicked, this, &LoginDialog::onLoginClicked);
    connect(passwordLineEdit, &QLineEdit::returnPressed, this, &LoginDialog::onLoginClicked);
    connect(usernameLineEdit, &QLineEdit::returnPressed, this, [this]() {
        passwordLineEdit->setFocus();
    });

    usernameLineEdit->setFocus();
}


void LoginDialog::onLoginClicked() {
    QString username = usernameLineEdit->text();
    QString password = passwordLineEdit->text();

    if (username == correctUsername && password == correctPassword) {
        QMessageBox::information(this, "登录成功", "欢迎回来, " + username + "!");
        accept();
    } else {
        QMessageBox::warning(this, "登录失败", "用户名或密码错误，请重试.");
        passwordLineEdit->clear();
        passwordLineEdit->setFocus();
    }
}