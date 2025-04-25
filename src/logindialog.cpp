#include "logindialog.hpp"
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>

LoginDialog::LoginDialog(QWidget* parent)
        : QDialog(parent)
{
    setWindowTitle("登录");
    setFixedSize(320, 180);

    // 创建用户名和密码输入框，并设置样式
    m_usernameLineEdit = new QLineEdit(this);
    m_passwordLineEdit = new QLineEdit(this);
    m_passwordLineEdit->setEchoMode(QLineEdit::Password);
    QString lineEditStyle = "QLineEdit {"
                            "  padding: 5px;"
                            "  border: 1px solid #bdc3c7;"
                            "  border-radius: 4px;"
                            "  font-size: 14px;"
                            "}";
    m_usernameLineEdit->setStyleSheet(lineEditStyle);
    m_passwordLineEdit->setStyleSheet(lineEditStyle);

    // 创建登录按钮并设置样式
    m_loginButton = new QPushButton("登录", this);
    QString buttonStyle = "QPushButton {"
                          "  background-color: #3498db;"
                          "  color: white;"
                          "  border: none;"
                          "  padding: 8px 16px;"
                          "  border-radius: 4px;"
                          "  font-size: 14px;"
                          "}"
                          "QPushButton:hover {"
                          "  background-color: #2980b9;"
                          "}";
    m_loginButton->setStyleSheet(buttonStyle);

    // 创建状态标签，用于显示错误信息
    m_statusLabel = new QLabel(this);
    m_statusLabel->setStyleSheet("QLabel { color: red; font-size: 12px; }");

    // 使用表单布局创建用户名和密码输入行
    QFormLayout* formLayout = new QFormLayout;
    formLayout->addRow("用户名:", m_usernameLineEdit);
    formLayout->addRow("密码:", m_passwordLineEdit);
    formLayout->setLabelAlignment(Qt::AlignRight);
    formLayout->setFormAlignment(Qt::AlignCenter);
    formLayout->setHorizontalSpacing(10);
    formLayout->setVerticalSpacing(8);

    // 创建按钮布局，登录按钮居中显示
    QHBoxLayout* buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_loginButton);
    buttonLayout->addStretch();

    // 主布局：垂直排列输入区域和按钮区域
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(formLayout);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(m_statusLabel, 0, Qt::AlignCenter);
    // 设置伸缩比例：第一部分占 3/4，第二部分占 1/4，状态标签不占伸缩空间
    mainLayout->setStretch(0, 3);
    mainLayout->setStretch(1, 1);
    mainLayout->setStretch(2, 0);
    mainLayout->setContentsMargins(20, 20, 20, 20);

    // 信号槽连接
    connect(m_loginButton, &QPushButton::clicked, this, &LoginDialog::attemptLogin);

    // 设置对话框整体背景色
    setStyleSheet("QDialog { background-color: #ecf0f1; }");
}

void LoginDialog::attemptLogin()
{
    // 使用硬编码的用户名和密码作为示例
    QString username = m_usernameLineEdit->text();
    QString password = m_passwordLineEdit->text();

    if(username == "admin" && password == "123456") {
        accept();  // 登录成功
    } else {
        m_statusLabel->setText("用户名或密码错误，请重试！");
        m_usernameLineEdit->clear();
        m_passwordLineEdit->clear();
    }
}

QString LoginDialog::username() const
{
    return m_usernameLineEdit->text();
}
