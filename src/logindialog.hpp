#ifndef LOGINDIALOG_HPP
#define LOGINDIALOG_HPP

#include <QDialog>

QT_BEGIN_NAMESPACE
class QLabel;
class QLineEdit;
class QPushButton;
class QCheckBox;
class QVBoxLayout;
class QHBoxLayout;
QT_END_NAMESPACE

class LoginDialog : public QDialog
{
Q_OBJECT

public:

    explicit LoginDialog(QWidget *parent = nullptr);

    ~LoginDialog() override = default;

private Q_SLOTS:

            void onLoginClicked();

private:
    QLabel *leftImageLabel{};
    QLabel *welcomeLabel{};
    QLabel *promptLabel{};
    QLineEdit *usernameLineEdit{}; // 用户名输入框
    QLineEdit *passwordLineEdit{}; // 密码输入框
    QCheckBox *rememberCheckBox{}; // "记住密码"
    QPushButton *loginButton{};    // 登录按钮

    QHBoxLayout *mainLayout{};
    QVBoxLayout *rightLayout{};

    const QString correctUsername = "admin";
    const QString correctPassword = "123456";


    void setupUi();
};

#endif // LOGINDIALOG_HPP